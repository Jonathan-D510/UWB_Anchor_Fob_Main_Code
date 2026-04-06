#include "uwb_responder.h"
#include "uwb_twr.h"
#include "deca_device_api.h"
#include "uart.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define FCODE_POLL   0xE0
#define FCODE_RESP   0xE1

#define UWB_CMD_REQ       0xC0
#define UWB_CMD_RESP      0xC1
#define UWB_CMD_RANGE_FOB 0x01

#define UWB_ID_FOB  1

/* 500 us was too short for delayed TX on this path */
#define UWB_RESP_DELAY_US  3000U

static uint8_t  g_my_id = 0;
static uint32_t g_rx_ok = 0;
static uint32_t g_rx_err = 0;
static uint32_t g_tx_ok = 0;
static uint32_t g_tx_fail = 0;
static uint8_t  g_last_src = 0xFF;
static uint8_t  g_last_seq = 0xFF;
static uint32_t g_last_hb_ms = 0;

static uint64_t u40_from_le(const uint8_t *p)
{
    uint64_t v = 0;
    v |= (uint64_t)p[0];
    v |= (uint64_t)p[1] << 8;
    v |= (uint64_t)p[2] << 16;
    v |= (uint64_t)p[3] << 24;
    v |= (uint64_t)p[4] << 32;
    return v;
}

static void u40_to_le(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
    p[4] = (uint8_t)((v >> 32) & 0xFF);
}

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return u40_from_le(ts);
}

static uint64_t us_to_dtu(uint32_t us)
{
    double dtu_per_s = 1.0 / DWT_TIME_UNITS;
    double dtu = ((double)us) * 1e-6 * dtu_per_s;
    return (uint64_t)(dtu + 0.5);
}

static void clear_status_all(void)
{
    dwt_writesysstatuslo(0xFFFFFFFFu);
    dwt_writesysstatushi(0xFFFFFFFFu);
}

static uint32_t rx_err_mask_lo(void)
{
    return (DWT_INT_RXPHE_BIT_MASK |
            DWT_INT_RXFCE_BIT_MASK |
            DWT_INT_RXFSL_BIT_MASK |
            DWT_INT_RXFTO_BIT_MASK |
            DWT_INT_RXPTO_BIT_MASK |
            DWT_INT_RXSTO_BIT_MASK |
            DWT_INT_ARFE_BIT_MASK  |
            DWT_INT_CIAERR_BIT_MASK);
}

static void rx_restart(void)
{
    dwt_forcetrxoff();
    clear_status_all();
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void heartbeat_1hz(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_last_hb_ms) >= 1000u)
    {
        g_last_hb_ms = now;
        dbg_printf("ANCH alive id=%u: rx_ok=%lu tx_ok=%lu rx_err=%lu tx_fail=%lu last_src=%u last_seq=%u\r\n",
                   (unsigned)g_my_id,
                   (unsigned long)g_rx_ok,
                   (unsigned long)g_tx_ok,
                   (unsigned long)g_rx_err,
                   (unsigned long)g_tx_fail,
                   (unsigned)g_last_src,
                   (unsigned)g_last_seq);
    }
}

static void tx_wait_done(void)
{
    while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) { }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

void uwb_responder_init(uint8_t my_id)
{
    g_my_id = my_id;
    g_rx_ok = g_rx_err = g_tx_ok = g_tx_fail = 0;
    g_last_src = 0xFF;
    g_last_seq = 0xFF;
    g_last_hb_ms = HAL_GetTick();

    dwt_forcetrxoff();
    clear_status_all();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    uwb_twr_init(g_my_id);

    dbg_printf("ANCH: init my_id=%u (POLL+CMD) RESP_DELAY_US=%u\r\n",
               (unsigned)g_my_id, (unsigned)UWB_RESP_DELAY_US);
    rx_restart();
}

void uwb_responder_step(void)
{
    uint32_t st_lo;
    uint8_t rxbuf[64] = {0};
    uint16_t len;

    heartbeat_1hz();

    st_lo = dwt_readsysstatuslo();

    if (st_lo & rx_err_mask_lo())
    {
        g_rx_err++;
        dwt_writesysstatuslo(rx_err_mask_lo());
        rx_restart();
        return;
    }

    if ((st_lo & DWT_INT_RXFCG_BIT_MASK) == 0)
        return;

    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    len = dwt_getframelength(NULL);
    if (len > sizeof(rxbuf)) len = sizeof(rxbuf);
    dwt_readrxdata(rxbuf, len, 0);

    if (len < 4)
    {
        g_rx_err++;
        rx_restart();
        return;
    }

    if (rxbuf[0] == UWB_CMD_REQ)
    {
        uint8_t main_id;
        uint8_t anchor_id;
        uint8_t seq;
        uint8_t cmd;
        uint8_t status = 0;
        uint32_t mm = 0;
        uint8_t resp[12] = {0};

        if (len < 5)
        {
            rx_restart();
            return;
        }

        main_id   = rxbuf[1];
        anchor_id = rxbuf[2];
        seq       = rxbuf[3];
        cmd       = rxbuf[4];

        if (anchor_id != g_my_id)
        {
            rx_restart();
            return;
        }

        g_rx_ok++;
        g_last_src = main_id;
        g_last_seq = seq;

        dbg_printf("ANCH%u CMD_REQ: from=%u seq=%u cmd=0x%02X len=%u\r\n",
                   (unsigned)g_my_id, (unsigned)main_id, (unsigned)seq,
                   (unsigned)cmd, (unsigned)len);

        if (cmd == UWB_CMD_RANGE_FOB)
        {
            float d = NAN;
            int rc = uwb_twr_range_to(UWB_ID_FOB, &d);

            if (rc == 0 && isfinite(d) && d > 0.05f && d < 100.0f)
            {
                mm = (uint32_t)(d * 1000.0f + 0.5f);
                status = 0;
                dbg_printf("ANCH%u CMD: FOB range OK rc=%d d=%.3f m mm=%lu\r\n",
                           (unsigned)g_my_id, rc, (double)d, (unsigned long)mm);
            }
            else
            {
                status = 1;
                mm = 0;
                dbg_printf("ANCH%u CMD: FOB range FAIL rc=%d d=%.3f\r\n",
                           (unsigned)g_my_id, rc, (double)d);
            }
        }
        else
        {
            status = 2;
            mm = 0;
            dbg_printf("ANCH%u CMD: unknown cmd=0x%02X\r\n",
                       (unsigned)g_my_id, (unsigned)cmd);
        }

        resp[0] = UWB_CMD_RESP;
        resp[1] = g_my_id;
        resp[2] = main_id;
        resp[3] = seq;
        put_u32_le(&resp[4], mm);
        resp[8] = status;

        dwt_forcetrxoff();
        clear_status_all();

        if (dwt_writetxdata(sizeof(resp), resp, 0) != DWT_SUCCESS)
        {
            g_tx_fail++;
            dbg_printf("ANCH%u CMD_RESP: write fail\r\n", (unsigned)g_my_id);
            rx_restart();
            return;
        }

        dwt_writetxfctrl((uint16_t)(sizeof(resp) + 2U), 0, 0);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
        {
            g_tx_fail++;
            dbg_printf("ANCH%u CMD_RESP: starttx fail\r\n", (unsigned)g_my_id);
            rx_restart();
            return;
        }

        tx_wait_done();
        g_tx_ok++;

        dbg_printf("ANCH%u CMD_RESP: to=%u seq=%u status=%u mm=%lu\r\n",
                   (unsigned)g_my_id, (unsigned)main_id, (unsigned)seq,
                   (unsigned)status, (unsigned long)mm);

        rx_restart();
        return;
    }

    if (rxbuf[0] != FCODE_POLL)
    {
        rx_restart();
        return;
    }

    {
        uint8_t src = rxbuf[1];
        uint8_t dst = rxbuf[2];
        uint8_t seq = rxbuf[3];
        uint64_t t2;
        uint64_t t3;
        uint8_t resp[4 + 5 + 5] = {0};
        int rc;

        if (dst != g_my_id)
        {
            rx_restart();
            return;
        }

        g_rx_ok++;
        g_last_src = src;
        g_last_seq = seq;

        t2 = read_rx_ts40();
        t3 = t2 + us_to_dtu(UWB_RESP_DELAY_US);
        t3 &= ~((uint64_t)0x1FF);

        resp[0] = FCODE_RESP;
        resp[1] = g_my_id;
        resp[2] = src;
        resp[3] = seq;
        u40_to_le(&resp[4], t2);
        u40_to_le(&resp[9], t3);

        dwt_forcetrxoff();
        clear_status_all();

        dwt_setdelayedtrxtime((uint32_t)(t3 >> 8));

        if (dwt_writetxdata(sizeof(resp), resp, 0) != DWT_SUCCESS)
        {
            g_tx_fail++;
            rx_restart();
            return;
        }

        dwt_writetxfctrl((uint16_t)(sizeof(resp) + 2U), 0, 0);

        rc = dwt_starttx(DWT_START_TX_DELAYED);
        if (rc != DWT_SUCCESS)
        {
            g_tx_fail++;
            dbg_printf("ANCH%u POLL_RESP: delayed TX fail rc=%d seq=%u\r\n",
                       (unsigned)g_my_id, rc, (unsigned)seq);
            rx_restart();
            return;
        }

        tx_wait_done();
        g_tx_ok++;

        rx_restart();
    }
}
