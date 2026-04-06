#include "uwb_ctrl.h"
#include "deca_device_api.h"
#include "stm32g4xx_hal.h"
#include "uart.h"
#include <string.h>

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

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void rx_restart(void)
{
    dwt_forcetrxoff();
    clear_status_all();
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

int uwb_ctrl_range_fob_via_anchor(uint8_t main_id,
                                  uint8_t anchor_id,
                                  uint8_t *io_seq,
                                  float *out_m,
                                  uint8_t *out_status,
                                  uint32_t timeout_ms)
{
    if (!io_seq) return -1;
    const uint8_t seq = *io_seq;

    uint8_t req[8] = {0};
    req[0] = UWB_CMD_REQ;
    req[1] = main_id;
    req[2] = anchor_id;
    req[3] = seq;
    req[4] = UWB_CMD_RANGE_FOB;

    /* TX request */
    dwt_forcetrxoff();
    clear_status_all();

    if (dwt_writetxdata(sizeof(req), req, 0) != DWT_SUCCESS) return -2;
    dwt_writetxfctrl((uint16_t)(sizeof(req) + 2U), 0, 0);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) return -3;

    while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) { }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    /* RX loop: ignore unrelated frames until we get the right response or timeout */
    rx_restart();

    uint32_t t0 = HAL_GetTick();
    for (;;)
    {
        uint32_t st = dwt_readsysstatuslo();

        if (st & rx_err_mask_lo())
        {
            dwt_writesysstatuslo(rx_err_mask_lo());
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if (st & DWT_INT_RXFCG_BIT_MASK)
        {
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            uint8_t rxbuf[32] = {0};
            uint16_t len = dwt_getframelength(NULL);
            if (len > sizeof(rxbuf)) len = sizeof(rxbuf);
            dwt_readrxdata(rxbuf, len, 0);

            /* If you want: uncomment to see what we're actually catching
            dbg_printf("CTRL: rx len=%u b0=%02X b1=%02X b2=%02X b3=%02X\r\n",
                       (unsigned)len, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
            */

            /* Ignore too-short frames (e.g., 4B poll + CRC) */
            if (len < 12)
            {
                // dbg_printf("CTRL: short len=%u (ignored)\r\n",(unsigned)len);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                goto check_timeout;
            }

            /* Must be CMD response */
            if (rxbuf[0] != UWB_CMD_RESP) { dwt_rxenable(DWT_START_RX_IMMEDIATE); goto check_timeout; }
            if (rxbuf[1] != anchor_id)    { dwt_rxenable(DWT_START_RX_IMMEDIATE); goto check_timeout; }
            if (rxbuf[2] != main_id)      { dwt_rxenable(DWT_START_RX_IMMEDIATE); goto check_timeout; }
            if (rxbuf[3] != seq)          { dwt_rxenable(DWT_START_RX_IMMEDIATE); goto check_timeout; }

            uint32_t mm = get_u32_le(&rxbuf[4]);
            uint8_t status = rxbuf[8];

            if (out_status) *out_status = status;
            if (out_m) *out_m = ((float)mm) * 0.001f;

            *io_seq = (uint8_t)(seq + 1);
            return 0;
        }

check_timeout:
        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            dbg_printf("CTRL: timeout waiting CMD_RESP from anchor_id=%u seq=%u\r\n",
                       (unsigned)anchor_id, (unsigned)seq);
            dwt_forcetrxoff();
            clear_status_all();
            return -10;
        }
    }
}
