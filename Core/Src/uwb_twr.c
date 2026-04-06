#include "uwb_twr.h"
#include "deca_device_api.h"
#include "uart.h"
#include "uwb_ids.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define FCODE_POLL   0xE0
#define FCODE_RESP   0xE1

#define SPEED_OF_LIGHT 299702547.0f
#define U40_MASK      ((uint64_t)0xFFFFFFFFFFULL)
#define RX_WAIT_MS    200u
#define MAX_ATTEMPTS  3u

/* Base correction for any initiator ranging to the FOB */
#define FOB_TREPLY_CORR_DTU        240ULL

/* Extra correction only when Anchor 1 ranges to FOB */
#define A1_TO_FOB_EXTRA_CORR_DTU   260ULL

static uint8_t  g_my_id = 0;
static uint8_t  g_seq   = 0;

static uint64_t get_u40(const uint8_t *p)
{
    uint64_t v = 0;
    v |= (uint64_t)p[0];
    v |= (uint64_t)p[1] << 8;
    v |= (uint64_t)p[2] << 16;
    v |= (uint64_t)p[3] << 24;
    v |= (uint64_t)p[4] << 32;
    return v;
}

static uint64_t read_tx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    return get_u40(ts);
}

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return get_u40(ts);
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

static uint64_t u40_diff(uint64_t a, uint64_t b)
{
    return (a - b) & U40_MASK;
}

static uint64_t apply_target_correction(uint8_t target_id, uint64_t treply_u40)
{
    uint64_t corr = 0;

    if (target_id == UWB_ID_FOB)
    {
        corr += FOB_TREPLY_CORR_DTU;

        /* Extra help only for Anchor 1 -> FOB */
        if (g_my_id == UWB_ID_A1)
        {
            corr += A1_TO_FOB_EXTRA_CORR_DTU;
        }

        if (treply_u40 > corr)
            return treply_u40 - corr;
        else
            return 0;
    }

    return treply_u40;
}

void uwb_twr_init(uint8_t my_id)
{
    g_my_id = my_id;
    g_seq = 0;

    dwt_forcetrxoff();
    clear_status_all();

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    dbg_printf("TWR: init my_id=%u\r\n", (unsigned)g_my_id);
}

int uwb_twr_range_to(uint8_t target_id, float *out_m)
{
    uint8_t attempt;

    if (out_m) *out_m = NAN;

    for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
    {
        uint8_t poll[4];
        uint8_t rxbuf[64];
        uint8_t want_seq;
        uint64_t t1, t2, t3, t4;
        uint64_t Tround_u40, Treply_u40, Treply_corr_u40;
        double tof_dtu, tof_s;
        float dist_m;
        uint32_t t0;

        poll[0] = FCODE_POLL;
        poll[1] = g_my_id;
        poll[2] = target_id;
        poll[3] = g_seq;
        want_seq = g_seq;
        g_seq++;

        dwt_forcetrxoff();
        clear_status_all();

        if (dwt_writetxdata(sizeof(poll), poll, 0) != DWT_SUCCESS)
            return -11;

        dwt_writetxfctrl((uint16_t)(sizeof(poll) + 2U), 0, 0);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
            return -10;

        while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) { }

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK |
                             DWT_INT_TXPHS_BIT_MASK |
                             DWT_INT_TXPRS_BIT_MASK |
                             DWT_INT_TXFRB_BIT_MASK);

        t1 = read_tx_ts40();

        rx_restart();

        t0 = HAL_GetTick();
        for (;;)
        {
            uint32_t st_lo = dwt_readsysstatuslo();

            if (st_lo & rx_err_mask_lo())
            {
                dwt_writesysstatuslo(rx_err_mask_lo());
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }

            if (st_lo & DWT_INT_RXFCG_BIT_MASK)
            {
                uint16_t len;

                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

                len = dwt_getframelength(NULL);
                if (len > sizeof(rxbuf)) len = sizeof(rxbuf);
                dwt_readrxdata(rxbuf, len, 0);

                t4 = read_rx_ts40();

                if (len < (4 + 5 + 5))
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    goto check_timeout;
                }

                if (rxbuf[0] != FCODE_RESP)
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    goto check_timeout;
                }

                if (rxbuf[1] != target_id || rxbuf[2] != g_my_id || rxbuf[3] != want_seq)
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    goto check_timeout;
                }

                t2 = get_u40(&rxbuf[4]);
                t3 = get_u40(&rxbuf[9]);

                Tround_u40 = u40_diff(t4, t1);
                Treply_u40 = u40_diff(t3, t2);
                Treply_corr_u40 = apply_target_correction(target_id, Treply_u40);

                tof_dtu = ((double)Tround_u40 - (double)Treply_corr_u40) * 0.5;

                if (!(tof_dtu > 0.0))
                {
                    dbg_printf("TWR: bad tof me=%u target=%u seq=%u att=%u "
                               "t1=%lu t2=%lu t3=%lu t4=%lu "
                               "Tround=%lu Treply=%lu TreplyC=%lu diff=%ld tof_dtu=%.1f\r\n",
                               (unsigned)g_my_id,
                               (unsigned)target_id,
                               (unsigned)want_seq,
                               (unsigned)attempt,
                               (unsigned long)t1,
                               (unsigned long)t2,
                               (unsigned long)t3,
                               (unsigned long)t4,
                               (unsigned long)Tround_u40,
                               (unsigned long)Treply_u40,
                               (unsigned long)Treply_corr_u40,
                               (long)((int64_t)Tround_u40 - (int64_t)Treply_corr_u40),
                               tof_dtu);

                    dwt_forcetrxoff();
                    clear_status_all();
                    break;
                }

                tof_s = tof_dtu * (double)DWT_TIME_UNITS;
                dist_m = (float)(tof_s * (double)SPEED_OF_LIGHT);

                /* Allow short ranges while tuning FOB path */
                if (!isfinite(dist_m) || dist_m <= 0.01f || dist_m > 100.0f)
                {
                    dbg_printf("TWR: bad dist me=%u target=%u seq=%u att=%u "
                               "dist=%.3f m tof_dtu=%.1f "
                               "Tround=%lu Treply=%lu TreplyC=%lu\r\n",
                               (unsigned)g_my_id,
                               (unsigned)target_id,
                               (unsigned)want_seq,
                               (unsigned)attempt,
                               (double)dist_m,
                               tof_dtu,
                               (unsigned long)Tround_u40,
                               (unsigned long)Treply_u40,
                               (unsigned long)Treply_corr_u40);

                    dwt_forcetrxoff();
                    clear_status_all();
                    break;
                }

                if (out_m) *out_m = dist_m;
                return 0;
            }

check_timeout:
            if ((HAL_GetTick() - t0) > RX_WAIT_MS)
            {
                dwt_forcetrxoff();
                clear_status_all();
                break;
            }
        }
    }

    return -21;
}
