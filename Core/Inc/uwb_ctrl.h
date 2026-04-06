#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Command protocol bytes */
#define UWB_CMD_REQ   0xC0
#define UWB_CMD_RESP  0xC1

#define UWB_CMD_RANGE_FOB  0x01

/* Sends "range FOB" request to an anchor and waits for response.
   Returns 0 on success, negative on timeout/error.
   out_m: meters
   out_status: responder status byte (0=OK)
*/
int uwb_ctrl_range_fob_via_anchor(uint8_t main_id,
                                  uint8_t anchor_id,
                                  uint8_t *io_seq,
                                  float *out_m,
                                  uint8_t *out_status,
                                  uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
