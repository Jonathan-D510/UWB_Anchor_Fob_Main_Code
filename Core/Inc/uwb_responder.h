#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Must be long enough for main initiator RX */
#ifndef UWB_RESP_DELAY_US
#define UWB_RESP_DELAY_US  3000u   /* 3ms */
#endif

void uwb_responder_init(uint8_t my_id);
void uwb_responder_step(void);

#ifdef __cplusplus
}
#endif
