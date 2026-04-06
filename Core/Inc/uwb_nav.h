#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uwb_nav_init(uint8_t my_id);
void uwb_nav_step(void);

#ifdef __cplusplus
}
#endif
