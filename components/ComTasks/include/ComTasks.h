#pragma once
#include "AppContext.h"
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void start_rx_task(AppContext* ctx, UBaseType_t prio,
                   const uint8_t peer_mac[6], uint8_t channel);

void start_tx_task(AppContext* ctx, UBaseType_t prio,
                   const uint8_t peer_mac[6]);

#ifdef __cplusplus
}
#endif