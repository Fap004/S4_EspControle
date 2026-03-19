#pragma once
#include "freertos/FreeRTOS.h"  // ← UBaseType_t vient d'ici
#include "AppContext.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_control_task(AppContext* ctx, UBaseType_t prio);

#ifdef __cplusplus
}
#endif