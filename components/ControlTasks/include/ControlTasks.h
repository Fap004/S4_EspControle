#pragma once
#include "AppContext.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_control_task(AppContext* ctx, UBaseType_t prio);

#ifdef __cplusplus
}
#endif