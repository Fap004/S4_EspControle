#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG_DATA_LEN 2

typedef struct {
    uint16_t seq;
    uint8_t  data[MSG_DATA_LEN];
    uint16_t crc;
} msg_t;

#ifdef __cplusplus
}
#endif
