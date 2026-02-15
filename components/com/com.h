#pragma once
#include <stdint.h>
#include <stddef.h>

void com_init();
void com_add_peer(const uint8_t mac[6]);
void com_send(const uint8_t mac[6], const uint8_t *data, size_t len);
