#pragma once
#include "at32_usbd_compat.h"

extern usbd_desc_handler pcan_at32_usbd_desc;
void pcan_usbd_encode_utf16_str(const char *ascii, uint8_t *out, uint16_t max_bytes, uint16_t *out_len);
