#pragma once
#include <stdint.h>
#include "hpm_usbd_compat.h"

#define PCAN_USB_BUFFER_CMD   (0)
#define PCAN_USB_BUFFER_DATA  (1)

void pcan_protocol_init(void);
void pcan_protocol_poll(void);
void pcan_protocol_process_data(uint8_t ep, uint8_t *ptr, uint16_t size);
void pcan_ep0_receive_data(const uint8_t *buf, uint16_t len);
int pcan_protocol_can_rx_has_space(void);
uint8_t pcan_protocol_device_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
