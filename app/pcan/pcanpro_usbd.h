#pragma once
#include <stdint.h>
#include "at32_usbd_compat.h"

#define PCAN_USB_EP_CMDOUT          0x01U
#define PCAN_USB_EP_CMDIN           0x81U
#define PCAN_USB_EP_MSGOUT_CH1      0x02U
#define PCAN_USB_EP_MSGIN_CH1       0x82U
#define PCAN_USB_EP_MSGOUT_CH2      0x03U
#define PCAN_USB_EP_MSGIN_CH2       0x83U

#define PCAN_FS_MAX_BULK_PACKET_SIZE  64U
#define PCAN_HS_MAX_BULK_PACKET_SIZE  512U

#define PCAN_DATA_PACKET_SIZE         64U
#define PCAN_CMD_PACKET_SIZE          64U

#ifndef PCAN_LIN_INTERFACE_STR_INDEX
#define PCAN_LIN_INTERFACE_STR_INDEX  6U
#endif

struct t_class_data
{
  uint8_t  ep_tx_in_use[8];
  uint8_t  ep_tx_need_zlp[8];
  uint8_t  buffer_cmd[PCAN_DATA_PACKET_SIZE];
  uint8_t  buffer_data[PCAN_DATA_PACKET_SIZE];
  uint8_t  buffer_ch2[PCAN_DATA_PACKET_SIZE];
};

extern usbd_class_handler usbd_pcanpro;
extern volatile uint32_t pcan_bulk_out_rx_len;
