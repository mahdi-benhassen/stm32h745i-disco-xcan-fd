#pragma once
#include <stdint.h>
#include "usbd_def.h"

#define PCAN_FS_MAX_BULK_PACKET_SIZE  (64U)
#define PCAN_HS_MAX_BULK_PACKET_SIZE  (512U)

/* PCAN-USB Endpoints */
#define PCAN_USB_EP_CMDOUT            0x01U
#define PCAN_USB_EP_CMDIN             0x81U
#define PCAN_USB_EP_MSGOUT_CH1        0x02U
#define PCAN_USB_EP_MSGIN_CH1         0x82U
#define PCAN_USB_EP_MSGOUT_CH2        0x03U
#define PCAN_USB_EP_MSGIN_CH2         0x83U

#if defined(PCAN_PRO_FD) || defined(PCAN_FD) || defined(PCAN_X6)
#define PCAN_DATA_PACKET_SIZE         (PCAN_FS_MAX_BULK_PACKET_SIZE)
#define PCAN_CMD_PACKET_SIZE          (PCAN_FS_MAX_BULK_PACKET_SIZE)
#elif defined(PCAN_PRO)
#define PCAN_DATA_PACKET_SIZE         (64U)
#define PCAN_CMD_PACKET_SIZE          (512U)
#else
#error "Select one of PCAN_PRO_FD, PCAN_FD, PCAN_X6, or PCAN_PRO"
#endif

struct t_class_data
{
  int tx_flow_control_in_use;
  uint8_t ep_tx_in_use[16];
  uint8_t cmd_ep_buffer[PCAN_CMD_PACKET_SIZE];
  uint8_t data1_ep_buffer[PCAN_DATA_PACKET_SIZE];
  uint8_t data2_ep_buffer[PCAN_DATA_PACKET_SIZE];
};

extern USBD_ClassTypeDef usbd_pcanpro;
