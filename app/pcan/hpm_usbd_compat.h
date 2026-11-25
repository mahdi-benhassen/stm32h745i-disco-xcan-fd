#ifndef HPM_USBD_COMPAT_H
#define HPM_USBD_COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "hpm_soc.h"
#include "hpm_soc_feature.h"
#include "hpm_romapi.h"
#include "usbd_core.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef USBD_OK
#define USBD_OK   (0U)
#endif

#ifndef USBD_FAIL
#define USBD_FAIL (1U)
#endif

#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif

#ifndef __ALIGN_END
#define __ALIGN_END USB_MEM_ALIGNX
#endif

typedef struct
{
  uint8_t dev_state;
  void   *pClassData;
} USBD_HandleTypeDef;

/* Matches CherryUSB struct usb_setup_packet memory layout for direct cast */
typedef struct usb_setup_packet USBD_SetupReqTypedef;

static inline uint32_t hpm_get_uid_w0(void)
{
  return ROM_API_TABLE_ROOT->otp_driver_if->read_from_shadow(OTP_SOC_UUID_IDX);
}

uint8_t USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
uint8_t USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
uint8_t USBD_CtlSendStatus(USBD_HandleTypeDef *pdev);
void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

#ifdef __cplusplus
}
#endif

#endif /* HPM_USBD_COMPAT_H */
