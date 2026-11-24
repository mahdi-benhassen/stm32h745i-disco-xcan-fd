#ifndef AT32_USBD_COMPAT_H
#define AT32_USBD_COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_core.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN ALIGNED_HEAD
#endif

#ifndef __ALIGN_END
#define __ALIGN_END ALIGNED_TAIL
#endif

#ifndef USBD_OK
#define USBD_OK USB_OK
#endif

#ifndef USBD_FAIL
#define USBD_FAIL USB_FAIL
#endif

#ifndef USB_LEN_DEV_DESC
#define USB_LEN_DEV_DESC 0x12U
#endif

#ifndef USB_LEN_LANGID_STR_DESC
#define USB_LEN_LANGID_STR_DESC 0x04U
#endif

typedef usbd_core_type USBD_HandleTypeDef;
typedef usb_setup_type USBD_SetupReqTypedef;

static inline uint32_t at32_get_uid_w0(void)
{
  return *(const volatile uint32_t *)0x1FFFF7E8UL;
}

static inline void USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len)
{
  usbd_ctrl_send(pdev, pbuf, len);
}

static inline void USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len)
{
  usbd_ctrl_recv(pdev, pbuf, len);
}

static inline void USBD_CtlSendStatus(USBD_HandleTypeDef *pdev)
{
  /* AT32 usbd_interface_request() auto-sends ZLP when wLength == 0 && ret == USB_OK */
  if ((pdev->setup.bmRequestType & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
  {
    usbd_ctrl_send_status(pdev);
  }
}

static inline void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  UNUSED(req);
  usbd_ctrl_unsupport(pdev);
}

#ifdef __cplusplus
}
#endif

#endif /* AT32_USBD_COMPAT_H */
