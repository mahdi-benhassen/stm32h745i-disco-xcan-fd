#ifndef USB_CONF_H
#define USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f45x.h"
#include "at32f45x_usb.h"

#define USE_OTG_DEVICE_MODE

#define USB_ID                           0U
#define OTG_CLOCK                        CRM_OTGFS1_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS1_IRQn
#define OTG_IRQ_HANDLER                  OTGFS1_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS1_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS1_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_18

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_DP                       GPIO_PINS_12
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE12
#define OTG_PIN_DM                       GPIO_PINS_11
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE11
#define OTG_PIN_VBUS                     GPIO_PINS_9
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE9
#define OTG_PIN_ID                       GPIO_PINS_10
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE10
#define OTG_PIN_MUX                      GPIO_MUX_10

#ifdef USE_OTG_DEVICE_MODE
/* OTGFS1 packet FIFO allocation in 32-bit words (292 words = 1168 bytes <= 1.25 KB) */
#define USBD_RX_SIZE                     128U
#define USBD_EP0_TX_SIZE                 24U
#define USBD_EP1_TX_SIZE                 20U
#define USBD_EP2_TX_SIZE                 20U
#define USBD_EP3_TX_SIZE                 20U
#define USBD_EP4_TX_SIZE                 20U
#define USBD_EP5_TX_SIZE                 20U
#define USBD_EP6_TX_SIZE                 20U
#define USBD_EP7_TX_SIZE                 20U

#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM                  8U
#endif
#endif

/* AT-START-F457 Type-C device connector omits dedicated VBUS sensing pin */
#define USB_VBUS_IGNORE

#define USBD_SUPPORT_WINUSB              0

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* USB_CONF_H */
