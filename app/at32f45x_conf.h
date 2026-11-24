#ifndef AT32F45X_CONF_H
#define AT32F45X_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(HEXT_VALUE)
#define HEXT_VALUE                       ((uint32_t)8000000)
#endif

#define HEXT_STARTUP_TIMEOUT             ((uint16_t)0x3000)
#define HICK_VALUE                       ((uint32_t)8000000)
#define LEXT_VALUE                       ((uint32_t)32768)

/* Enabled AT32F45x peripheral drivers */
#define CAN_MODULE_ENABLED
#define CRM_MODULE_ENABLED
#define EXINT_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define SCFG_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define USB_MODULE_ENABLED

#ifdef CAN_MODULE_ENABLED
#include "at32f45x_can.h"
#endif
#ifdef CRM_MODULE_ENABLED
#include "at32f45x_crm.h"
#endif
#ifdef EXINT_MODULE_ENABLED
#include "at32f45x_exint.h"
#endif
#ifdef FLASH_MODULE_ENABLED
#include "at32f45x_flash.h"
#endif
#ifdef GPIO_MODULE_ENABLED
#include "at32f45x_gpio.h"
#endif
#ifdef MISC_MODULE_ENABLED
#include "at32f45x_misc.h"
#endif
#ifdef PWC_MODULE_ENABLED
#include "at32f45x_pwc.h"
#endif
#ifdef SCFG_MODULE_ENABLED
#include "at32f45x_scfg.h"
#endif
#ifdef TMR_MODULE_ENABLED
#include "at32f45x_tmr.h"
#endif
#ifdef USART_MODULE_ENABLED
#include "at32f45x_usart.h"
#endif
#ifdef USB_MODULE_ENABLED
#include "at32f45x_usb.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* AT32F45X_CONF_H */
