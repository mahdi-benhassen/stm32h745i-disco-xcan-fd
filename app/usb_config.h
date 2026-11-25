#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

#include "board.h"

#ifdef __RTTHREAD__
#include <rtthread.h>
#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)
#else
#include <stdio.h>
#define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)
#endif

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_ERROR
#endif

/* HPM5361 USB0 embeds a native USB 2.0 High-Speed (480 Mbps) PHY */
#define CONFIG_USB_HS

#ifdef CONFIG_USB_DCACHE_ENABLE
#define CONFIG_USB_ALIGN_SIZE HPM_L1C_CACHELINE_SIZE
#else
#define CONFIG_USB_ALIGN_SIZE 4
#endif

#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable.non_init")))

#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

#define CONFIG_USBDEV_TEST_MODE
#define CONFIG_USBDEV_ADVANCE_DESC

#define CONFIG_USBDEV_MAX_BUS USB_SOC_MAX_COUNT

#ifndef CONFIG_HPM_USBD_BASE
#define CONFIG_HPM_USBD_BASE HPM_USB0_BASE
#endif
#ifndef CONFIG_HPM_USBD_IRQn
#define CONFIG_HPM_USBD_IRQn IRQn_USB0
#endif

#ifndef usb_phyaddr2ramaddr
#define usb_phyaddr2ramaddr(addr) core_local_mem_to_sys_address(BOARD_RUNNING_CORE, addr)
#endif

#ifndef usb_ramaddr2phyaddr
#define usb_ramaddr2phyaddr(addr) sys_address_to_core_local_mem(BOARD_RUNNING_CORE, addr)
#endif

#endif /* CHERRYUSB_CONFIG_H */
