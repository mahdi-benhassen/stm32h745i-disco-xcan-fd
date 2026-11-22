#pragma once

/*
 * Isolated USB identity configuration.
 * Keep open-source defaults in the public repository; override locally via
 * CMake -D flags or local edits when testing against vendor drivers.
 */

#ifndef PCAN_USB_VID
#define PCAN_USB_VID              0x0483U
#endif

#ifndef PCAN_USB_PID
#define PCAN_USB_PID              0x0011U
#endif

#ifndef PCAN_USB_MANUFACTURER_STR
#define PCAN_USB_MANUFACTURER_STR "XCAN"
#endif

#ifndef PCAN_USB_PRODUCT_STR
#define PCAN_USB_PRODUCT_STR      "XCAN-USB Pro FD"
#endif

#ifndef PCAN_USB_CONFIG_STR
#define PCAN_USB_CONFIG_STR       "Config00"
#endif

#ifndef PCAN_USB_IF_CAN_STR
#define PCAN_USB_IF_CAN_STR       "XCAN-USB Pro FD CAN"
#endif

#ifndef PCAN_USB_IF_LIN_STR
#define PCAN_USB_IF_LIN_STR       "XCAN-USB Pro FD LIN"
#endif
