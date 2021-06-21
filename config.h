#ifndef CONFIG_H
#define CONFIG_H

#include "user_config.h"

#ifndef USB_VID
#define USB_VID 0xdead
#endif

#ifndef USB_PID
#define USB_PID 0xca5d
#endif

#ifndef USB_MANUFACTURER
#define USB_MANUFACTURER "davidgf.net (libopencm3 based)"
#endif

#ifndef USB_PRODUCT
#define USB_PRODUCT "DFU bootloader [" VERSION "]"
#endif

#endif //CONFIG_H
