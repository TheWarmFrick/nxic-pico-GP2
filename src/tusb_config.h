#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// Root hub ports
//   RHPort0 : native USB  (USB-C)  -> Switch dock (device: Pro Controller)
//   RHPort1 : PIO-USB GP12/GP13 (USB-A) -> hub + keyboard/mouse (host)
//--------------------------------------------------------------------
#define BOARD_TUD_RHPORT      0
#define BOARD_TUH_RHPORT      1

#define CFG_TUD_ENABLED       1
#define CFG_TUH_ENABLED       1
#define CFG_TUH_RPI_PIO_USB   1

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU          OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN    __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// Device (Pro Controller emulation)
//--------------------------------------------------------------------
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_HID               1
#define CFG_TUD_HID_EP_BUFSIZE    64

#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

// CDC-NCM network interface (only enumerated in web config mode)
#define CFG_TUD_NCM               1

//--------------------------------------------------------------------
// Host (keyboard / mouse, behind a hub)
//--------------------------------------------------------------------
// Large enough for the long report descriptors of gaming mice
#define CFG_TUH_ENUMERATION_BUFSIZE 1024

#define CFG_TUH_HUB               4
#define CFG_TUH_DEVICE_MAX        8

#define CFG_TUH_HID               8
#define CFG_TUH_HID_EPIN_BUFSIZE  64
#define CFG_TUH_HID_EPOUT_BUFSIZE 64

#ifdef __cplusplus
}
#endif

#endif // _TUSB_CONFIG_H_
