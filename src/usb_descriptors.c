// USB descriptors.
//   Controller mode: Nintendo Switch Pro Controller (VID 057E / PID 2009)
//   Web config mode: CDC-NCM network adapter
// The mode is selected via `usb_config_mode` before tud_init().

#include <stdio.h>

#include "tusb.h"
#include "pico/unique_id.h"

#define USB_VID 0x057E
#define USB_PID 0x2009

#define CFGMODE_VID 0x2E8A // Raspberry Pi
#define CFGMODE_PID 0x10C0

bool usb_config_mode = false;
extern uint8_t tud_network_mac_address[6]; // config_mode.c

//--------------------------------------------------------------------
// Device descriptor
//--------------------------------------------------------------------
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0200,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01,
};

static tusb_desc_device_t const desc_device_cfgmode = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = CFGMODE_VID,
    .idProduct          = CFGMODE_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return usb_config_mode ? (uint8_t const *)&desc_device_cfgmode
                           : (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------
// HID report descriptor (from a real Pro Controller USB capture)
//--------------------------------------------------------------------
static uint8_t const desc_hid_report[] = {
    0x05, 0x01,                   // Usage Page (Generic Desktop)
    0x15, 0x00,                   // Logical Minimum (0)
    0x09, 0x04,                   // Usage (Joystick)
    0xA1, 0x01,                   // Collection (Application)
    0x85, 0x30,                   //   Report ID (0x30) - full input report
    0x05, 0x01,                   //   Usage Page (Generic Desktop)
    0x05, 0x09,                   //   Usage Page (Button)
    0x19, 0x01,                   //   Usage Minimum (1)
    0x29, 0x0A,                   //   Usage Maximum (10)
    0x15, 0x00, 0x25, 0x01,       //   Logical 0..1
    0x75, 0x01, 0x95, 0x0A,       //   10 bits
    0x55, 0x00, 0x65, 0x00,       //   Unit exponent / unit
    0x81, 0x02,                   //   Input (Data,Var,Abs)
    0x05, 0x09,                   //   Usage Page (Button)
    0x19, 0x0B,                   //   Usage Minimum (11)
    0x29, 0x0E,                   //   Usage Maximum (14)
    0x15, 0x00, 0x25, 0x01,       //   Logical 0..1
    0x75, 0x01, 0x95, 0x04,       //   4 bits
    0x81, 0x02,                   //   Input (Data,Var,Abs)
    0x75, 0x01, 0x95, 0x02,       //   2 bits
    0x81, 0x03,                   //   Input (Const)
    0x0B, 0x01, 0x00, 0x01, 0x00, //   Usage (vendor)
    0xA1, 0x00,                   //   Collection (Physical)
    0x0B, 0x30, 0x00, 0x01, 0x00, //     Usage (vendor: X)
    0x0B, 0x31, 0x00, 0x01, 0x00, //     Usage (vendor: Y)
    0x0B, 0x32, 0x00, 0x01, 0x00, //     Usage (vendor: Z)
    0x0B, 0x35, 0x00, 0x01, 0x00, //     Usage (vendor: Rz)
    0x15, 0x00,                   //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65535)
    0x75, 0x10, 0x95, 0x04,       //     4 x 16 bits
    0x81, 0x02,                   //     Input (Data,Var,Abs)
    0xC0,                         //   End Collection
    0x0B, 0x39, 0x00, 0x01, 0x00, //   Usage (vendor: hat)
    0x15, 0x00, 0x25, 0x07,       //   Logical 0..7
    0x35, 0x00,                   //   Physical Minimum (0)
    0x46, 0x3B, 0x01,             //   Physical Maximum (315)
    0x65, 0x14,                   //   Unit (degrees)
    0x75, 0x04, 0x95, 0x01,       //   4 bits
    0x81, 0x02,                   //   Input (Data,Var,Abs)
    0x05, 0x09,                   //   Usage Page (Button)
    0x19, 0x0F, 0x29, 0x12,       //   Buttons 15..18
    0x15, 0x00, 0x25, 0x01,       //   Logical 0..1
    0x75, 0x01, 0x95, 0x04,       //   4 bits
    0x81, 0x02,                   //   Input (Data,Var,Abs)
    0x75, 0x08, 0x95, 0x34,       //   52 bytes padding
    0x81, 0x03,                   //   Input (Const)
    0x06, 0x00, 0xFF,             //   Usage Page (Vendor Defined)
    0x85, 0x21,                   //   Report ID (0x21) - subcommand reply
    0x09, 0x01,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x81, 0x03,                   //   Input (Const)
    0x85, 0x81,                   //   Report ID (0x81) - USB command reply
    0x09, 0x02,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x81, 0x03,                   //   Input (Const)
    0x85, 0x01,                   //   Report ID (0x01) - rumble + subcommand
    0x09, 0x03,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x91, 0x83,                   //   Output (Const,Volatile)
    0x85, 0x10,                   //   Report ID (0x10) - rumble only
    0x09, 0x04,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x91, 0x83,                   //   Output (Const,Volatile)
    0x85, 0x80,                   //   Report ID (0x80) - USB command
    0x09, 0x05,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x91, 0x83,                   //   Output (Const,Volatile)
    0x85, 0x82,                   //   Report ID (0x82)
    0x09, 0x06,                   //   Usage (vendor)
    0x75, 0x08, 0x95, 0x3F,       //   63 bytes
    0x91, 0x83,                   //   Output (Const,Volatile)
    0xC0,                         // End Collection
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return desc_hid_report;
}

//--------------------------------------------------------------------
// Configuration descriptor
//--------------------------------------------------------------------
enum {
    ITF_NUM_HID = 0,
    ITF_NUM_TOTAL,
};

#define EPNUM_HID_OUT 0x01
#define EPNUM_HID_IN  0x81

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
                             sizeof(desc_hid_report),
                             EPNUM_HID_OUT, EPNUM_HID_IN,
                             CFG_TUD_HID_EP_BUFSIZE, 8),
};

// Web config mode: one NCM interface pair
enum {
    ITF_NUM_CDC_NCM = 0,
    ITF_NUM_CDC_NCM_DATA,
    ITF_NUM_TOTAL_CFGMODE,
};

#define CONFIG_TOTAL_LEN_CFGMODE (TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static uint8_t const desc_configuration_cfgmode[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL_CFGMODE, 0, CONFIG_TOTAL_LEN_CFGMODE, 0, 100),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_CDC_NCM, 4, 5, 0x81, 64, 0x02, 0x82,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return usb_config_mode ? desc_configuration_cfgmode : desc_configuration;
}

//--------------------------------------------------------------------
// String descriptors
//--------------------------------------------------------------------
static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
static char mac_str[13];

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: English (US)
    "Nintendo Co., Ltd.",       // 1: Manufacturer
    "Pro Controller",           // 2: Product
    serial_str,                 // 3: Serial
};

static char const *string_desc_arr_cfgmode[] = {
    (const char[]){0x09, 0x04}, // 0: English (US)
    "NXIC",                     // 1: Manufacturer
    "NXIC Web Config",          // 2: Product
    serial_str,                 // 3: Serial
    "NXIC Network",             // 4: NCM interface
    mac_str,                    // 5: iMacAddress
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc_str[32 + 1];
    size_t chr_count;

    char const *const *arr = usb_config_mode ? string_desc_arr_cfgmode : string_desc_arr;
    size_t arr_size = usb_config_mode ? TU_ARRAY_SIZE(string_desc_arr_cfgmode)
                                      : TU_ARRAY_SIZE(string_desc_arr);

    if (index == 0) {
        memcpy(&desc_str[1], arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= arr_size) return NULL;

        if (index == 3 && serial_str[0] == '\0') {
            pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        }
        if (usb_config_mode && index == 5 && mac_str[0] == '\0') {
            for (int i = 0; i < 6; i++) {
                snprintf(&mac_str[i * 2], 3, "%02X", tud_network_mac_address[i]);
            }
        }

        const char *str = arr[index];
        chr_count = strlen(str);
        if (chr_count > 32) chr_count = 32;

        for (size_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = (uint16_t)str[i];
        }
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
