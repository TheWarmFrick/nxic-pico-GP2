// USB host side: collect keyboard / mouse reports (runs on core1), expose
// merged state to core0.
//
// Keyboards are switched to boot protocol and strictly validated.
//
// Mice are kept in REPORT protocol and their report descriptor is parsed to
// find the exact bit offsets / sizes of X, Y and the buttons. Many gaming
// mice send 12/16-bit deltas (optionally behind a report ID); assuming the
// 3-byte boot layout for those reads the X high byte as "Y", which shows up
// as phantom vertical input. Descriptor parsing handles them all.
//
// Mouse movement is accumulated so no motion is ever lost regardless of the
// mouse polling rate.

#include <string.h>

#include "pico/sync.h"
#include "tusb.h"

#include "hid_input.h"

//--------------------------------------------------------------------
// Mouse report layout extracted from the HID report descriptor
//--------------------------------------------------------------------
typedef struct {
    bool valid;
    bool is_mouse;       // descriptor declares Usage (Mouse)
    bool has_report_id;
    uint8_t xy_report_id;
    uint8_t btn_report_id;
    uint16_t x_off, y_off, btn_off; // bit offsets within the report body
    uint8_t x_size, y_size;         // bits (8 / 12 / 16)
    uint8_t btn_count;
} mouse_layout_t;

typedef struct {
    bool used;
    uint8_t dev_addr;
    uint8_t instance;
    uint8_t itf_protocol; // HID_ITF_PROTOCOL_KEYBOARD / MOUSE
    bool use_layout;      // mouse: parse with `layout` instead of boot format
    mouse_layout_t layout;
    hid_keyboard_report_t kb;
    uint8_t mouse_buttons;
} slot_t;

static slot_t slots[CFG_TUH_HID];
static critical_section_t lock;
static int32_t acc_dx, acc_dy;

void hid_input_init(void) {
    critical_section_init(&lock);
}

static slot_t *find_slot(uint8_t dev_addr, uint8_t instance) {
    for (int i = 0; i < CFG_TUH_HID; i++) {
        if (slots[i].used && slots[i].dev_addr == dev_addr && slots[i].instance == instance) {
            return &slots[i];
        }
    }
    return NULL;
}

//--------------------------------------------------------------------
// Minimal HID report descriptor parser for mice
//--------------------------------------------------------------------
#define MAX_REPORT_IDS 8

static uint16_t *cursor_for(uint8_t id, uint8_t *ids, uint16_t *curs, int *n) {
    for (int i = 0; i < *n; i++) {
        if (ids[i] == id) return &curs[i];
    }
    if (*n >= MAX_REPORT_IDS) return NULL;
    ids[*n] = id;
    curs[*n] = 0;
    return &curs[(*n)++];
}

static bool parse_mouse_desc(uint8_t const *d, uint16_t len, mouse_layout_t *out) {
    uint8_t ids[MAX_REPORT_IDS];
    uint16_t curs[MAX_REPORT_IDS];
    int ncurs = 0;

    uint8_t cur_id = 0;
    bool any_id = false;
    uint16_t usage_page = 0;
    uint16_t usages[8];
    int nusages = 0;
    uint32_t rsize = 0, rcount = 0;
    bool seen_collection = false;
    uint8_t xy_id = 0, btn_id = 0;

    memset(out, 0, sizeof(*out));
    if (!d || len == 0) return false;

    uint16_t pos = 0;
    while (pos < len) {
        uint8_t prefix = d[pos++];
        if (prefix == 0xFE) { // long item: skip
            if (pos >= len) break;
            uint8_t dlen = d[pos];
            pos = (uint16_t)(pos + 2 + dlen);
            continue;
        }
        uint8_t isize = prefix & 3;
        if (isize == 3) isize = 4;
        if (pos + isize > len) break;
        uint32_t data = 0;
        for (uint8_t i = 0; i < isize; i++) data |= (uint32_t)d[pos + i] << (8 * i);
        pos = (uint16_t)(pos + isize);

        uint8_t type = (prefix >> 2) & 3;
        uint8_t tag = prefix >> 4;

        if (type == 1) { // global
            switch (tag) {
                case 0: usage_page = (uint16_t)data; break;
                case 7: rsize = data; break;
                case 8: cur_id = (uint8_t)data; any_id = true; break;
                case 9: rcount = data; break;
                default: break;
            }
        } else if (type == 2) { // local
            if (tag == 0 && nusages < 8) usages[nusages++] = (uint16_t)data;
        } else if (type == 0) { // main
            if (tag == 0xA) { // collection
                if (!seen_collection && usage_page == 0x01 && nusages > 0 &&
                    (usages[0] == 0x02 || usages[0] == 0x01)) {
                    out->is_mouse = (usages[0] == 0x02);
                }
                seen_collection = true;
            } else if (tag == 8) { // input
                uint16_t *cur = cursor_for(cur_id, ids, curs, &ncurs);
                if (!cur) return false;
                bool constant = (data & 1) != 0;
                if (!constant) {
                    if (usage_page == 0x01) { // generic desktop: X / Y
                        for (uint32_t i = 0; i < rcount; i++) {
                            uint16_t u = (i < (uint32_t)nusages)
                                             ? usages[i]
                                             : (nusages ? usages[nusages - 1] : 0);
                            uint16_t off = (uint16_t)(*cur + i * rsize);
                            if (u == 0x30 && out->x_size == 0) {
                                out->x_off = off;
                                out->x_size = (uint8_t)rsize;
                                xy_id = cur_id;
                            } else if (u == 0x31 && out->y_size == 0) {
                                out->y_off = off;
                                out->y_size = (uint8_t)rsize;
                            }
                        }
                    } else if (usage_page == 0x09 && out->btn_count == 0 && rsize == 1) {
                        out->btn_off = *cur;
                        out->btn_count = (uint8_t)(rcount < 8 ? rcount : 8);
                        btn_id = cur_id;
                    }
                }
                *cur = (uint16_t)(*cur + rsize * rcount);
            }
            nusages = 0; // main items consume local state
        }
    }

    out->has_report_id = any_id;
    out->xy_report_id = xy_id;
    out->btn_report_id = btn_id;
    out->valid = out->x_size >= 4 && out->x_size <= 16 &&
                 out->y_size >= 4 && out->y_size <= 16;
    return out->valid || out->is_mouse;
}

//--------------------------------------------------------------------
// Bit-field extraction (HID fields are LSB-first)
//--------------------------------------------------------------------
static uint32_t extract_bits(uint8_t const *p, uint16_t off, uint8_t bits) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < bits; i++) {
        uint16_t b = (uint16_t)(off + i);
        if (p[b >> 3] & (1u << (b & 7))) v |= 1u << i;
    }
    return v;
}

static int32_t extract_signed(uint8_t const *p, uint16_t off, uint8_t bits) {
    uint32_t v = extract_bits(p, off, bits);
    if (bits < 32 && (v & (1u << (bits - 1)))) v |= ~((1u << bits) - 1u);
    return (int32_t)v;
}

static void handle_layout_mouse(slot_t *s, uint8_t const *report, uint16_t len) {
    mouse_layout_t const *L = &s->layout;
    uint8_t const *p = report;
    uint16_t plen = len;
    uint8_t rid = 0;

    if (L->has_report_id) {
        if (plen < 2) return;
        rid = p[0];
        p++;
        plen--;
    }
    uint32_t plen_bits = (uint32_t)plen * 8;

    critical_section_enter_blocking(&lock);
    if (!L->has_report_id || rid == L->xy_report_id) {
        uint32_t need_x = (uint32_t)L->x_off + L->x_size;
        uint32_t need_y = (uint32_t)L->y_off + L->y_size;
        if (need_x <= plen_bits && need_y <= plen_bits) {
            acc_dx += extract_signed(p, L->x_off, L->x_size);
            acc_dy += extract_signed(p, L->y_off, L->y_size);
        }
    }
    if (L->btn_count && (!L->has_report_id || rid == L->btn_report_id)) {
        if ((uint32_t)L->btn_off + L->btn_count <= plen_bits) {
            s->mouse_buttons = (uint8_t)extract_bits(p, L->btn_off, L->btn_count);
        }
    }
    critical_section_exit(&lock);
}

//--------------------------------------------------------------------
// TinyUSB host callbacks (core1)
//--------------------------------------------------------------------
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len) {
    uint8_t itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    mouse_layout_t lay;
    bool parsed = parse_mouse_desc(desc_report, desc_len, &lay);

    bool is_kbd = (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD);
    bool is_mouse = (itf_protocol == HID_ITF_PROTOCOL_MOUSE) ||
                    (parsed && lay.is_mouse);
    if (!is_kbd && !is_mouse) return;

    critical_section_enter_blocking(&lock);
    slot_t *s = find_slot(dev_addr, instance);
    if (!s) {
        for (int i = 0; i < CFG_TUH_HID; i++) {
            if (!slots[i].used) {
                s = &slots[i];
                break;
            }
        }
    }
    if (s) {
        memset(s, 0, sizeof(*s));
        s->used = true;
        s->dev_addr = dev_addr;
        s->instance = instance;
        s->itf_protocol = is_kbd ? HID_ITF_PROTOCOL_KEYBOARD : HID_ITF_PROTOCOL_MOUSE;
        if (is_mouse && lay.valid) {
            s->use_layout = true;
            s->layout = lay;
        }
    }
    critical_section_exit(&lock);
    if (!s) return;

    if (is_kbd || !s->use_layout) {
        // Boot protocol for keyboards, and for mice whose descriptor we
        // could not parse
        if (tuh_hid_get_protocol(dev_addr, instance) != HID_PROTOCOL_BOOT) {
            tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_BOOT);
        }
    }
    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    critical_section_enter_blocking(&lock);
    slot_t *s = find_slot(dev_addr, instance);
    if (s) memset(s, 0, sizeof(*s));
    critical_section_exit(&lock);
}

// A valid boot keyboard report is exactly 8 bytes and every keycode slot is
// either empty or a real usage. Report-protocol data (report-ID prefixed,
// NKRO bitmaps, vendor reports) fails these checks -- without them, stray
// bytes were being read as phantom keypresses.
static bool valid_boot_kb_report(uint8_t const *report, uint16_t len) {
    if (len != 8) return false;
    for (int k = 2; k < 8; k++) {
        uint8_t kc = report[k];
        if (kc != 0 && (kc < HID_KEY_A || kc > HID_KEY_GUI_RIGHT)) return false;
    }
    return true;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len) {
    slot_t *s = find_slot(dev_addr, instance);
    if (s && len > 0) {
        if (s->itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
            if (tuh_hid_get_protocol(dev_addr, instance) == HID_PROTOCOL_BOOT &&
                valid_boot_kb_report(report, len)) {
                critical_section_enter_blocking(&lock);
                memcpy(&s->kb, report, sizeof(hid_keyboard_report_t));
                critical_section_exit(&lock);
            }
        } else if (s->use_layout) {
            handle_layout_mouse(s, report, len);
        } else if (tuh_hid_get_protocol(dev_addr, instance) == HID_PROTOCOL_BOOT &&
                   len >= 3 && len <= 8) {
            hid_mouse_report_t const *m = (hid_mouse_report_t const *)report;
            critical_section_enter_blocking(&lock);
            s->mouse_buttons = m->buttons;
            acc_dx += m->x;
            acc_dy += m->y;
            critical_section_exit(&lock);
        }
    }
    tuh_hid_receive_report(dev_addr, instance);
}

//--------------------------------------------------------------------
// Merged-state getters (core0)
//--------------------------------------------------------------------
bool hid_key_down(uint8_t keycode) {
    bool hit = false;
    critical_section_enter_blocking(&lock);
    for (int i = 0; i < CFG_TUH_HID && !hit; i++) {
        if (!slots[i].used || slots[i].itf_protocol != HID_ITF_PROTOCOL_KEYBOARD) continue;
        for (int k = 0; k < 6; k++) {
            if (slots[i].kb.keycode[k] == keycode) {
                hit = true;
                break;
            }
        }
    }
    critical_section_exit(&lock);
    return hit;
}

uint8_t hid_kbd_modifiers(void) {
    uint8_t mods = 0;
    critical_section_enter_blocking(&lock);
    for (int i = 0; i < CFG_TUH_HID; i++) {
        if (slots[i].used && slots[i].itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
            mods |= slots[i].kb.modifier;
        }
    }
    critical_section_exit(&lock);
    return mods;
}

uint8_t hid_mouse_buttons(void) {
    uint8_t buttons = 0;
    critical_section_enter_blocking(&lock);
    for (int i = 0; i < CFG_TUH_HID; i++) {
        if (slots[i].used && slots[i].itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
            buttons |= slots[i].mouse_buttons;
        }
    }
    critical_section_exit(&lock);
    return buttons;
}

void hid_mouse_take_deltas(int32_t *dx, int32_t *dy) {
    critical_section_enter_blocking(&lock);
    *dx = acc_dx;
    *dy = acc_dy;
    acc_dx = 0;
    acc_dy = 0;
    critical_section_exit(&lock);
}
