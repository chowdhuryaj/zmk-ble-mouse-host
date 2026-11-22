/*
 * SPDX-License-Identifier: MIT
 *
 * Advertising-data (AD) fields the mouse host looks at while scanning. Pure C
 * (no Zephyr), so it is host-tested (test/test_ad_parse.c).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AD_APPEARANCE_HID_KEYBOARD 0x03C1
#define AD_APPEARANCE_HID_MOUSE 0x03C2
#define AD_UUID_HIDS 0x1812

/* Longest name kept: a legacy advertisement's whole AD is 31 bytes, so a name
 * field carries at most 29. */
#define AD_NAME_MAX 29

typedef struct {
    uint16_t appearance; /* 0 = not advertised */
    bool hid_uuid;       /* 0x1812 in a 16-bit service UUID list */
    uint8_t name_len;    /* 0 = no name field */
    char name[AD_NAME_MAX + 1];
} AdInfo;

/* Parse one advertisement or scan response. A complete name wins over a
 * shortened one. Malformed trailing fields are ignored. */
void ad_parse(const uint8_t *ad, uint16_t len, AdInfo *out);

/* Worth listing as a mouse candidate: the Mouse appearance, or the HID
 * service UUID (ZMK devices such as a trackball advertise the Keyboard
 * appearance plus 0x1812, so the UUID alone also lists keyboards). */
static inline bool ad_is_hid(const AdInfo *ai) {
    return ai->appearance == AD_APPEARANCE_HID_MOUSE || ai->hid_uuid;
}
