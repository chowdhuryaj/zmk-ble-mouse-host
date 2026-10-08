// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tadashi Kadowaki
#ifndef __HID_PARSER_H__
#define __HID_PARSER_H__

/*
 * Extracts the bit positions of buttons, X, Y, wheel and horizontal wheel
 * (Consumer AC Pan) from a HID Report Descriptor so a live input report can be
 * decoded generically, regardless of the mouse's specific report layout.
 *
 * Pointers are plain `const unsigned char *` so the same code compiles both on
 * an 8-bit microcontroller and on the host with -DHOST_TEST for unit testing.
 */

/* One extracted field. bitOffset/bitSize are measured from the start of the
 * report, including the 1-byte Report ID when present. For `buttons`, bitSize
 * holds the total number of button bits in the block. */
typedef struct
{
	unsigned char valid;
	unsigned short bitOffset;
	unsigned char bitSize;
	unsigned char reportId;
} HidField;

typedef struct
{
	unsigned char hasReportId;
	unsigned char mouseReportId; /* report id carrying X/Y (0 if none) */
	HidField buttons;
	HidField x;
	HidField y;
	HidField wheel;
	HidField hwheel;
} MouseLayout;

/* Decoded mouse movement for one report.
 * buttons and X/Y are kept at full resolution; wheel/hwheel are clamped to 8-bit. */
typedef struct
{
	unsigned short buttons; /* up to 16 buttons (bit0 = button 1) */
	signed short dx;
	signed short dy;
	signed char wheel;
	signed char hwheel;
} MouseState;

/* Parse a HID Report Descriptor into a MouseLayout. Always clears `out` first.
 * out->x.valid indicates a usable mouse layout was found. */
void hid_parse_mouse(const unsigned char *desc, unsigned short len, MouseLayout *out);

/* Keyboard input report (usage page 0x07): 8 modifier bits (usages 0xE0-0xE7)
 * plus either a key array (6KRO style, 8-bit usages) or a bitmap (NKRO). Offsets
 * are in bits from the start of the report, including the Report ID byte. */
typedef struct
{
	unsigned char valid; /* a key array / bitmap was found */
	unsigned char reportId;
	unsigned char hasMods;
	unsigned short modOffset;
	unsigned char isBitmap;
	unsigned short keysOffset;
	unsigned short keysCount; /* array: 8-bit slots; bitmap: number of bits */
	unsigned short usageBase; /* bitmap: usage of the first bit */
} KeyLayout;

/* Consumer input report (usage page 0x0C): an array of 8- or 16-bit usages. */
typedef struct
{
	unsigned char valid;
	unsigned char reportId;
	unsigned short offset;
	unsigned char size; /* 8 or 16 */
	unsigned char count;
} ConsumerLayout;

/* Parse a HID Report Descriptor for the first keyboard and consumer input
 * reports. Always clears both outputs first. */
void hid_parse_keys(const unsigned char *desc, unsigned short len, KeyLayout *kb,
		    ConsumerLayout *cc);

/* Decode one input report using a parsed layout. The report buffer must be in
 * the descriptor's own framing, i.e. prefixed with the 1-byte Report ID when
 * layout->mouseReportId is non-zero.
 * Returns 1 and fills `st` if the report matches the mouse layout, else 0. */
unsigned char hid_extract_mouse(const MouseLayout *layout,
				const unsigned char *report, unsigned short len,
				MouseState *st);

#endif
