// SPDX-License-Identifier: MIT
#ifndef __KEY_DIFF_H__
#define __KEY_DIFF_H__

#include <zmk_ble_mouse_host/hid_parser.h>

/*
 * Hardware-independent keyboard / consumer report decoding: each report is
 * turned into the set of usages currently held, diffed against the previous
 * set, and one callback is made per press / release.
 *
 * Reports are in the descriptor's own framing (Report ID byte first when the
 * layout has one), same as hid_extract_mouse().
 */

#define KD_PAGE_KEYBOARD 0x07
#define KD_PAGE_CONSUMER 0x0C

/* Max usages tracked per report type; extra held usages are ignored. */
#define KD_MAX_HELD 32

typedef struct
{
	unsigned short usage[KD_MAX_HELD];
	unsigned char n;
} HeldSet;

/* Called once per change. Releases come first (last pressed first), then presses
 * (modifiers before keys). */
typedef void (*kd_emit_fn)(void *ctx, unsigned short page, unsigned short usage,
			   unsigned char pressed);

/* Keyboard modifiers are reported as usages 0xE0-0xE7 on the keyboard page.
 * A report containing ErrorRollOver (0x01) is ignored (state kept). */
void kd_keyboard(HeldSet *held, const KeyLayout *l, const unsigned char *report,
		 unsigned short len, kd_emit_fn emit, void *ctx);

void kd_consumer(HeldSet *held, const ConsumerLayout *l, const unsigned char *report,
		 unsigned short len, kd_emit_fn emit, void *ctx);

/* Release everything still held (disconnect / unpair) and empty the set. */
void kd_release_all(HeldSet *held, unsigned short page, kd_emit_fn emit, void *ctx);

#endif
