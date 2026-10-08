// SPDX-License-Identifier: MIT
/* Host-side tests for key_diff.c using ZMK's real report descriptors. */
#include <stdio.h>
#include <string.h>
#include <zmk_ble_mouse_host/key_diff.h>
#include "zmk_desc.h"

static int failures = 0;

#define CHECK(cond, msg)                                                      \
	do {                                                                  \
		if (!(cond)) {                                                \
			printf("  FAIL: %s\n", msg);                          \
			failures++;                                           \
		}                                                             \
	} while (0)

/* Records events as "+7:E0 " / "-C:CD " (press/release, page, usage in hex). */
static char log_buf[256];

static void rec(void *ctx, unsigned short page, unsigned short usage, unsigned char pressed)
{
	(void)ctx;
	sprintf(log_buf + strlen(log_buf), "%c%X:%X ", pressed ? '+' : '-', page, usage);
}

#define EXPECT(str, msg)                                                       \
	do {                                                                   \
		if (strcmp(log_buf, str) != 0)                                 \
			printf("  FAIL: %s: got \"%s\" want \"%s\"\n", msg, log_buf, str), failures++; \
		log_buf[0] = 0;                                                \
	} while (0)

static void test_6kro(void)
{
	KeyLayout kb;
	ConsumerLayout cc;
	HeldSet h = {{0}, 0};
	/* id, mods, reserved, 6 keys */
	unsigned char shift[]   = {1, 0x02, 0, 0, 0, 0, 0, 0, 0};
	unsigned char shift_a[] = {1, 0x02, 0, 0x04, 0, 0, 0, 0, 0};
	unsigned char a_only[]  = {1, 0x00, 0, 0x04, 0, 0, 0, 0, 0};
	unsigned char a_b[]     = {1, 0x00, 0, 0x04, 0x05, 0, 0, 0, 0};
	unsigned char b_only[]  = {1, 0x00, 0, 0x05, 0, 0, 0, 0, 0};
	unsigned char rollover[] = {1, 0x00, 0, 1, 1, 1, 1, 1, 1};
	unsigned char none[]    = {1, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char other_id[] = {2, 0x02, 0, 0x04, 0, 0, 0, 0, 0};

	printf("test_6kro\n");
	log_buf[0] = 0;
	hid_parse_keys(zmk_desc_default, sizeof(zmk_desc_default), &kb, &cc);

	kd_keyboard(&h, &kb, shift, sizeof(shift), rec, 0);
	EXPECT("+7:E1 ", "left shift press (modifier usage 0xE1)");
	kd_keyboard(&h, &kb, shift, sizeof(shift), rec, 0);
	EXPECT("", "unchanged report -> no events");
	kd_keyboard(&h, &kb, shift_a, sizeof(shift_a), rec, 0);
	EXPECT("+7:4 ", "A pressed while shift held");
	kd_keyboard(&h, &kb, a_only, sizeof(a_only), rec, 0);
	EXPECT("-7:E1 ", "shift released, A stays");
	kd_keyboard(&h, &kb, a_b, sizeof(a_b), rec, 0);
	EXPECT("+7:5 ", "B pressed (roll-over)");
	kd_keyboard(&h, &kb, b_only, sizeof(b_only), rec, 0);
	EXPECT("-7:4 ", "A released, B stays");
	kd_keyboard(&h, &kb, rollover, sizeof(rollover), rec, 0);
	EXPECT("", "ErrorRollOver report ignored");
	kd_keyboard(&h, &kb, other_id, sizeof(other_id), rec, 0);
	EXPECT("", "wrong report id ignored");
	kd_keyboard(&h, &kb, none, sizeof(none), rec, 0);
	EXPECT("-7:5 ", "B released");

	/* release-all on disconnect: keys before modifiers, then set is empty */
	kd_keyboard(&h, &kb, shift_a, sizeof(shift_a), rec, 0);
	log_buf[0] = 0;
	kd_release_all(&h, KD_PAGE_KEYBOARD, rec, 0);
	EXPECT("-7:4 -7:E1 ", "release-all releases key then modifier");
	kd_release_all(&h, KD_PAGE_KEYBOARD, rec, 0);
	EXPECT("", "release-all on empty set");
	kd_keyboard(&h, &kb, shift_a, sizeof(shift_a), rec, 0);
	EXPECT("+7:E1 +7:4 ", "after release-all, a held key presses again (modifier first)");
}

static void test_consumer_6kro_build(void)
{
	KeyLayout kb;
	ConsumerLayout cc;
	HeldSet h = {{0}, 0};
	unsigned char vol_up[] = {2, 0xE9, 0, 0, 0, 0, 0};
	unsigned char both[]   = {2, 0xE9, 0xEA, 0, 0, 0, 0};
	unsigned char none[]   = {2, 0, 0, 0, 0, 0, 0};

	printf("test_consumer_basic\n");
	log_buf[0] = 0;
	hid_parse_keys(zmk_desc_default, sizeof(zmk_desc_default), &kb, &cc);
	kd_consumer(&h, &cc, vol_up, sizeof(vol_up), rec, 0);
	EXPECT("+C:E9 ", "volume up press");
	kd_consumer(&h, &cc, both, sizeof(both), rec, 0);
	EXPECT("+C:EA ", "volume down added");
	kd_consumer(&h, &cc, none, sizeof(none), rec, 0);
	EXPECT("-C:EA -C:E9 ", "both released");
}

static void test_nkro_full(void)
{
	KeyLayout kb;
	ConsumerLayout cc;
	HeldSet kh = {{0}, 0}, ch = {{0}, 0};
	unsigned char k[1 + 2 + 13] = {1};
	unsigned char c[1 + 12] = {2};

	printf("test_nkro_full\n");
	log_buf[0] = 0;
	hid_parse_keys(zmk_desc_nkro_full, sizeof(zmk_desc_nkro_full), &kb, &cc);

	k[1] = 0x01;      /* left ctrl */
	k[3 + 0] = 0x10;  /* usage 4 (A): bit 4 of byte 0 of the bitmap */
	k[3 + 12] = 0x80; /* bit 103 = usage 0x67 (keypad =) */
	kd_keyboard(&kh, &kb, k, sizeof(k), rec, 0);
	EXPECT("+7:E0 +7:4 +7:67 ", "NKRO bitmap: ctrl, A, keypad =");
	k[1] = 0;
	k[3 + 0] = 0;
	kd_keyboard(&kh, &kb, k, sizeof(k), rec, 0);
	EXPECT("-7:4 -7:E0 ", "A and ctrl released, keypad = held");

	c[1] = 0xCD; /* play/pause 0x00CD */
	c[3] = 0x21; c[4] = 0x02; /* AC Search 0x0221, above 8 bits */
	kd_consumer(&ch, &cc, c, sizeof(c), rec, 0);
	EXPECT("+C:CD +C:221 ", "16-bit consumer usages");
	c[1] = 0;
	kd_consumer(&ch, &cc, c, sizeof(c), rec, 0);
	EXPECT("-C:CD ", "play/pause released");
	kd_release_all(&ch, KD_PAGE_CONSUMER, rec, 0);
	EXPECT("-C:221 ", "release-all consumer");
}

int main(void)
{
	test_6kro();
	test_consumer_6kro_build();
	test_nkro_full();
	if (failures == 0) {
		printf("\nALL KEY DIFF TESTS PASSED\n");
		return 0;
	}
	printf("\n%d TEST(S) FAILED\n", failures);
	return 1;
}
