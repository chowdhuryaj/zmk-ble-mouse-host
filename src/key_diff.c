// SPDX-License-Identifier: MIT
#include <zmk_ble_mouse_host/key_diff.h>

static unsigned long read_bits(const unsigned char *buf, unsigned short bitoff,
			       unsigned char nbits)
{
	unsigned long val = 0;
	unsigned char k;
	for (k = 0; k < nbits; k++) {
		unsigned short b = (unsigned short)(bitoff + k);
		val |= ((unsigned long)((buf[b >> 3] >> (b & 7)) & 1)) << k;
	}
	return val;
}

static void add(HeldSet *s, unsigned short usage)
{
	unsigned char i;
	if (!usage)
		return;
	for (i = 0; i < s->n; i++)
		if (s->usage[i] == usage)
			return;
	if (s->n < KD_MAX_HELD) /* ponytail: silently drops the 33rd held usage */
		s->usage[s->n++] = usage;
}

static unsigned char has(const HeldSet *s, unsigned short usage)
{
	unsigned char i;
	for (i = 0; i < s->n; i++)
		if (s->usage[i] == usage)
			return 1;
	return 0;
}

static void diff(HeldSet *held, const HeldSet *cur, unsigned short page, kd_emit_fn emit,
		 void *ctx)
{
	int i;
	unsigned char n;
	for (i = held->n - 1; i >= 0; i--)
		if (!has(cur, held->usage[i]))
			emit(ctx, page, held->usage[i], 0);
	for (i = 0; i < cur->n; i++)
		if (!has(held, cur->usage[i]))
			emit(ctx, page, cur->usage[i], 1);
	held->n = cur->n;
	for (n = 0; n < cur->n; n++)
		held->usage[n] = cur->usage[n];
}

void kd_keyboard(HeldSet *held, const KeyLayout *l, const unsigned char *report,
		 unsigned short len, kd_emit_fn emit, void *ctx)
{
	HeldSet cur;
	unsigned short lenbits = (unsigned short)(len << 3);
	unsigned short i;

	cur.n = 0;
	if (!l->valid || (l->reportId && (len < 1 || report[0] != l->reportId)))
		return;
	if (l->hasMods && (unsigned short)(l->modOffset + 8) <= lenbits) {
		unsigned char mods = (unsigned char)read_bits(report, l->modOffset, 8);
		for (i = 0; i < 8; i++)
			if (mods & (1u << i))
				add(&cur, (unsigned short)(0xE0 + i));
	}
	if (l->isBitmap) {
		for (i = 0; i < l->keysCount; i++) {
			unsigned short b = (unsigned short)(l->keysOffset + i);
			if (b >= lenbits)
				break;
			if ((report[b >> 3] >> (b & 7)) & 1)
				add(&cur, (unsigned short)(l->usageBase + i));
		}
	} else {
		for (i = 0; i < l->keysCount; i++) {
			unsigned short b = (unsigned short)(l->keysOffset + i * 8);
			unsigned short u;
			if (b + 8 > lenbits)
				break;
			u = (unsigned short)read_bits(report, b, 8);
			if (u == 0x01)
				return; /* ErrorRollOver: not a real state, keep what we have */
			if (u > 0x03) /* 0x02 POSTFail, 0x03 undefined */
				add(&cur, u);
		}
	}
	diff(held, &cur, KD_PAGE_KEYBOARD, emit, ctx);
}

void kd_consumer(HeldSet *held, const ConsumerLayout *l, const unsigned char *report,
		 unsigned short len, kd_emit_fn emit, void *ctx)
{
	HeldSet cur;
	unsigned short lenbits = (unsigned short)(len << 3);
	unsigned short i;

	cur.n = 0;
	if (!l->valid || (l->reportId && (len < 1 || report[0] != l->reportId)))
		return;
	for (i = 0; i < l->count; i++) {
		unsigned short b = (unsigned short)(l->offset + i * l->size);
		if (b + l->size > lenbits)
			break;
		add(&cur, (unsigned short)read_bits(report, b, l->size));
	}
	diff(held, &cur, KD_PAGE_CONSUMER, emit, ctx);
}

void kd_release_all(HeldSet *held, unsigned short page, kd_emit_fn emit, void *ctx)
{
	HeldSet none;
	none.n = 0;
	diff(held, &none, page, emit, ctx);
}
