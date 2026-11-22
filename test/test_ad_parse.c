/* Host test for ad_parse(). Run: make -C test */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <zmk_ble_mouse_host/ad_parse.h>

int main(void) {
    AdInfo ai;

    /* Flags, mouse appearance, HID UUID among others, shortened then complete name. */
    const uint8_t mouse[] = {0x02, 0x01, 0x06, 0x03, 0x19, 0xC2, 0x03, 0x05, 0x03, 0x0F,
                             0x18, 0x12, 0x18, 0x03, 0x08, 'E',  'L',  0x07, 0x09, 'E',
                             'L',  'E',  'C',  'O',  'M'};
    ad_parse(mouse, sizeof(mouse), &ai);
    assert(ai.appearance == AD_APPEARANCE_HID_MOUSE);
    assert(ai.hid_uuid);
    assert(ai.name_len == 6 && strcmp(ai.name, "ELECOM") == 0);
    assert(ad_is_hid(&ai));

    /* ZMK device: keyboard appearance + HID UUID still lists. */
    const uint8_t zmk[] = {0x03, 0x19, 0xC1, 0x03, 0x03, 0x03, 0x12, 0x18};
    ad_parse(zmk, sizeof(zmk), &ai);
    assert(ai.appearance == AD_APPEARANCE_HID_KEYBOARD && ai.hid_uuid && ai.name_len == 0);
    assert(ad_is_hid(&ai));

    /* Not HID; truncated trailing field ignored; complete name kept over a later short one. */
    const uint8_t other[] = {0x04, 0x09, 'A', 'B', 'C', 0x02, 0x08, 'Z', 0x05, 0x19, 0x00};
    ad_parse(other, sizeof(other), &ai);
    assert(!ad_is_hid(&ai));
    assert(strcmp(ai.name, "ABC") == 0);

    /* Over-long name is capped. */
    uint8_t longname[2 + 40];
    longname[0] = 41;
    longname[1] = 0x09;
    memset(longname + 2, 'x', 40);
    ad_parse(longname, 42, &ai);
    assert(ai.name_len == AD_NAME_MAX);

    printf("ad_parse: ok\n");
    return 0;
}
