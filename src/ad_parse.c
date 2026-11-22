/*
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <zmk_ble_mouse_host/ad_parse.h>

/* AD type codes (Bluetooth Assigned Numbers). */
#define AD_UUID16_SOME 0x02
#define AD_UUID16_ALL 0x03
#define AD_NAME_SHORT 0x08
#define AD_NAME_COMPLETE 0x09
#define AD_APPEARANCE 0x19

void ad_parse(const uint8_t *ad, uint16_t len, AdInfo *out) {
    bool complete = false;

    memset(out, 0, sizeof(*out));
    /* Each field: [length (type + data)][type][data...]; length 0 ends early. */
    for (uint16_t i = 0; i + 1 < len;) {
        uint8_t flen = ad[i];
        if (flen == 0 || i + 1 + flen > len) {
            break;
        }
        uint8_t type = ad[i + 1];
        const uint8_t *d = &ad[i + 2];
        uint8_t dlen = flen - 1;

        switch (type) {
        case AD_APPEARANCE:
            if (dlen >= 2) {
                out->appearance = (uint16_t)(d[0] | (d[1] << 8));
            }
            break;
        case AD_UUID16_SOME:
        case AD_UUID16_ALL:
            for (uint8_t u = 0; u + 1 < dlen; u += 2) {
                if ((uint16_t)(d[u] | (d[u + 1] << 8)) == AD_UUID_HIDS) {
                    out->hid_uuid = true;
                }
            }
            break;
        case AD_NAME_COMPLETE:
        case AD_NAME_SHORT:
            if (!complete) {
                uint8_t n = dlen > AD_NAME_MAX ? AD_NAME_MAX : dlen;
                memcpy(out->name, d, n);
                out->name[n] = '\0';
                out->name_len = n;
                complete = (type == AD_NAME_COMPLETE);
            }
            break;
        default:
            break;
        }
        i += 1 + flen;
    }
}
