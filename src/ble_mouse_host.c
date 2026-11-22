/*
 * Copyright (c) 2026 Tadashi Kadowaki
 * SPDX-License-Identifier: MIT
 *
 * BLE mouse host (central / HID-over-GATT host).
 *
 * Connects to an external BLE mouse, subscribes to its HID Input Report(s) and
 * injects the pointer events into ZMK's pointing pipeline via the Zephyr input
 * subsystem. Because the events ride ZMK's normal mouse HID path, they are sent
 * to whichever output (BLE profile / USB) the keyboard is currently using --
 * switching the keyboard's output moves the mouse with it.
 *
 * Reconnecting to a bonded mouse connects directly to its known address
 * (bt_conn_le_create, "Initiating State"), NOT by scanning: the mouse reconnects
 * via directed advertising, which this scan config never surfaces to the scan
 * callback (root-caused with an RF sniffer to a TargetA address-type mismatch),
 * so don't switch this back to a scan-then-match. Initial/fresh pairing, where
 * the mouse's address isn't known yet, does scan, matched by HID appearance
 * (see scan_cb()), or by advertised name when pair-name-prefix is set (needed for
 * ZMK devices, which advertise the Keyboard appearance).
 *
 * With CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS the device's keyboard and consumer
 * input reports are also decoded and raised as zmk_keycode_state_changed events
 * (from the system workqueue), so a ZMK trackball keeps its key/media bindings.
 *
 * AJ fork additions: a list scan that only collects nearby HID devices (never
 * connects) and pairing to one chosen address, for a configurator; plus a
 * status snapshot (state, name, battery, button count) and a state-changed
 * event, for a status widget. See include/zmk/ble_mouse_host.h.
 */

#define DT_DRV_COMPAT zmk_ble_mouse_host

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>
#include <zmk/ble_mouse_host.h>
#include <zmk_ble_mouse_host/hid_parser.h>
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
#include <zmk/events/keycode_state_changed.h>
#include <zmk_ble_mouse_host/key_diff.h>
#endif

LOG_MODULE_REGISTER(ble_mouse_host, CONFIG_ZMK_BLE_MOUSE_HOST_LOG_LEVEL);

ZMK_EVENT_IMPL(zmk_ble_mouse_host_state_changed);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* The virtual input device that emits the forwarded pointer events. */
DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
static const struct device *const mh_dev = DEVICE_DT_INST_GET(0);

#define MAX_REPORTS CONFIG_ZMK_BLE_MOUSE_HOST_MAX_REPORTS
#define MAX_SUBS    (MAX_REPORTS + 1) /* + boot mouse */

/* Mouse link connection parameters: low latency for responsive pointing.
 * interval 11.25-15 ms (units of 1.25 ms), latency 0, supervision 4 s. */
#define MH_CONN_PARAM BT_LE_CONN_PARAM(9, 12, 0, 400)

/* ---- state ------------------------------------------------------------- */

enum mh_mode { MH_IDLE, MH_SCAN };

static struct bt_conn *mouse_conn;
static enum mh_mode mode;
static bool connecting;
static bool disc_started;
/* True while the current connection attempt is a fresh pairing (a scan match to
 * an unknown address), false when reconnecting to the known bonded address. Lets
 * disc_service_cb() tell "a device we scanned that turned out not to be a mouse"
 * apart from "our real bonded mouse", so it only drops the bond in the former. */
static bool pairing_attempt;

static bt_addr_le_t mouse_addr;
static bool have_bond;

/* HID-over-GATT handles discovered on the mouse. */
static uint16_t svc_end;
static uint16_t proto_handle;       /* Protocol Mode (write) */
static uint16_t map_handle;         /* Report Map (read) */
static uint16_t boot_mouse_handle;  /* Boot Mouse Input Report (notify) */
static uint16_t report_handles[MAX_REPORTS];
static uint16_t report_end[MAX_REPORTS]; /* last attribute handle of each report characteristic */
static uint8_t report_ids[MAX_REPORTS];  /* from the Report Reference descriptor (0 = unknown) */
static int n_reports;
static int pending_end = -1; /* report whose report_end the next characteristic sets */
static bool use_boot;

static struct bt_gatt_discover_params disc;
static struct bt_gatt_subscribe_params sub[MAX_SUBS];
static struct bt_gatt_discover_params sub_disc[MAX_SUBS];
static int n_subs;
static struct bt_gatt_read_params map_read;

static uint8_t prev_buttons;

/* Name of the bonded mouse (or pairing target), taken from its advertising at
 * pairing time and saved with the bond, so the status has it after a reboot. */
static char mouse_name[ZMK_BLE_MOUSE_HOST_NAME_MAX + 1];
static uint8_t battery = 0xFF; /* BAS level of the connected mouse, 0xFF unknown */

/* zmk_ble_mouse_host_pair_addr() target: connected to directly (no scan, the
 * address is known), MH_TARGET_TRIES times, then given up. */
#define MH_TARGET_TRIES 5
static bt_addr_le_t target_addr;
static bool have_target;
static uint8_t target_tries;

/* The bond held before a pair-by-address attempt. Its keys and saved
 * settings stay until the new mouse bonds (then it is unpaired), so a
 * failed attempt, or a reboot mid-attempt, falls back to it. */
static bt_addr_le_t prev_addr;
static char prev_name[ZMK_BLE_MOUSE_HOST_NAME_MAX + 1];
static bool have_prev;
static void target_failed(void);

/* Set by zmk_ble_mouse_host_unpair() to stop reconnect/scan attempts until
 * zmk_ble_mouse_host_pair() is called. */
static bool mh_paused;

/* Set by the mouse_pair trigger, cleared when that pairing session ends. Gates
 * scanning for a NEW mouse: there is deliberately no automatic scan -- while
 * unbonded, mh_begin() idles until mouse_pair is pressed, so a boot-time scan
 * can't interfere with the split's own peripheral pairing. Reconnecting to an
 * already-bonded mouse is automatic (have_bond branch) and unaffected. */
static bool pairing_requested;

/* List scan (zmk_ble_mouse_host_scan_start): collects, never connects. */
static bool listing;
static struct zmk_ble_mouse_host_scan_result results[CONFIG_ZMK_BLE_MOUSE_HOST_LIST_SIZE];
static int n_results;

static void mh_begin(void);
static void start_discovery(struct bt_conn *conn);
static void clear_bond(void);

/* Status changed: raise the event from the system workqueue, never from a BT
 * callback (listeners may touch the display or the keymap). */
static void notify_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    struct zmk_ble_mouse_host_status st;
    zmk_ble_mouse_host_get_status(&st);
    raise_zmk_ble_mouse_host_state_changed(
        (struct zmk_ble_mouse_host_state_changed){.state = st.state});
}
static K_WORK_DEFINE(notify_work, notify_work_cb);

static void mh_notify(void) { k_work_submit(&notify_work); }

/* Never call bt_conn_le_create() (which mh_begin() may do) from within a BT
 * connection callback -- doing so re-entrantly deadlocks the device. Callbacks
 * defer via this work item; non-callback contexts (mh_thread, keymap triggers)
 * call mh_begin() directly. */
static void mh_retry_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    mh_begin();
}
static K_WORK_DELAYABLE_DEFINE(mh_retry_work, mh_retry_work_cb);

/* Delay before retrying after a failed/timed-out create or -EALREADY. This device
 * is also ZMK's split central, which shares Zephyr's single connection initiator
 * and scanner with us; an immediate retry would starve it of initiator windows. */
#define MH_RETRY_DELAY K_MSEC(CONFIG_ZMK_BLE_MOUSE_HOST_RECONNECT_DELAY_MS)

/* True only while the scan running is ours (bt_le_scan_start returned 0). A scan
 * started by ZMK's split central must never be stopped by us: ZMK's is_scanning
 * would stay true and it would never rescan for a dropped half. */
static bool scan_owned;

static void stop_own_scan(void) {
    if (scan_owned) {
        bt_le_scan_stop();
        scan_owned = false;
    }
    mode = MH_IDLE;
    listing = false; /* the list scan rides the same scan */
    mh_notify();
}

/* ---- settings (persist the bonded mouse address) ----------------------- */

static int mh_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (settings_name_steq(name, "addr", NULL)) {
        ssize_t rc = read_cb(cb_arg, &mouse_addr, sizeof(mouse_addr));
        if (rc == sizeof(mouse_addr)) {
            have_bond = true;
        }
        return 0;
    }
    if (settings_name_steq(name, "name", NULL)) {
        ssize_t rc = read_cb(cb_arg, mouse_name, sizeof(mouse_name) - 1);
        mouse_name[rc > 0 ? rc : 0] = '\0';
        return 0;
    }
    return -ENOENT;
}
SETTINGS_STATIC_HANDLER_DEFINE(mouse_host, "mouse_host", NULL, mh_settings_set, NULL, NULL);

/* ---- input injection (runs on a work queue, never in the BT RX thread) -- */

enum rep_kind { REP_MOUSE, REP_KEYBOARD, REP_CONSUMER };

struct raw_rep {
    uint16_t handle;
    uint8_t is_boot;
    uint8_t kind; /* enum rep_kind */
    uint8_t len;
    uint8_t buf[32];
};

K_MSGQ_DEFINE(rep_q, sizeof(struct raw_rep), 16, 4);

/* Scroll-wheel inversion flag, defined in wheel_invert.c and set from the active
 * output endpoint (or the wheel-invert behavior). */
extern bool mh_wheel_inverted;

static void emit_motion(int dx, int dy, int wheel, int pan, uint8_t buttons) {
    if (mh_wheel_inverted) {
        wheel = -wheel; /* vertical wheel only, matching per-endpoint config */
    }
    uint8_t changed = buttons ^ prev_buttons;
    int n = (dx ? 1 : 0) + (dy ? 1 : 0) + (wheel ? 1 : 0) + (pan ? 1 : 0);
    for (int b = 0; b < 8; b++) {
        if (changed & BIT(b)) {
            n++;
        }
    }
    if (n == 0) {
        return;
    }

    int e = 0;
#define SYNC (++e == n)
    /* Buttons first, highest first: buttons 6-8 (INPUT_BTN_5..7) are not in
     * ZMK's mouse HID and are usually consumed by a zmk,input-processor-behaviors
     * on the listener (which stops the event, sync flag included). Emitting
     * them first puts the sync on a forwarded event whenever there is one. */
    for (int b = 7; b >= 0; b--) {
        if (changed & BIT(b)) {
            /* Short wait, not K_NO_WAIT: a button event dropped on a full
             * input queue would leave its key position held (stuck layer or
             * key). Runs on the system workqueue; motion still never waits. */
            input_report_key(mh_dev, INPUT_BTN_0 + b, (buttons >> b) & 1, SYNC, K_MSEC(10));
        }
    }
    if (dx) {
        input_report_rel(mh_dev, INPUT_REL_X, dx, SYNC, K_NO_WAIT);
    }
    if (dy) {
        input_report_rel(mh_dev, INPUT_REL_Y, dy, SYNC, K_NO_WAIT);
    }
    if (wheel) {
        input_report_rel(mh_dev, INPUT_REL_WHEEL, wheel, SYNC, K_NO_WAIT);
    }
    if (pan) {
        input_report_rel(mh_dev, INPUT_REL_HWHEEL, pan, SYNC, K_NO_WAIT);
    }
#undef SYNC
    prev_buttons = buttons;
}

static void release_all_buttons(void) {
    emit_motion(0, 0, 0, 0, 0);
}

#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
/* Keyboard / consumer forwarding. Layouts are parsed from the report map; the
 * held sets are only touched from the system workqueue (rep_work_cb and
 * release_keys_work), which is also where the events are raised. */
static KeyLayout kb_layout;
static ConsumerLayout cc_layout;
static HeldSet kb_held, cc_held;

static void emit_key(void *ctx, unsigned short page, unsigned short usage, unsigned char pressed) {
    ARG_UNUSED(ctx);
    raise_zmk_keycode_state_changed((struct zmk_keycode_state_changed){
        .usage_page = page,
        .keycode = usage,
        .state = pressed,
        .timestamp = k_uptime_get(),
    });
}

static void forward_keys(const struct raw_rep *r) {
    /* As for the mouse: notifications lack the report-id byte, the layouts' offsets
     * include it. */
    uint8_t buf[sizeof(r->buf) + 1];
    uint8_t id = (r->kind == REP_KEYBOARD) ? kb_layout.reportId : cc_layout.reportId;
    uint16_t n = 0;
    if (id) {
        buf[n++] = id;
    }
    memcpy(buf + n, r->buf, r->len);
    n += r->len;
    if (r->kind == REP_KEYBOARD) {
        kd_keyboard(&kb_held, &kb_layout, buf, n, emit_key, NULL);
    } else {
        kd_consumer(&cc_held, &cc_layout, buf, n, emit_key, NULL);
    }
}

static void release_keys_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    kd_release_all(&kb_held, KD_PAGE_KEYBOARD, emit_key, NULL);
    kd_release_all(&cc_held, KD_PAGE_CONSUMER, emit_key, NULL);
}
static K_WORK_DEFINE(release_keys_work, release_keys_work_cb);
#endif

/* ---- HID report map parsing (generic; see zmk_ble_mouse_host/hid_parser.h) */

static MouseLayout layout;
static bool layout_valid;
static uint8_t report_map[512];
static uint16_t report_map_len;

/* Length in bytes of the mouse's input report, in the report descriptor's own
 * framing (i.e. INCLUDING the leading report-id byte when mouseReportId != 0).
 * A mouse can expose more than one input-report characteristic (e.g. an extra
 * vendor report); notifications are gated on this length so only the report
 * that actually matches the parsed mouse layout is decoded. */
static uint16_t mouse_report_len;

static uint16_t layout_report_len(const MouseLayout *l) {
    uint16_t maxbit = 0;
#define MH_END(f)                                                                                  \
    do {                                                                                           \
        if ((f).valid) {                                                                           \
            uint16_t e = (f).bitOffset + (f).bitSize;                                               \
            if (e > maxbit) {                                                                      \
                maxbit = e;                                                                        \
            }                                                                                      \
        }                                                                                          \
    } while (0)
    MH_END(l->buttons);
    MH_END(l->x);
    MH_END(l->y);
    MH_END(l->wheel);
    MH_END(l->hwheel);
#undef MH_END
    return (maxbit + 7) / 8;
}

/* Parse a mouse report and inject the pointer events. Uses the report-map
 * derived field layout (hid_parser) when available; falls back to the fixed
 * boot layout ([buttons][int8 x][int8 y][int8 wheel]) otherwise. */
static void parse_and_inject(const struct raw_rep *r) {
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
    if (r->kind != REP_MOUSE) {
        forward_keys(r);
        return;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_LOG_REPORTS)
    /* Verbose per-report tracing (first 40 reports, then every 64th) plus a raw
     * hexdump. Off by default -- it floods the console while the mouse moves;
     * enable only when reverse-engineering a new mouse's report layout. */
    static uint32_t count;
    bool log_it = (count < 40 || (count & 0x3f) == 0);
    count++;
#else
    const bool log_it = false;
#endif

    int dx = 0, dy = 0, wheel = 0, pan = 0;
    uint8_t buttons = 0;
    const char *how;

    if (!r->is_boot && layout_valid) {
        /* HOGP notifications carry the report WITHOUT the leading report-id
         * byte, but the parsed layout's offsets include it (USB framing). Put
         * the report-id byte back so hid_extract_mouse() sees the framing it
         * expects and its own report-id check applies. */
        uint8_t buf[24];
        const uint8_t *rep;
        uint16_t rlen;
        if (layout.mouseReportId) {
            buf[0] = layout.mouseReportId;
            uint8_t cp = MIN(r->len, (uint8_t)(sizeof(buf) - 1));
            memcpy(buf + 1, r->buf, cp);
            rep = buf;
            rlen = (uint16_t)cp + 1;
        } else {
            rep = r->buf;
            rlen = r->len;
        }

        /* Decode only the report characteristic whose length matches the parsed
         * mouse report; ignore unrelated (e.g. vendor) input reports. */
        MouseState st;
        if (rlen != mouse_report_len || !hid_extract_mouse(&layout, rep, rlen, &st)) {
            if (log_it) {
                LOG_INF("ignored report h%u len%u (not the mouse report)", r->handle, r->len);
            }
            return;
        }
        how = "map";
        dx = st.dx;
        dy = st.dy;
        wheel = st.wheel;
        pan = st.hwheel;
        buttons = (uint8_t)st.buttons;
    } else {
        /* Boot protocol, or report map could not be parsed: fixed boot layout
         * [buttons][int8 x][int8 y][int8 wheel]. */
        how = "boot";
        if (r->len < 3) {
            if (log_it) {
                LOG_INF("short report h%u len %u", r->handle, r->len);
            }
            return;
        }
        buttons = r->buf[0];
        dx = (int8_t)r->buf[1];
        dy = (int8_t)r->buf[2];
        wheel = (r->len > 3) ? (int8_t)r->buf[3] : 0;
    }

    if (log_it) {
        LOG_INF("report h%u %s len%u: btn=0x%02x dx=%d dy=%d wh=%d pan=%d [%s]", r->handle,
                r->is_boot ? "boot" : "rep", r->len, buttons, dx, dy, wheel, pan, how);
        LOG_HEXDUMP_INF(r->buf, r->len, "  raw");
    }

    emit_motion(dx, dy, wheel, pan, buttons);
}

static void rep_work_cb(struct k_work *work) {
    struct raw_rep r;
    while (k_msgq_get(&rep_q, &r, K_NO_WAIT) == 0) {
        parse_and_inject(&r);
    }
}
K_WORK_DEFINE(rep_work, rep_work_cb);

/* Disconnect: release held buttons on the same workqueue as rep_work, after
 * dropping reports still queued, so prev_buttons has one owner and a queued
 * press can't re-press a button after its release. */
static void release_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    k_msgq_purge(&rep_q);
    release_all_buttons();
}
static K_WORK_DEFINE(release_work, release_work_cb);

/* ---- GATT notify ------------------------------------------------------- */

/* Which decoder a notification on this handle goes to. Keyboard / consumer reports
 * are told apart from the mouse by Report ID (Report Reference descriptor); anything
 * else keeps the mouse path, which gates on report length. */
static uint8_t report_kind(uint16_t handle) {
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
    for (int i = 0; i < n_reports; i++) {
        if (report_handles[i] != handle) {
            continue;
        }
        uint8_t id = report_ids[i];
        /* Without report ids in the descriptor (id 0) only trust it when there is no mouse. */
        if (id || !layout_valid) {
            if (kb_layout.valid && id == kb_layout.reportId) {
                return REP_KEYBOARD;
            }
            if (cc_layout.valid && id == cc_layout.reportId) {
                return REP_CONSUMER;
            }
        }
    }
#else
    ARG_UNUSED(handle);
#endif
    return REP_MOUSE;
}

static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length) {
    if (!data) {
        LOG_INF("mouse notification unsubscribed (handle %u)", params->value_handle);
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    struct raw_rep r = {
        .handle = params->value_handle,
        .is_boot = (params->value_handle == boot_mouse_handle),
        .kind = report_kind(params->value_handle),
        .len = MIN(length, sizeof(r.buf)),
    };
    memcpy(r.buf, data, r.len);
    int err = k_msgq_put(&rep_q, &r, K_NO_WAIT);
    if (err && r.kind != REP_MOUSE) {
        /* Key reports carry absolute state, so when the queue is full drop the oldest
         * report rather than this one: losing the latest could leave a key stuck. */
        struct raw_rep old;
        k_msgq_get(&rep_q, &old, K_NO_WAIT);
        err = k_msgq_put(&rep_q, &r, K_NO_WAIT);
    }
    if (err == 0) {
        k_work_submit(&rep_work);
    }
    return BT_GATT_ITER_CONTINUE;
}

/* ---- GATT discovery ---------------------------------------------------- */

static void do_subscribes(struct bt_conn *conn);

static uint8_t read_map_cb(struct bt_conn *conn, uint8_t err,
                           struct bt_gatt_read_params *params, const void *data,
                           uint16_t length) {
    if (data && length) {
        uint16_t cp = MIN(length, (uint16_t)(sizeof(report_map) - report_map_len));
        memcpy(report_map + report_map_len, data, cp);
        report_map_len += cp;
        return BT_GATT_ITER_CONTINUE; /* keep reading (blob) until the full map is returned */
    }

    LOG_HEXDUMP_INF(report_map, report_map_len, "HID report map");
    hid_parse_mouse(report_map, report_map_len, &layout);
    layout_valid = layout.x.valid;
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
    hid_parse_keys(report_map, report_map_len, &kb_layout, &cc_layout);
    LOG_INF("keys: keyboard=%s (rid %d, %s, mods=%s) consumer=%s (rid %d, %ux%u-bit)",
            kb_layout.valid ? "y" : "n", kb_layout.reportId, kb_layout.isBitmap ? "bitmap" : "array",
            kb_layout.hasMods ? "y" : "n", cc_layout.valid ? "y" : "n", cc_layout.reportId,
            cc_layout.count, cc_layout.size);
#endif
    if (layout_valid) {
        mouse_report_len = layout_report_len(&layout);
        if (layout.buttons.valid && layout.buttons.bitSize > 8) {
            /* emit_motion() forwards buttons 1-8 only (INPUT_BTN_0..7). */
            LOG_WRN("report map has %u buttons; only buttons 1-8 are forwarded",
                    layout.buttons.bitSize);
        }
        LOG_INF("layout: rid=%d len=%uB X(off%u sz%u) Y(off%u sz%u) wheel=%s pan=%s btn(off%u n%u)",
                layout.mouseReportId, mouse_report_len, layout.x.bitOffset, layout.x.bitSize,
                layout.y.bitOffset, layout.y.bitSize, layout.wheel.valid ? "y" : "n",
                layout.hwheel.valid ? "y" : "n", layout.buttons.bitOffset, layout.buttons.bitSize);
    } else {
        LOG_WRN("report map: no X/Y fields found; using boot-layout fallback parsing");
    }

    /* Subscribe only AFTER the map is read & parsed, so reports are decoded with
     * the correct layout from the very first notification. */
    do_subscribes(conn);
    return BT_GATT_ITER_STOP;
}

/* Battery level (BAS 0x2A19), read by UUID over the whole handle range, so no
 * service discovery is needed; a mouse without BAS just answers "not found".
 * Re-read every few minutes instead of subscribing: notify support is
 * optional in BAS, and a CCC auto-discovery past a non-notifying
 * characteristic could pick up the next characteristic's CCC. */
#define MH_BATTERY_PERIOD K_MINUTES(5)
static struct bt_gatt_read_params bas_read;

static uint8_t bas_read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
                           const void *data, uint16_t length) {
    if (!err && data && length >= 1) {
        battery = MIN(((const uint8_t *)data)[0], 100);
        mh_notify();
    }
    return BT_GATT_ITER_STOP;
}

static void battery_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(battery_work, battery_work_cb);

static void battery_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    /* mouse_conn belongs to the BT thread (on_disconnected unrefs it), so
     * take our own reference by address instead of reading it here. */
    if (!have_bond) {
        return;
    }
    struct bt_conn *c = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &mouse_addr);
    if (!c) {
        return;
    }
    bas_read.func = bas_read_cb;
    bas_read.handle_count = 0; /* read by UUID */
    bas_read.by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    bas_read.by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    bas_read.by_uuid.uuid = BT_UUID_BAS_BATTERY_LEVEL;
    bt_gatt_read(c, &bas_read); /* fails harmlessly if the link just dropped */
    bt_conn_unref(c);
    k_work_reschedule(&battery_work, MH_BATTERY_PERIOD);
}

static void do_subscribe(struct bt_conn *conn, uint16_t value_handle, int idx) {
    sub[idx].notify = notify_cb;
    sub[idx].value = BT_GATT_CCC_NOTIFY;
    sub[idx].value_handle = value_handle;
    sub[idx].ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE; /* auto-discover */
    sub[idx].disc_params = &sub_disc[idx];
    sub[idx].end_handle = svc_end;

    int err = bt_gatt_subscribe(conn, &sub[idx]);
    if (err && err != -EALREADY) {
        LOG_ERR("subscribe (handle %u) failed (err %d)", value_handle, err);
    } else {
        LOG_INF("subscribed to input report (handle %u)", value_handle);
    }
}

/* Subscribe to every notifying HID input report (report protocol) AND, if
 * present, the boot mouse report -- so we receive pointer data regardless of the
 * mouse's active protocol mode. Most BLE mice ignore a switch to boot protocol
 * and keep sending report-protocol notifications. Protocol Mode resets to Report
 * on each connection (per HOGP), so we leave it at the default. */
static void do_subscribes(struct bt_conn *conn) {
    n_subs = 0;
    for (int i = 0; i < n_reports && n_subs < MAX_SUBS; i++) {
        do_subscribe(conn, report_handles[i], n_subs++);
    }
    if (boot_mouse_handle && n_subs < MAX_SUBS) {
        do_subscribe(conn, boot_mouse_handle, n_subs++);
    }
    use_boot = (n_reports == 0);

    LOG_INF("mouse ready: %d subscription(s) (%d report char(s) + %s boot)", n_subs, n_reports,
            boot_mouse_handle ? "1" : "0");
    /* After the subscribe writes have had time to land (a read issued while
     * they are in flight can fail, see subscribe_phase()). */
    k_work_reschedule(&battery_work, K_SECONDS(2));
    mh_notify();
}

#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
/* Read each notifying report characteristic's Report Reference descriptor
 * [report id, type], one at a time, then go on to subscribe_phase(). */
static struct bt_gatt_read_params ref_read;
static int ref_idx;

static void subscribe_phase(struct bt_conn *conn);

static void read_next_ref(struct bt_conn *conn);

static uint8_t ref_read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
                           const void *data, uint16_t length) {
    if (!err && data && length >= 2) {
        report_ids[ref_idx] = ((const uint8_t *)data)[0];
        LOG_INF("report char h%u: report id %u", report_handles[ref_idx], report_ids[ref_idx]);
    }
    ref_idx++;
    read_next_ref(conn);
    return BT_GATT_ITER_STOP;
}

static void read_next_ref(struct bt_conn *conn) {
    for (; ref_idx < n_reports; ref_idx++) {
        ref_read.func = ref_read_cb;
        ref_read.handle_count = 0; /* read by UUID */
        ref_read.by_uuid.start_handle = report_handles[ref_idx] + 1;
        ref_read.by_uuid.end_handle = report_end[ref_idx];
        ref_read.by_uuid.uuid = BT_UUID_HIDS_REPORT_REF;
        if (ref_read.by_uuid.start_handle <= ref_read.by_uuid.end_handle &&
            bt_gatt_read(conn, &ref_read) == 0) {
            return; /* ref_read_cb continues the chain */
        }
    }
    subscribe_phase(conn);
}
#endif

/* Read & parse the report map FIRST, then subscribe (in read_map_cb). A GATT
 * read issued while subscribe CCC writes are in flight can fail or be starved,
 * which previously left us decoding reports with the wrong (boot) layout. */
static void subscribe_phase(struct bt_conn *conn) {
    if (map_handle) {
        report_map_len = 0;
        layout_valid = false;
        map_read.func = read_map_cb;
        map_read.handle_count = 1;
        map_read.single.handle = map_handle;
        map_read.single.offset = 0;
        int err = bt_gatt_read(conn, &map_read);
        if (err) {
            LOG_ERR("report map read failed (err %d); subscribing with boot fallback", err);
            do_subscribes(conn);
        }
    } else {
        LOG_WRN("no report map characteristic; using boot-layout fallback parsing");
        do_subscribes(conn);
    }
}

static uint8_t disc_chrc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params) {
    /* The previous report characteristic ends right before this declaration. */
    if (pending_end >= 0) {
        report_end[pending_end] = attr ? attr->handle - 1 : svc_end;
        pending_end = -1;
    }
    if (!attr) {
        LOG_INF("HID characteristics: %d report(s), boot=%s, proto=%s, map=%s", n_reports,
                boot_mouse_handle ? "y" : "n", proto_handle ? "y" : "n", map_handle ? "y" : "n");
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
        ref_idx = 0;
        read_next_ref(conn);
#else
        subscribe_phase(conn);
#endif
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *c = attr->user_data;
    if (!bt_uuid_cmp(c->uuid, BT_UUID_HIDS_PROTOCOL_MODE)) {
        proto_handle = c->value_handle;
    } else if (!bt_uuid_cmp(c->uuid, BT_UUID_HIDS_REPORT_MAP)) {
        map_handle = c->value_handle;
    } else if (!bt_uuid_cmp(c->uuid, BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT)) {
        boot_mouse_handle = c->value_handle;
    } else if (!bt_uuid_cmp(c->uuid, BT_UUID_HIDS_REPORT)) {
        if ((c->properties & BT_GATT_CHRC_NOTIFY) && n_reports < MAX_REPORTS) {
            report_ids[n_reports] = 0;
            pending_end = n_reports;
            report_handles[n_reports++] = c->value_handle;
        }
    }
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t disc_service_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               struct bt_gatt_discover_params *params) {
    if (!attr) {
        if (pairing_attempt) {
            /* We connected to a scan match that has no HID service -- a device
             * that advertised the mouse appearance but isn't one. Undo the bond
             * just formed with it and disconnect, so we never reconnect to it;
             * mh_begin() then idles until the user triggers pairing again
             * (clear_bond() also drops a pair-by-address target). */
            LOG_WRN("pairing target has no HID service; not a mouse -- dropping it");
            if (have_target) {
                target_failed();
            } else {
                clear_bond();
            }
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        } else {
            /* A real bonded mouse always has the service, so this is unexpected.
             * Leave the link up rather than risk a reconnect loop or wiping a
             * good bond on a transient glitch. */
            LOG_WRN("no HID service found on bonded mouse (unexpected)");
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *v = attr->user_data;
    svc_end = v->end_handle;
    LOG_INF("HID service handles %u..%u", attr->handle, svc_end);

    disc.uuid = NULL;
    disc.func = disc_chrc_cb;
    disc.start_handle = attr->handle + 1;
    disc.end_handle = svc_end;
    disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &disc);
    if (err) {
        LOG_ERR("characteristic discovery failed (err %d)", err);
    }
    return BT_GATT_ITER_STOP;
}

static void discovery_retry_fn(struct k_work *work) {
    ARG_UNUSED(work);
    if (mouse_conn) {
        start_discovery(mouse_conn);
    }
}
static K_WORK_DELAYABLE_DEFINE(discovery_retry, discovery_retry_fn);

static void start_discovery(struct bt_conn *conn) {
    if (disc_started) {
        return;
    }
    disc_started = true;

    n_reports = 0;
    pending_end = -1;
    proto_handle = map_handle = boot_mouse_handle = 0;
    /* Drop any layout parsed from a previous mouse. subscribe_phase() normally
     * clears this before re-reading the report map, but not if the (new) mouse
     * has no Report Map characteristic -- without this reset we would then keep
     * decoding with the old mouse's layout instead of falling back to boot. */
    layout_valid = false;
    mouse_report_len = 0;
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
    kb_layout.valid = false;
    cc_layout.valid = false;
#endif

    disc.uuid = BT_UUID_HIDS;
    disc.func = disc_service_cb;
    disc.start_handle = 0x0001;
    disc.end_handle = 0xffff;
    disc.type = BT_GATT_DISCOVER_PRIMARY;

    /* Retry budget for THIS discovery episode. Reset once a discover request is
     * accepted (below) so the 5 attempts count per stuck episode, not once over
     * the whole device lifetime -- otherwise a handful of transient failures
     * spread across many reconnects would permanently exhaust the budget and a
     * later genuinely-recoverable failure would never be retried. */
    static uint8_t retry_count;

    int err = bt_gatt_discover(conn, &disc);
    if (err) {
        /* Transient failures (e.g. -ENOMEM from a momentarily exhausted ATT
         * request pool, more likely right after a fast reconnect) would
         * otherwise leave the mouse connected+secured but never subscribed --
         * connected but silently unresponsive. Retry a few times rather than
         * giving up on the first failure. */
        disc_started = false;
        if (retry_count < 5) {
            retry_count++;
            LOG_WRN("service discovery failed (err %d); retrying shortly (%d/5)", err,
                    retry_count);
            k_work_reschedule(&discovery_retry, K_MSEC(300));
            return;
        }
        retry_count = 0;
        LOG_ERR("service discovery failed (err %d)", err);
        return;
    }
    retry_count = 0;
}

/* ---- scanning / connecting --------------------------------------------- */

/* First-match pairing (&mouse_pair) matches the Mouse appearance (0x03C2)
 * specifically -- NOT the whole 0x03C0-0x03CF HID range and NOT the HID
 * service UUID (0x1812), both of which ZMK's own keyboard advertising also
 * carries, so a broader match connects to the wrong device (e.g. the
 * keyboard's own split half). The list scan is broader (ad_is_hid()): a person
 * picks from it, so a keyboard in the list costs nothing. */

#if DT_INST_NODE_HAS_PROP(0, pair_name_prefix)
#define PAIR_NAME_PREFIX DT_INST_PROP(0, pair_name_prefix)
BUILD_ASSERT(sizeof(PAIR_NAME_PREFIX) > 1, "pair-name-prefix must not be empty");
#endif

static void connect_to(const bt_addr_le_t *addr) {
    connecting = true;
    stop_own_scan();

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, MH_CONN_PARAM, &mouse_conn);
    if (err) {
        LOG_ERR("create connection failed (err %d)", err);
        connecting = false;
        /* -EALREADY: ZMK's split central is initiating. Back off so it can finish. */
        k_work_reschedule(&mh_retry_work, MH_RETRY_DELAY);
    }
    mh_notify();
}

static struct zmk_ble_mouse_host_scan_result *find_result(const bt_addr_le_t *addr) {
    for (int i = 0; i < n_results; i++) {
        if (!bt_addr_le_cmp(&results[i].addr, addr)) {
            return &results[i];
        }
    }
    return NULL;
}

/* List scan: remember every HID advertiser once (refreshing RSSI). The name
 * often only comes in the scan response (active scan), which carries no
 * appearance or UUID, so a response only fills in a known entry. */
static void list_add(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, const AdInfo *ai) {
    struct zmk_ble_mouse_host_scan_result *r = find_result(addr);
    if (!r) {
        /* ponytail: first come, first kept; replace the weakest entry if a
         * crowded room ever fills the list with strangers. */
        if (adv_type == BT_GAP_ADV_TYPE_SCAN_RSP || !ad_is_hid(ai) ||
            n_results >= ARRAY_SIZE(results)) {
            return;
        }
        r = &results[n_results++];
        memset(r, 0, sizeof(*r));
        bt_addr_le_copy(&r->addr, addr);
    }
    r->rssi = rssi;
    if (ai->appearance) {
        r->appearance = ai->appearance;
    }
    if (ai->name_len && !r->name[0]) {
        memcpy(r->name, ai->name, sizeof(r->name));
    }
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
                    struct net_buf_simple *ad) {
    AdInfo ai;
    ad_parse(ad->data, ad->len, &ai);

    if (listing) {
        list_add(addr, rssi, adv_type, &ai);
    }

    /* The rest is first-match pairing (&mouse_pair), which only runs while
     * unbonded: reconnect to a known mouse uses direct connect, not scanning
     * (see mh_begin() / file header). Never match while bonded. */
    if (mouse_conn || connecting || have_bond || !pairing_requested) {
        return;
    }

    /* Broad-match, since we don't know the mouse's address ahead of time.
     * Directed adverts carry no AD payload and can't be appearance-matched,
     * so they're intentionally not accepted here -- a device we've never
     * bonded to has nothing to reconnect to us for anyway. With
     * pair-name-prefix, match the advertised name instead (appearance
     * ignored); the name may be in the scan response (active scan). */
    bool match = false;
#if DT_INST_NODE_HAS_PROP(0, pair_name_prefix)
    match = (adv_type == BT_GAP_ADV_TYPE_ADV_IND || adv_type == BT_GAP_ADV_TYPE_SCAN_RSP) &&
            ai.name_len >= sizeof(PAIR_NAME_PREFIX) - 1 &&
            memcmp(ai.name, PAIR_NAME_PREFIX, sizeof(PAIR_NAME_PREFIX) - 1) == 0;
#else
    match = adv_type == BT_GAP_ADV_TYPE_ADV_IND && ai.appearance == AD_APPEARANCE_HID_MOUSE;
#endif
    if (!match) {
        return;
    }

    char s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, s, sizeof(s));
    LOG_INF("found HID device %s (rssi %d), connecting", s, rssi);
    /* The name may only be in a scan response we never see now; then "". */
    memcpy(mouse_name, ai.name, sizeof(mouse_name));
    pairing_attempt = true; /* scanning only ever runs during fresh pairing */
    connect_to(addr);
}

static void scan_retry_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(scan_retry, scan_retry_fn);

static void start_scan(void) {
    mode = MH_SCAN;
    if (scan_owned) {
        return; /* our scan is already running (e.g. a stale retry fired) */
    }
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    if (err == -EALREADY) {
        /* Someone else (ZMK's split central) owns the scanner. Don't claim it, or
         * stop it later; wait and retry so pairing works once their scan ends. */
        LOG_WRN("scan busy (another scan is running); retrying in %d ms",
                CONFIG_ZMK_BLE_MOUSE_HOST_RECONNECT_DELAY_MS);
        k_work_reschedule(&scan_retry, MH_RETRY_DELAY);
    } else if (err) {
        /* A pairing scan can be requested while the BT stack is still busy
         * (e.g. -EBUSY), so a start can transiently fail. Retry shortly rather
         * than going silent until some other event (disconnect, another
         * pairing key press) happens to call mh_begin() again. */
        LOG_ERR("scan start failed (err %d); retrying shortly", err);
        k_work_reschedule(&scan_retry, K_MSEC(200));
    } else {
        scan_owned = true;
        LOG_INF("scanning for a HID mouse (%s)", listing ? "list" : "pair");
    }
    mh_notify();
}

static void scan_retry_fn(struct k_work *work) {
    ARG_UNUSED(work);
    /* A list scan may run beside a connected mouse; a pairing scan may not. */
    if (mode == MH_SCAN && (listing || (!mouse_conn && !connecting))) {
        start_scan();
    }
}

static void mh_begin(void) {
    if (mouse_conn || connecting) {
        return;
    }
    if (listing) {
        /* A list scan owns the scanner (initiating would stop it); it calls
         * mh_begin() again when it ends. */
        return;
    }
    if (mh_paused) {
        LOG_INF("mouse host paused; waiting for a pairing trigger");
        return;
    }
    if (have_bond) {
        /* Connect directly to the known bonded address (bt_conn_le_create,
         * "Initiating State") instead of scanning-then-matching -- the mouse's
         * directed reconnect adverts never reach a scan here (see file header). */
        char s[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&mouse_addr, s, sizeof(s));
        LOG_INF("connecting directly to bonded mouse %s", s);
        pairing_attempt = false; /* reconnect to a known bond, not a fresh pair */
        connect_to(&mouse_addr);
    } else if (have_target) {
        /* Pair-by-address: its address came from a list scan, so connect to it
         * directly. A few tries (it may have left pairing mode), then give up. */
        if (target_tries++ >= MH_TARGET_TRIES) {
            LOG_WRN("PAIR: chosen mouse did not pair after %d tries; giving up", MH_TARGET_TRIES);
            target_failed();
            return;
        }
        pairing_attempt = true;
        connect_to(&target_addr);
    } else if (pairing_requested) {
        /* No bond yet, but the user just pressed mouse_pair: broad-match scan
         * by HID appearance, since we don't know the mouse's address ahead of
         * time. */
        start_scan();
    } else {
        LOG_INF("no mouse bonded; waiting for a pairing trigger");
    }
}

/* List scan window over: hand the radio back (a first-match pairing scan that
 * was already running keeps going) and resume reconnecting. */
static void list_end_fn(struct k_work *work) {
    ARG_UNUSED(work);
    if (!listing) {
        return;
    }
    if (pairing_requested && !have_bond && !mouse_conn) {
        listing = false;
        mh_notify();
    } else {
        stop_own_scan();
    }
    mh_begin();
}
static K_WORK_DELAYABLE_DEFINE(list_end, list_end_fn);

/* ---- connection callbacks (shared; filter to our central mouse link) --- */

static void on_connected(struct bt_conn *conn, uint8_t err) {
    /* bt_conn_cb callbacks are GLOBAL -- they fire for every connection on the
     * device (including ZMK's own split-central link, also role CENTRAL), so
     * ignore any conn that isn't ours. connect_to() sets mouse_conn before this
     * can fire for our own attempt, so the identity check is sufficient. */
    if (conn != mouse_conn) {
        return;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0 || info.role != BT_CONN_ROLE_CENTRAL) {
        return; /* defensive; should be unreachable if conn == mouse_conn */
    }

    connecting = false;

    if (err) {
        LOG_ERR("mouse connection failed (err 0x%02x)", err);
        bt_conn_unref(mouse_conn);
        mouse_conn = NULL;
        k_work_reschedule(&mh_retry_work, MH_RETRY_DELAY);
        mh_notify();
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("PAIR: connected to mouse %s (had bond: %s); securing", addr,
            have_bond ? "yes" : "no");
    int sec = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec) {
        LOG_WRN("set_security failed (err %d); discovering without encryption", sec);
        start_discovery(conn);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != mouse_conn) {
        return;
    }

    LOG_INF("mouse disconnected (reason 0x%02x)", reason);
    k_work_submit(&release_work);
    k_work_cancel_delayable(&battery_work);
    battery = 0xFF;
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORWARD_KEYS)
    /* Events must not be raised from this (BT) context; the workqueue does it. */
    k_work_submit(&release_keys_work);
#endif

    bt_conn_unref(mouse_conn);
    mouse_conn = NULL;
    connecting = false;
    disc_started = false;
    n_subs = 0;
    for (int i = 0; i < MAX_SUBS; i++) {
        sub[i].value_handle = 0;
    }

    k_work_reschedule(&mh_retry_work, K_NO_WAIT);
    mh_notify();
}

/* Also drops a pair-by-address target: every caller either starts a new
 * pairing (and sets its own target after this) or abandons the current one. */
static void clear_bond(void) {
    if (have_bond) {
        bt_unpair(BT_ID_DEFAULT, &mouse_addr);
    }
    if (have_prev) {
        bt_unpair(BT_ID_DEFAULT, &prev_addr);
        have_prev = false;
    }
    settings_delete("mouse_host/addr");
    settings_delete("mouse_host/name");
    have_bond = false;
    have_target = false;
    memset(&mouse_addr, 0, sizeof(mouse_addr));
    mouse_name[0] = '\0';
    LOG_INF("cleared stored mouse bond");
    mh_notify();
}

/* A pair-by-address attempt failed: drop whatever half-made bond it left and
 * go back to the previous mouse, if there was one. */
static void target_failed(void) {
    if (!have_target) {
        return;
    }
    have_target = false;
    if (!have_prev || bt_addr_le_cmp(&prev_addr, &target_addr)) {
        bt_unpair(BT_ID_DEFAULT, &target_addr);
    }
    if (have_prev) {
        bt_addr_le_copy(&mouse_addr, &prev_addr);
        memcpy(mouse_name, prev_name, sizeof(mouse_name));
        have_bond = true;
        have_prev = false;
        LOG_INF("PAIR: back to the previous mouse");
    } else {
        mouse_name[0] = '\0';
    }
    k_work_reschedule(&mh_retry_work, MH_RETRY_DELAY);
    mh_notify();
}

void zmk_ble_mouse_host_pair(void) {
    LOG_INF("PAIR: pairing trigger -- forgetting any bonded mouse and scanning");
    mh_paused = false;
    pairing_requested = true;
    clear_bond();
    if (mouse_conn) {
        /* on_disconnected() will defer to mh_begin() (via mh_retry_work),
         * which (mh_paused now false, pairing_requested now true) starts a
         * fresh scan. */
        bt_conn_disconnect(mouse_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    } else {
        stop_own_scan(); /* in case a stale scan of ours for the old bond is running */
        mh_begin();
    }
}

void zmk_ble_mouse_host_unpair(void) {
    LOG_INF("PAIR: unpair trigger -- forgetting the bonded mouse and pausing");
    mh_paused = true;
    pairing_requested = false;
    clear_bond();
    if (mouse_conn) {
        bt_conn_disconnect(mouse_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    } else {
        stop_own_scan();
    }
}

/* Temporarily stop all radio activity (scan / reconnect-initiate / the mouse
 * link) WITHOUT forgetting the bond, then resume. Used to keep the RF quiet
 * during a timing-critical IR bit-bang (see zmk_ir_tx_active below). */
void zmk_ble_mouse_host_pause(void) {
    mh_paused = true;
    if (mode == MH_SCAN) {
        stop_own_scan();
    }
    if (mouse_conn) {
        /* Disconnects a live link, or cancels an in-flight reconnect-initiate
         * (the biggest RF-jitter source). on_disconnected -> mh_begin sees
         * mh_paused and idles. */
        bt_conn_disconnect(mouse_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

void zmk_ble_mouse_host_resume(void) {
    if (!mh_paused) {
        return;
    }
    mh_paused = false;
    mh_begin(); /* reconnect to the bond (if any) */
}

/* zmk-ir hook (weak-overridden): quiet the mouse link's RF while IR TX bit-bangs
 * the NEC waveform, so its interrupts don't jitter the timing. The hardware-PWM
 * IR backend (CONFIG_IR_TX_HW_PWM) clocks the carrier in hardware and is immune
 * to jitter, so there's nothing to quiet -- skip the pause/resume then (otherwise
 * every IR press would needlessly drop and reconnect the mouse). */
void zmk_ir_tx_active(bool active) {
#if IS_ENABLED(CONFIG_IR_TX_HW_PWM)
    ARG_UNUSED(active);
#else
    if (active) {
        zmk_ble_mouse_host_pause();
        k_msleep(30); /* let the scan/initiate/disconnect actually settle */
    } else {
        zmk_ble_mouse_host_resume();
    }
#endif
}

void zmk_ble_mouse_host_dump(void) {
    char s[BT_ADDR_LE_STR_LEN];

    LOG_INF("=== BLE mouse host ===");
    if (have_bond) {
        bt_addr_le_to_str(&mouse_addr, s, sizeof(s));
        LOG_INF("  bonded mouse: %s", s);
    } else {
        LOG_INF("  bonded mouse: (none)");
    }

    if (mouse_conn) {
        bt_addr_le_to_str(bt_conn_get_dst(mouse_conn), s, sizeof(s));
        LOG_INF("  connection: CONNECTED to %s", s);
    } else if (mh_paused) {
        LOG_INF("  connection: paused (unpaired -- press mouse_pair to resume)");
    } else if (connecting) {
        LOG_INF("  connection: connecting...");
    } else if (mode == MH_SCAN) {
        LOG_INF("  connection: scanning...");
    } else {
        LOG_INF("  connection: idle");
    }

    if (layout_valid) {
        LOG_INF("  report layout: rid=%d len=%uB X/Y=%s wheel=%s pan=%s buttons=%s(n=%d)",
                layout.mouseReportId, mouse_report_len,
                (layout.x.valid && layout.y.valid) ? "ok" : "MISSING",
                layout.wheel.valid ? "y" : "n", layout.hwheel.valid ? "y" : "n",
                layout.buttons.valid ? "y" : "n", layout.buttons.bitSize);
    } else {
        LOG_INF("  report layout: not parsed from report map (using fallback parser)");
    }
    LOG_INF("  subscriptions: %d, protocol: %s", n_subs, use_boot ? "boot" : "report");
}

void zmk_ble_mouse_host_get_status(struct zmk_ble_mouse_host_status *out) {
    memset(out, 0, sizeof(*out));
    if (mouse_conn && !connecting && n_subs > 0) {
        out->state = ZMK_BLE_MOUSE_HOST_CONNECTED;
    } else if (mode == MH_SCAN) {
        out->state = ZMK_BLE_MOUSE_HOST_SCANNING;
    } else if (!mh_paused && (have_bond || have_target || mouse_conn)) {
        out->state = ZMK_BLE_MOUSE_HOST_CONNECTING;
    } else {
        out->state = ZMK_BLE_MOUSE_HOST_IDLE;
    }
    out->bonded = have_bond;
    out->listing = listing;
    if (have_bond) {
        bt_addr_le_copy(&out->addr, &mouse_addr);
    } else if (have_target) {
        bt_addr_le_copy(&out->addr, &target_addr);
    }
    memcpy(out->name, mouse_name, sizeof(out->name));
    out->battery = battery;
    bool parsed = mouse_conn && layout_valid;
    out->buttons = (parsed && layout.buttons.valid) ? layout.buttons.bitSize : 0;
    out->wheel = parsed && layout.wheel.valid;
    out->pan = parsed && layout.hwheel.valid;
}

void zmk_ble_mouse_host_scan_start(void) {
    LOG_INF("PAIR: list scan for %d s", CONFIG_ZMK_BLE_MOUSE_HOST_LIST_SCAN_S);
    n_results = 0;
    listing = true;
    if (connecting && mouse_conn) {
        /* Cancel a pending reconnect-initiate (the bonded mouse is likely off):
         * the scanner cannot start beside it. on_connected() sees the cancel,
         * and mh_begin() waits for the list scan to end. */
        bt_conn_disconnect(mouse_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
    k_work_cancel_delayable(&scan_retry);
    start_scan();
    k_work_reschedule(&list_end, K_SECONDS(CONFIG_ZMK_BLE_MOUSE_HOST_LIST_SCAN_S));
}

void zmk_ble_mouse_host_scan_stop(void) { k_work_reschedule(&list_end, K_NO_WAIT); }

int zmk_ble_mouse_host_scan_count(void) { return n_results; }

int zmk_ble_mouse_host_scan_get(int idx, struct zmk_ble_mouse_host_scan_result *out) {
    if (idx < 0 || idx >= n_results) {
        return -ENOENT;
    }
    *out = results[idx];
    return 0;
}

void zmk_ble_mouse_host_pair_addr(const bt_addr_le_t *addr) {
    char s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, s, sizeof(s));
    LOG_INF("PAIR: pairing chosen device %s", s);

    struct zmk_ble_mouse_host_scan_result *r = find_result(addr);
    bt_addr_le_t a;
    bt_addr_le_copy(&a, addr); /* addr may point into results[] */

    /* First: ends a list scan (listing = false), or mh_begin() would wait
     * on it forever when the link below drops. */
    stop_own_scan();
    k_work_cancel_delayable(&list_end);
    mh_paused = false;
    pairing_requested = false; /* not a first-match scan */
    if (have_bond) {
        /* Keep the bond (keys + saved settings) until the new one forms. */
        bt_addr_le_copy(&prev_addr, &mouse_addr);
        memcpy(prev_name, mouse_name, sizeof(prev_name));
        have_prev = true;
        have_bond = false;
    }
    bt_addr_le_copy(&target_addr, &a);
    have_target = true;
    target_tries = 0;
    memcpy(mouse_name, r ? r->name : "", r ? sizeof(mouse_name) : 1);
    if (mouse_conn) {
        /* on_disconnected() / on_connected(err) defer to mh_begin(), which
         * connects to the target. */
        bt_conn_disconnect(mouse_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    } else {
        mh_begin();
    }
    mh_notify();
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    if (conn != mouse_conn) {
        return;
    }
    if (err) {
        /* The peer rejected our keys -- typically because its pairing was
         * removed on its side. Drop the stale bond and re-pair from scratch.
         * Defense in depth: only clear the stored bond if this connection is
         * actually to that bonded address. scan_cb() should never connect us
         * to anything else while have_bond is true, but if it somehow did, a
         * security failure with an unrelated peer must NOT wipe the real
         * mouse's bond. */
        bool is_bonded_peer = have_bond && !bt_addr_le_cmp(bt_conn_get_dst(conn), &mouse_addr);
        if (have_target && pairing_attempt) {
            LOG_WRN("chosen mouse refused pairing (err %d)", err);
            target_failed();
        } else if (is_bonded_peer || !have_bond) {
            LOG_WRN("mouse security failed (err %d); clearing stale bond to re-pair", err);
            clear_bond();
        } else {
            LOG_WRN("security failed (err %d) with a peer that isn't our bonded mouse; "
                     "leaving the stored bond alone",
                     err);
        }
        bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL); /* triggers rescan via disconnect */
        return;
    }
    LOG_INF("PAIR: security established, level %d (encrypted)", level);
    if (level >= BT_SECURITY_L2) {
        start_discovery(conn);
    }
}

BT_CONN_CB_DEFINE(mh_conn_cb) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .security_changed = on_security_changed,
};

/* ---- pairing completion (additional info callback; multiple allowed) --- */

static void on_pairing_complete(struct bt_conn *conn, bool bonded) {
    if (conn != mouse_conn) {
        return; /* ignore host pairing */
    }

    bt_addr_le_copy(&mouse_addr, bt_conn_get_dst(conn));
    have_bond = true;
    have_target = false;
    if (have_prev && bt_addr_le_cmp(&prev_addr, &mouse_addr)) {
        bt_unpair(BT_ID_DEFAULT, &prev_addr); /* the old mouse, now replaced */
    }
    have_prev = false;
    pairing_requested = false; /* pairing session complete; back to reconnect-only */
    int err = settings_save_one("mouse_host/addr", &mouse_addr, sizeof(mouse_addr));
    settings_save_one("mouse_host/name", mouse_name, strlen(mouse_name));
    mh_notify();

    char s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&mouse_addr, s, sizeof(s));
    LOG_INF("PAIR: bonded & %s mouse %s (settings save: %s)", bonded ? "saved" : "NOT-bonded", s,
            err ? "FAILED" : "ok");
}

static struct bt_conn_auth_info_cb mh_auth_info_cb = {
    .pairing_complete = on_pairing_complete,
};

/* ---- startup ----------------------------------------------------------- */

static void mh_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    bt_conn_auth_info_cb_register(&mh_auth_info_cb);

    while (!bt_is_ready()) {
        k_sleep(K_MSEC(200));
    }
    /* Let ZMK's BLE init / advertising settle before we add a central role.
     * Not load-bearing for reconnect timing (that's handled by connecting
     * directly to the known address, not by racing a scan window -- see the
     * file header comment), just a safety margin before touching the BT
     * stack. */
    k_sleep(K_MSEC(1000));

#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_HOST_FORGET_ON_BOOT)
    /* Free all BLE host profiles (and re-advertise) so a host can pair freshly,
     * even without a &bt BT_CLR key in the keymap. Leaves the mouse bond intact
     * so the mouse keeps working. */
    LOG_WRN("FORGET_ON_BOOT: clearing all BLE host profiles for fresh host pairing");
    zmk_ble_clear_all_bonds();
#endif

    if (have_bond) {
        char s[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&mouse_addr, s, sizeof(s));
        LOG_INF("PAIR: starting with stored bond %s", s);
    } else {
        LOG_INF("PAIR: starting with no stored bond");
    }
    mh_begin();
}

K_THREAD_DEFINE(mh_tid, 2048, mh_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                2000);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
