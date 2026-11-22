/*
 * Copyright (c) 2026 Tadashi Kadowaki
 * SPDX-License-Identifier: MIT
 */

#pragma once

/* Forget the bonded mouse (if any), disconnect it (if connected), and
 * actively scan to pair a new one. Intended for a keymap trigger. */
void zmk_ble_mouse_host_pair(void);

/* Forget the bonded mouse (if any), disconnect it (if connected), and stop
 * trying to reconnect until zmk_ble_mouse_host_pair() is called again.
 * Intended for a keymap trigger. */
void zmk_ble_mouse_host_unpair(void);

/* Log the mouse bond address, connection state, and (if known) the parsed HID
 * report layout to the console. Intended for a keymap trigger. */
void zmk_ble_mouse_host_dump(void);

/* Temporarily stop all RF activity (scan / reconnect / the mouse link) without
 * forgetting the bond, then resume. Useful to keep the radio quiet during a
 * timing-critical operation elsewhere (the module uses this to go quiet during
 * an IR bit-bang via zmk-ir's zmk_ir_tx_active() hook). */
void zmk_ble_mouse_host_pause(void);
void zmk_ble_mouse_host_resume(void);

/* Set scroll-wheel direction inversion for the forwarded mouse:
 * 0 = off, 1 = on, other = toggle. Normally driven automatically from the
 * active output endpoint (see the wheel-invert-* devicetree properties); this
 * is also exposed as the zmk,behavior-mouse-host-wheel-invert keymap trigger. */
void zmk_ble_mouse_host_wheel_invert(int action);

/* ---- status, scan list and pair-by-address (AJ fork) -------------------
 * For a configurator channel (Totem-ZMK's Flask mouse-host channel) and a
 * status widget. The getters only copy state and are safe from any thread.
 * The actions start BT operations, so call them from a thread, not from a
 * BT callback (the Flask channel defers them to the system workqueue). */

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/bluetooth/addr.h>
#include <zmk/event_manager.h>
#include <zmk_ble_mouse_host/ad_parse.h>

#define ZMK_BLE_MOUSE_HOST_NAME_MAX AD_NAME_MAX

enum zmk_ble_mouse_host_state {
    /* Nothing to reach: no bond and no pairing running, or unpaired. */
    ZMK_BLE_MOUSE_HOST_IDLE = 0,
    /* Our scan is running (list scan or a first-match &mouse_pair scan). */
    ZMK_BLE_MOUSE_HOST_SCANNING = 1,
    /* Trying to reach the bonded or chosen mouse (it may simply be off). */
    ZMK_BLE_MOUSE_HOST_CONNECTING = 2,
    /* Connected and subscribed: reports flow. */
    ZMK_BLE_MOUSE_HOST_CONNECTED = 3,
};

struct zmk_ble_mouse_host_status {
    enum zmk_ble_mouse_host_state state;
    bool bonded;
    bool listing;      /* a zmk_ble_mouse_host_scan_start() list scan is running */
    bt_addr_le_t addr; /* bonded mouse, else the pairing target; zero if none */
    char name[ZMK_BLE_MOUSE_HOST_NAME_MAX + 1]; /* from its advertising; "" if unknown */
    uint8_t battery;   /* BAS level 0-100, 0xFF = unknown / no BAS */
    uint8_t buttons;   /* buttons in its report map, 0 = unknown */
};

void zmk_ble_mouse_host_get_status(struct zmk_ble_mouse_host_status *out);

struct zmk_ble_mouse_host_scan_result {
    bt_addr_le_t addr;
    int8_t rssi;         /* last seen */
    uint16_t appearance; /* 0 = not advertised; 0x03C2 mouse, 0x03C1 keyboard */
    char name[ZMK_BLE_MOUSE_HOST_NAME_MAX + 1];
};

/* List nearby HID devices (Mouse appearance or HID service UUID in their
 * advertising) for CONFIG_ZMK_BLE_MOUSE_HOST_LIST_SCAN_S seconds. Clears the
 * previous list. Never connects, and keeps a connected mouse connected; a
 * pending reconnect to the bonded mouse waits until the list scan ends. */
void zmk_ble_mouse_host_scan_start(void);
void zmk_ble_mouse_host_scan_stop(void);
int zmk_ble_mouse_host_scan_count(void);
/* 0, or -ENOENT past the end. */
int zmk_ble_mouse_host_scan_get(int idx, struct zmk_ble_mouse_host_scan_result *out);

/* Forget the bonded mouse and pair the device at addr (it must be in pairing
 * mode). Retries a few times, then gives up (state back to IDLE). */
void zmk_ble_mouse_host_pair_addr(const bt_addr_le_t *addr);

/* Raised (from the system workqueue) whenever anything in the status may have
 * changed; read zmk_ble_mouse_host_get_status() for the new values. */
struct zmk_ble_mouse_host_state_changed {
    enum zmk_ble_mouse_host_state state;
};

ZMK_EVENT_DECLARE(zmk_ble_mouse_host_state_changed);
