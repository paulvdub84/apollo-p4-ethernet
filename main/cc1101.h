#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int pin_cs;
    int pin_sck;
    int pin_mosi;
    int pin_miso;
    int pin_gdo0;
    int pin_gdo2;
} cc1101_pins_t;

typedef struct {
    uint8_t partnum;
    uint8_t version;
} cc1101_id_t;

typedef struct {
    uint32_t ts_ms;
    int8_t   rssi_dbm;
    uint8_t  lqi;
    bool     crc_ok;   // CC1101 CRC flag (may not be meaningful for Apollo)
    uint8_t  len;
    uint8_t  data[64];
} cc1101_rx_frame_t;

typedef void (*cc1101_rx_cb_t)(const cc1101_rx_frame_t *frame, void *user_ctx);

esp_err_t cc1101_init(const cc1101_pins_t *pins);
esp_err_t cc1101_read_id(cc1101_id_t *out);

/**
 * Apollo-profile RX (reliability):
 *  - 433.92 MHz
 *  - 2-FSK
 *  - Manchester decoding enabled
 *  - ~1 kbps
 *  - narrow RX BW (~58 kHz)
 *  - carrier/RSSI gated capture
 */
esp_err_t cc1101_start_rx_apollo(cc1101_rx_cb_t cb, void *user_ctx);

esp_err_t cc1101_stop_rx(void);
