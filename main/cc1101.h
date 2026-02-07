#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int pin_cs;     // CSN
    int pin_sck;    // SCK
    int pin_mosi;   // MOSI
    int pin_miso;   // MISO (SO)
    int pin_gdo0;   // GDO0 (optional, -1 if unused)
    int pin_gdo2;   // GDO2 (optional, -1 if unused)
} cc1101_pins_t;

typedef struct {
    uint8_t partnum;
    uint8_t version;
} cc1101_id_t;

esp_err_t cc1101_init(const cc1101_pins_t *pins);
esp_err_t cc1101_read_id(cc1101_id_t *out);
