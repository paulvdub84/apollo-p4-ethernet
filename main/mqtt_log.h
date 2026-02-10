#pragma once

#include "esp_err.h"

// Installs an ESP-IDF log hook that mirrors logs to MQTT.
// Logs are published to `topic` as plain text lines.
esp_err_t mqtt_log_init(const char *topic);
