#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mqtt_connected_cb_t)(void);

esp_err_t mqtt_start(const char *host,
                     int port,
                     const char *username,
                     const char *password);

void mqtt_stop(void);

int mqtt_publish(const char *topic, const char *payload, int qos, bool retain);

bool mqtt_is_connected(void);

void mqtt_set_connected_callback(mqtt_connected_cb_t cb);

#ifdef __cplusplus
}
#endif
