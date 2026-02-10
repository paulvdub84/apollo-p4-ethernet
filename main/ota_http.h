#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t ota_http_start(httpd_handle_t *out_server);
void ota_http_stop(httpd_handle_t server);
