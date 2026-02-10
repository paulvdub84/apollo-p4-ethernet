#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_http_server.h"

#include "esp_ota_ops.h"
#include "esp_app_format.h"

static const char *TAG = "ota_http";

/* How much we read from the HTTP body at once */
#define OTA_BUF_SZ (1024)

/*
  Minimal OTA endpoint:
    - GET  /        -> simple status page
    - POST /ota     -> raw .bin upload (build/apollo_p4_eth.bin)
*/
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *resp =
        "Apollo P4 OTA server is running.\n"
        "POST firmware to /ota as raw binary.\n";
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

/* Extract esp_app_desc_t from the first chunk, if present */
static bool try_parse_app_desc(const uint8_t *buf, int len, esp_app_desc_t *out)
{
    // Layout: esp_image_header_t + esp_image_segment_header_t + esp_app_desc_t
    const int off = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    if (len < off + (int)sizeof(esp_app_desc_t)) return false;

    memcpy(out, buf + off, sizeof(esp_app_desc_t));
    // Basic sanity: version string should be NUL-terminated somewhere
    out->version[sizeof(out->version) - 1] = 0;
    return true;
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA POST /ota content_len=%d", (int)req->content_len);

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        ESP_LOGE(TAG, "No OTA update partition available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition: %s @ 0x%" PRIx32 " size=0x%" PRIx32,
             update->label, update->address, update->size);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update, req->content_len, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    uint8_t buf[OTA_BUF_SZ];
    int remaining = req->content_len;

    bool have_desc = false;
    esp_app_desc_t new_desc = {0};

    while (remaining > 0) {
        int to_read = remaining > OTA_BUF_SZ ? OTA_BUF_SZ : remaining;
        int r = httpd_req_recv(req, (char *)buf, to_read);

        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            // retry
            continue;
        }
        if (r <= 0) {
            ESP_LOGE(TAG, "httpd_req_recv failed: %d", r);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv failed");
            return ESP_FAIL;
        }

        if (!have_desc) {
            have_desc = try_parse_app_desc(buf, r, &new_desc);
            if (have_desc) {
                ESP_LOGI(TAG, "Incoming firmware: project=%s version=%s",
                         new_desc.project_name, new_desc.version);
            }
        }

        err = esp_ota_write(ota_handle, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        remaining -= r;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK. Rebooting into new firmware...\n");

    ESP_LOGW(TAG, "OTA complete. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
    return ESP_OK;
}

esp_err_t ota_http_start(httpd_handle_t *out_server)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t ota = {
        .uri      = "/ota",
        .method   = HTTP_POST,
        .handler  = ota_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &ota);

    ESP_LOGI(TAG, "OTA HTTP server started on port %d", config.server_port);

    if (out_server) *out_server = server;
    return ESP_OK;
}

void ota_http_stop(httpd_handle_t server)
{
    if (server) httpd_stop(server);
}
