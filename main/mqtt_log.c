#include "mqtt_log.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "esp_log.h"
#include "mqtt.h"

static const char *TAG = "mqtt_log";

#define LOG_LINE_MAX   256
#define LOG_QUEUE_LEN  40

typedef struct {
    char line[LOG_LINE_MAX];
} log_item_t;

static QueueHandle_t s_q = NULL;
static char s_topic[128] = {0};

// original vprintf used by esp_log
static vprintf_like_t s_orig_vprintf = NULL;

// recursion guard (avoid logging our own mqtt_publish calls)
static volatile bool s_suppress = false;

static void log_task(void *arg)
{
    (void)arg;

    log_item_t item;

    while (1) {
        if (xQueueReceive(s_q, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Only publish when MQTT connected. If disconnected, drop lines (queue is bounded).
        if (!mqtt_is_connected()) {
            continue;
        }

        // Prevent recursion if mqtt_publish internally logs
        s_suppress = true;
        mqtt_publish(s_topic, item.line, 0, false);
        s_suppress = false;

        // tiny yield so we never hog CPU in bursty log periods
        vTaskDelay(1);
    }
}

static int mqtt_log_vprintf(const char *fmt, va_list args)
{
    // Always keep normal UART output
    int written = 0;
    if (s_orig_vprintf) {
        va_list args_copy;
        va_copy(args_copy, args);
        written = s_orig_vprintf(fmt, args_copy);
        va_end(args_copy);
    }

    // If not initialised or suppressed, do nothing else
    if (!s_q || s_suppress) {
        return written;
    }

    // Format into a fixed buffer
    log_item_t item;
    item.line[0] = '\0';

    va_list args2;
    va_copy(args2, args);
    vsnprintf(item.line, sizeof(item.line), fmt, args2);
    va_end(args2);

    // Enqueue (ISR-safe)
    if (xPortInIsrContext()) {
        BaseType_t hp = pdFALSE;
        (void)xQueueSendFromISR(s_q, &item, &hp);
        if (hp) portYIELD_FROM_ISR();
    } else {
        (void)xQueueSend(s_q, &item, 0); // non-blocking
    }

    return written;
}

esp_err_t mqtt_log_init(const char *topic)
{
    if (!topic || topic[0] == '\0') return ESP_ERR_INVALID_ARG;

    strncpy(s_topic, topic, sizeof(s_topic) - 1);
    s_topic[sizeof(s_topic) - 1] = '\0';

    if (!s_q) {
        s_q = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_item_t));
        if (!s_q) return ESP_ERR_NO_MEM;
    }

    // Install log hook once
    if (!s_orig_vprintf) {
        s_orig_vprintf = esp_log_set_vprintf(mqtt_log_vprintf);
    }

    // Start publisher task once
    static bool started = false;
    if (!started) {
        started = true;
        if (xTaskCreate(log_task, "mqtt_log", 4096, NULL, 8, NULL) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "MQTT logging enabled -> topic: %s", s_topic);
    return ESP_OK;
}
