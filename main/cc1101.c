#include "cc1101.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"

static const char *TAG = "cc1101";

/* ======================= CC1101 SPI / Registers ======================= */

#define CC1101_WRITE_BURST   0x40
#define CC1101_READ_SINGLE   0x80
#define CC1101_READ_BURST    0xC0

/* Command strobes */
#define CC1101_SRES   0x30
#define CC1101_SFSTXON 0x31
#define CC1101_SXOFF  0x32
#define CC1101_SCAL   0x33
#define CC1101_SRX    0x34
#define CC1101_STX    0x35
#define CC1101_SIDLE  0x36
#define CC1101_SWOR   0x38
#define CC1101_SPWD   0x39
#define CC1101_SFRX   0x3A
#define CC1101_SFTX   0x3B
#define CC1101_SWORRST 0x3C
#define CC1101_SNOP   0x3D

/* Config registers */
#define CC1101_IOCFG2     0x00
#define CC1101_IOCFG1     0x01
#define CC1101_IOCFG0     0x02
#define CC1101_FIFOTHR    0x03
#define CC1101_SYNC1      0x04
#define CC1101_SYNC0      0x05
#define CC1101_PKTLEN     0x06
#define CC1101_PKTCTRL1   0x07
#define CC1101_PKTCTRL0   0x08
#define CC1101_ADDR       0x09
#define CC1101_CHANNR     0x0A
#define CC1101_FSCTRL1    0x0B
#define CC1101_FSCTRL0    0x0C
#define CC1101_FREQ2      0x0D
#define CC1101_FREQ1      0x0E
#define CC1101_FREQ0      0x0F
#define CC1101_MDMCFG4    0x10
#define CC1101_MDMCFG3    0x11
#define CC1101_MDMCFG2    0x12
#define CC1101_MDMCFG1    0x13
#define CC1101_MDMCFG0    0x14
#define CC1101_DEVIATN    0x15
#define CC1101_MCSM2      0x16
#define CC1101_MCSM1      0x17
#define CC1101_MCSM0      0x18
#define CC1101_FOCCFG     0x19
#define CC1101_BSCFG      0x1A
#define CC1101_AGCCTRL2   0x1B
#define CC1101_AGCCTRL1   0x1C
#define CC1101_AGCCTRL0   0x1D
#define CC1101_WOREVT1    0x1E
#define CC1101_WOREVT0    0x1F
#define CC1101_WORCTRL    0x20
#define CC1101_FREND1     0x21
#define CC1101_FREND0     0x22
#define CC1101_FSCAL3     0x23
#define CC1101_FSCAL2     0x24
#define CC1101_FSCAL1     0x25
#define CC1101_FSCAL0     0x26
#define CC1101_RCCTRL1    0x27
#define CC1101_RCCTRL0    0x28
#define CC1101_FSTEST     0x29
#define CC1101_PTEST      0x2A
#define CC1101_AGCTEST    0x2B
#define CC1101_TEST2      0x2C
#define CC1101_TEST1      0x2D
#define CC1101_TEST0      0x2E

/* Status registers */
#define CC1101_PARTNUM    0x30
#define CC1101_VERSION    0x31
#define CC1101_RXBYTES    0x3B

/* FIFO */
#define CC1101_FIFO       0x3F

/* ======================= Driver state ======================= */

typedef struct {
    bool inited;
    cc1101_pins_t pins;
    spi_device_handle_t spi;
    SemaphoreHandle_t spi_lock;

    TaskHandle_t rx_task;
    cc1101_rx_cb_t cb;
    void *cb_ctx;

    volatile bool rx_running;
} cc1101_ctx_t;

static cc1101_ctx_t s;

/* ======================= Low-level SPI helpers ======================= */

static esp_err_t spi_txrx(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err;
    xSemaphoreTake(s.spi_lock, portMAX_DELAY);
    err = spi_device_transmit(s.spi, &t);
    xSemaphoreGive(s.spi_lock);
    return err;
}

static esp_err_t cc1101_strobe(uint8_t cmd)
{
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = { 0 };
    return spi_txrx(tx, rx, 1);
}

static esp_err_t cc1101_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { addr, val };
    uint8_t rx[2] = { 0 };
    return spi_txrx(tx, rx, 2);
}

static esp_err_t cc1101_read_reg_status(uint8_t addr, uint8_t *out)
{
    uint8_t tx[2] = { (uint8_t)(addr | CC1101_READ_SINGLE), 0x00 };
    uint8_t rx[2] = { 0 };
    esp_err_t err = spi_txrx(tx, rx, 2);
    if (err == ESP_OK) *out = rx[1];
    return err;
}

static esp_err_t cc1101_read_burst(uint8_t addr, uint8_t *out, size_t len)
{
    if (len == 0) return ESP_OK;

    uint8_t hdr = addr | CC1101_READ_BURST;

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;

    uint8_t *tx = heap_caps_malloc(1 + len, MALLOC_CAP_DEFAULT);
    uint8_t *rx = heap_caps_malloc(1 + len, MALLOC_CAP_DEFAULT);
    if (!tx || !rx) {
        if (tx) heap_caps_free(tx);
        if (rx) heap_caps_free(rx);
        return ESP_ERR_NO_MEM;
    }

    tx[0] = hdr;
    memset(&tx[1], 0, len);
    memset(rx, 0, 1 + len);

    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err;
    xSemaphoreTake(s.spi_lock, portMAX_DELAY);
    err = spi_device_transmit(s.spi, &t);
    xSemaphoreGive(s.spi_lock);

    if (err == ESP_OK) memcpy(out, &rx[1], len);

    heap_caps_free(tx);
    heap_caps_free(rx);
    return err;
}

/* ======================= RSSI helper ======================= */

static int8_t cc1101_rssi_to_dbm(uint8_t rssi_raw)
{
    int16_t rssi_dec = (rssi_raw >= 128) ? ((int16_t)rssi_raw - 256) : rssi_raw;
    int16_t dbm = (rssi_dec / 2) - 74;
    if (dbm < -128) dbm = -128;
    if (dbm > 127) dbm = 127;
    return (int8_t)dbm;
}

/* ======================= IRQ + RX task ======================= */

static void IRAM_ATTR gdo0_isr(void *arg)
{
    (void)arg;
    BaseType_t hp = pdFALSE;
    if (s.rx_task) vTaskNotifyGiveFromISR(s.rx_task, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static esp_err_t cc1101_flush_rx(void)
{
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SIDLE), TAG, "SIDLE");
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SFRX), TAG, "SFRX");
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SRX), TAG, "SRX");
    return ESP_OK;
}

static void rx_task_fn(void *arg)
{
    (void)arg;

    while (s.rx_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!s.rx_running) break;

        // Read RXBYTES
        uint8_t rxbytes = 0;
        if (cc1101_read_reg_status(CC1101_RXBYTES, &rxbytes) != ESP_OK) {
            continue;
        }
        rxbytes &= 0x7F;
        if (rxbytes == 0) continue;

        uint8_t buf[64 + 2] = {0};
        size_t total = 0;

        for (int loops = 0; loops < 8; loops++) {
            if (cc1101_read_reg_status(CC1101_RXBYTES, &rxbytes) != ESP_OK) break;
            rxbytes &= 0x7F;
            if (rxbytes == 0) break;

            size_t to_read = rxbytes;
            if (to_read > sizeof(buf) - total) to_read = sizeof(buf) - total;
            if (to_read == 0) break;

            if (cc1101_read_burst(CC1101_FIFO, &buf[total], to_read) != ESP_OK) break;
            total += to_read;

            if (total >= sizeof(buf)) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (total < 3) continue;

        uint8_t rssi_raw = buf[total - 2];
        uint8_t lqi_crc  = buf[total - 1];

        cc1101_rx_frame_t frame = {0};
        frame.ts_ms = (uint32_t)esp_log_timestamp();
        frame.rssi_dbm = cc1101_rssi_to_dbm(rssi_raw);
        frame.lqi = (lqi_crc & 0x7F);
        frame.crc_ok = (lqi_crc & 0x80) ? true : false;

        size_t payload_len = total - 2;
        if (payload_len > 64) payload_len = 64;
        frame.len = (uint8_t)payload_len;
        memcpy(frame.data, buf, payload_len);

        if (s.cb) s.cb(&frame, s.cb_ctx);

        if (rxbytes & 0x80) {
            ESP_LOGW(TAG, "RX overflow, flushing");
            cc1101_flush_rx();
        }
    }

    vTaskDelete(NULL);
}

/* ======================= Apollo profile config ======================= */

static esp_err_t cc1101_config_apollo_profile(void)
{
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SIDLE), TAG, "SIDLE");
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SFRX), TAG, "SFRX");
    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SFTX), TAG, "SFTX");

    // GDO0 = RX FIFO threshold
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_IOCFG0, 0x00), TAG, "IOCFG0");
    // FIFO threshold ~16
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FIFOTHR, 0x03), TAG, "FIFOTHR");

    // APPEND_STATUS=1
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_PKTCTRL1, 0x04), TAG, "PKTCTRL1");

    // Infinite length
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_PKTCTRL0, 0x02), TAG, "PKTCTRL0");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_PKTLEN, 0xFF), TAG, "PKTLEN");

    // 433.92 MHz (26 MHz crystal)
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FREQ2, 0x10), TAG, "FREQ2");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FREQ1, 0xB0), TAG, "FREQ1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FREQ0, 0x71), TAG, "FREQ0");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCTRL1, 0x06), TAG, "FSCTRL1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCTRL0, 0x00), TAG, "FSCTRL0");

    // ~1 kbps, RXBW ~58 kHz
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MDMCFG4, 0xF5), TAG, "MDMCFG4");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MDMCFG3, 0x3E), TAG, "MDMCFG3");

    // Manchester enabled, no sync
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MDMCFG2, 0x08), TAG, "MDMCFG2");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MDMCFG1, 0x22), TAG, "MDMCFG1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MDMCFG0, 0xF8), TAG, "MDMCFG0");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_DEVIATN, 0x15), TAG, "DEVIATN");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MCSM2, 0x07), TAG, "MCSM2");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MCSM1, 0x3C), TAG, "MCSM1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_MCSM0, 0x18), TAG, "MCSM0");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_AGCCTRL2, 0x43), TAG, "AGCCTRL2");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_AGCCTRL1, 0x40), TAG, "AGCCTRL1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_AGCCTRL0, 0x91), TAG, "AGCCTRL0");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCAL3, 0xE9), TAG, "FSCAL3");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCAL2, 0x2A), TAG, "FSCAL2");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCAL1, 0x00), TAG, "FSCAL1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_FSCAL0, 0x1F), TAG, "FSCAL0");

    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_TEST2, 0x81), TAG, "TEST2");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_TEST1, 0x35), TAG, "TEST1");
    ESP_RETURN_ON_ERROR(cc1101_write_reg(CC1101_TEST0, 0x09), TAG, "TEST0");

    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SRX), TAG, "SRX");
    return ESP_OK;
}

/* ======================= Public API ======================= */

esp_err_t cc1101_init(const cc1101_pins_t *pins)
{
    ESP_RETURN_ON_FALSE(pins != NULL, ESP_ERR_INVALID_ARG, TAG, "pins null");

    memset(&s, 0, sizeof(s));
    s.pins = *pins;
    s.spi_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s.spi_lock != NULL, ESP_ERR_NO_MEM, TAG, "spi_lock");

    spi_bus_config_t buscfg = {
        .mosi_io_num = s.pins.pin_mosi,
        .miso_io_num = s.pins.pin_miso,
        .sclk_io_num = s.pins.pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = s.pins.pin_cs,
        .queue_size = 1,
        // IMPORTANT: do NOT set SPI_DEVICE_HALFDUPLEX (we need full-duplex for status reads)
        .flags = 0,
    };

    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &s.spi), TAG, "spi_bus_add_device");

    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << s.pins.pin_gdo0);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_POSEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config gdo0");

    ESP_RETURN_ON_ERROR(cc1101_strobe(CC1101_SRES), TAG, "SRES");
    vTaskDelay(pdMS_TO_TICKS(5));

    s.inited = true;
    ESP_LOGI(TAG, "CC1101 init OK");
    return ESP_OK;
}

esp_err_t cc1101_read_id(cc1101_id_t *out)
{
    ESP_RETURN_ON_FALSE(s.inited, ESP_ERR_INVALID_STATE, TAG, "not inited");
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out null");

    uint8_t part = 0, ver = 0;
    ESP_RETURN_ON_ERROR(cc1101_read_reg_status(CC1101_PARTNUM, &part), TAG, "read PARTNUM");
    ESP_RETURN_ON_ERROR(cc1101_read_reg_status(CC1101_VERSION, &ver), TAG, "read VERSION");

    out->partnum = part;
    out->version = ver;
    return ESP_OK;
}

esp_err_t cc1101_start_rx_apollo(cc1101_rx_cb_t cb, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(s.inited, ESP_ERR_INVALID_STATE, TAG, "not inited");

    s.cb = cb;
    s.cb_ctx = user_ctx;

    ESP_RETURN_ON_ERROR(cc1101_config_apollo_profile(), TAG, "config apollo");

    if (!s.rx_task) {
        s.rx_running = true;
        BaseType_t ok = xTaskCreatePinnedToCore(rx_task_fn, "cc1101_rx", 4096, NULL, 10, &s.rx_task, 0);
        ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "xTaskCreatePinnedToCore");
    } else {
        s.rx_running = true;
    }

    esp_err_t isr = gpio_install_isr_service(0);
    if (isr != ESP_OK && isr != ESP_ERR_INVALID_STATE) {
        return isr;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s.pins.pin_gdo0, gdo0_isr, NULL), TAG, "gpio_isr_handler_add");

    ESP_LOGI(TAG, "RX started (433.92MHz baseline)");
    return ESP_OK;
}

esp_err_t cc1101_stop_rx(void)
{
    if (!s.inited) return ESP_OK;

    s.rx_running = false;

    if (s.pins.pin_gdo0 >= 0) {
        gpio_isr_handler_remove(s.pins.pin_gdo0);
        gpio_set_intr_type(s.pins.pin_gdo0, GPIO_INTR_DISABLE);
    }

    if (s.rx_task) {
        xTaskNotifyGive(s.rx_task);
        s.rx_task = NULL;
    }

    cc1101_strobe(CC1101_SIDLE);
    cc1101_strobe(CC1101_SFRX);

    ESP_LOGI(TAG, "RX stopped");
    return ESP_OK;
}
