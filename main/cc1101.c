#include "cc1101.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"

/* CC1101 strobes */
#define CC1101_SRES   0x30
#define CC1101_SIDLE  0x36
#define CC1101_SRX    0x34
#define CC1101_SFRX   0x3A
#define CC1101_SCAL   0x33

/* Burst/single read/write */
#define CC1101_READ_SINGLE   0x80
#define CC1101_READ_BURST    0xC0
#define CC1101_WRITE_BURST   0x40

/* Registers (addresses) */
#define REG_IOCFG2     0x00
#define REG_IOCFG0     0x02
#define REG_FIFOTHR    0x03
#define REG_PKTCTRL1   0x04
#define REG_PKTCTRL0   0x05
#define REG_PKTLEN     0x06
#define REG_FSCTRL1    0x0B
#define REG_FREQ2      0x0D
#define REG_FREQ1      0x0E
#define REG_FREQ0      0x0F
#define REG_MDMCFG4    0x10
#define REG_MDMCFG3    0x11
#define REG_MDMCFG2    0x12
#define REG_MDMCFG1    0x13
#define REG_MDMCFG0    0x14
#define REG_DEVIATN    0x15
#define REG_MCSM1      0x17
#define REG_MCSM0      0x18
#define REG_FOCCFG     0x19
#define REG_BSCFG      0x1A
#define REG_AGCCTRL2   0x1B
#define REG_AGCCTRL1   0x1C
#define REG_AGCCTRL0   0x1D
#define REG_FREND1     0x21
#define REG_FREND0     0x22
#define REG_FSCAL3     0x23
#define REG_FSCAL2     0x24
#define REG_FSCAL1     0x25
#define REG_FSCAL0     0x26
#define REG_TEST2      0x2C
#define REG_TEST1      0x2D
#define REG_TEST0      0x2E
#define REG_PARTNUM    0x30
#define REG_VERSION    0x31
#define REG_LQI        0x33
#define REG_RSSI       0x34
#define REG_PKTSTATUS  0x38
#define REG_RXBYTES    0x3B
#define REG_FIFO       0x3F

static const char *TAG = "cc1101";

static spi_device_handle_t s_spi;
static cc1101_rx_cb_t s_cb = NULL;
static void *s_ctx = NULL;
static TaskHandle_t s_rx_task = NULL;

/* Reliability tuning */
#define POLL_MS                   15
#define RSSI_STRONG_DBM           (-90)  // gate threshold (looser; we filter later)
#define MAX_CAPTURE_BYTES         64     // capture window for CRC scanning
#define CAPTURE_WAIT_MS           90     // wait after detect to grab most of burst
#define QUIET_AFTER_CAPTURE_MS    1200   // avoid repeated triggers

static esp_err_t spi_txrx(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_spi, &t);
}

static void strobe(uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx = 0;
    (void)spi_txrx(&tx, &rx, 1);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | CC1101_READ_SINGLE), 0x00 };
    uint8_t rx[2] = {0};
    (void)spi_txrx(tx, rx, 2);
    return rx[1];
}

static void write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    uint8_t rx[2] = {0};
    (void)spi_txrx(tx, rx, 2);
}

static void read_burst(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | CC1101_READ_BURST);
    (void)spi_txrx(tx, rx, len + 1);
    memcpy(buf, &rx[1], len);
}

static int8_t rssi_raw_to_dbm(uint8_t rssi_raw)
{
    // good enough for gating / relative values
    return (int8_t)((int)rssi_raw / 2 - 74);
}

static bool looks_like_repeating_byte(const uint8_t *d, uint8_t len)
{
    if (len < 8) return false;
    uint8_t b = d[0];
    for (uint8_t i = 1; i < len; i++) {
        if (d[i] != b) return false;
    }
    return true;
}

/*
 Apollo profile config:
  - 433.92MHz (assumes 26MHz XTAL on CC1101 module)
  - 2-FSK
  - Manchester decoding enabled
  - ~1.0kbps
  - RX BW ~58kHz
  - deviation ~5.2kHz
  - infinite packet length (we do framing ourselves)
  - no sync requirement (we gate by carrier/RSSI)
*/
static void cc1101_config_apollo(void)
{
    strobe(CC1101_SIDLE);
    strobe(CC1101_SFRX);

    // GDOs: safe defaults (not currently used)
    write_reg(REG_IOCFG2, 0x29);
    write_reg(REG_IOCFG0, 0x2E);

    // FIFO threshold (default-ish)
    write_reg(REG_FIFOTHR, 0x07);

    // Packet handling: infinite length, no HW CRC
    write_reg(REG_PKTCTRL1, 0x00);
    write_reg(REG_PKTCTRL0, 0x02);  // LENGTH_CONFIG=2 (infinite), CRC off
    write_reg(REG_PKTLEN, 0xFF);

    // IF frequency
    write_reg(REG_FSCTRL1, 0x06);

    // 433.92 MHz -> FREQ = 0x10B071 (26MHz XTAL)
    write_reg(REG_FREQ2, 0x10);
    write_reg(REG_FREQ1, 0xB0);
    write_reg(REG_FREQ0, 0x71);

    // MDMCFG4/3:
    //  - RX BW ~58kHz (CHANBW_E=3, CHANBW_M=3)
    //  - Data rate ~1001 bps (DRATE_E=5, DRATE_M=0x43)
    write_reg(REG_MDMCFG4, 0xF5);
    write_reg(REG_MDMCFG3, 0x43);

    // MDMCFG2:
    //  - 2-FSK (MOD_FORMAT=0)
    //  - Manchester enable
    //  - SYNC_MODE=0 (no sync)
    write_reg(REG_MDMCFG2, 0x08);

    // spacing/others not critical in single-channel sniff mode
    write_reg(REG_MDMCFG1, 0x22);
    write_reg(REG_MDMCFG0, 0xF8);

    // Deviation ~5.16kHz
    write_reg(REG_DEVIATN, 0x15);

    // Main state machine: stay in RX after RX, auto-cal
    write_reg(REG_MCSM1, 0x3C);
    write_reg(REG_MCSM0, 0x10); // FS_AUTOCAL=1

    // Typical recommended baseband settings
    write_reg(REG_FOCCFG,  0x16);
    write_reg(REG_BSCFG,   0x6C);
    write_reg(REG_AGCCTRL2,0x03);
    write_reg(REG_AGCCTRL1,0x40);
    write_reg(REG_AGCCTRL0,0x91);
    write_reg(REG_FREND1,  0x56);
    write_reg(REG_FREND0,  0x10);
    write_reg(REG_FSCAL3,  0xE9);
    write_reg(REG_FSCAL2,  0x2A);
    write_reg(REG_FSCAL1,  0x00);
    write_reg(REG_FSCAL0,  0x1F);
    write_reg(REG_TEST2,   0x81);
    write_reg(REG_TEST1,   0x35);
    write_reg(REG_TEST0,   0x09);

    // Calibrate then RX
    strobe(CC1101_SCAL);
    vTaskDelay(pdMS_TO_TICKS(5));
    strobe(CC1101_SRX);
}

static void rx_task_fn(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Apollo RX task running (FSK + Manchester, burst capture)");

    uint32_t quiet_until_ms = 0;

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (quiet_until_ms && (int32_t)(now_ms - quiet_until_ms) < 0) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        uint8_t pktstatus = read_reg(REG_PKTSTATUS);
        bool carrier = (pktstatus & 0x40) != 0; // CS bit
        int8_t rssi_dbm = rssi_raw_to_dbm(read_reg(REG_RSSI));

        // gate: either carrier present OR strong-ish RSSI
        if (!carrier && rssi_dbm < RSSI_STRONG_DBM) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        // Wait long enough to capture most of the burst (~136ms total).
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_WAIT_MS));

        uint8_t rxbytes = read_reg(REG_RXBYTES) & 0x7F;
        if (rxbytes == 0) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        if (rxbytes > MAX_CAPTURE_BYTES) rxbytes = MAX_CAPTURE_BYTES;

        cc1101_rx_frame_t f = {0};
        f.ts_ms = (uint32_t)(esp_timer_get_time() / 1000);
        f.rssi_dbm = rssi_dbm;

        uint8_t lqi = read_reg(REG_LQI);
        f.lqi = lqi & 0x7F;
        f.crc_ok = (lqi & 0x80) != 0; // CC1101 CRC flag (not the Apollo CRC8)

        read_burst(REG_FIFO, f.data, rxbytes);
        f.len = rxbytes;

        // Flush FIFO & back to RX quickly
        strobe(CC1101_SFRX);
        strobe(CC1101_SRX);

        // Drop obvious junk (all bytes equal)
        if (looks_like_repeating_byte(f.data, f.len)) {
            ESP_LOGD(TAG, "Dropped repeating-byte capture (len=%u rssi=%d)", f.len, (int)f.rssi_dbm);
            quiet_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + QUIET_AFTER_CAPTURE_MS;
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        ESP_LOGI(TAG, "CAPTURE len=%u rssi=%d lqi=%u carrier=%d",
                 f.len, (int)f.rssi_dbm, (unsigned)f.lqi, carrier ? 1 : 0);

        if (s_cb) s_cb(&f, s_ctx);

        // quiet period to avoid repeated triggers on same burst / local interference
        quiet_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + QUIET_AFTER_CAPTURE_MS;

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

esp_err_t cc1101_init(const cc1101_pins_t *pins)
{
    if (!pins) return ESP_ERR_INVALID_ARG;

    spi_bus_config_t buscfg = {
        .mosi_io_num = pins->pin_mosi,
        .miso_io_num = pins->pin_miso,
        .sclk_io_num = pins->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = pins->pin_cs,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    strobe(CC1101_SRES);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "CC1101 init OK");
    return ESP_OK;
}

esp_err_t cc1101_read_id(cc1101_id_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    out->partnum = read_reg(REG_PARTNUM);
    out->version = read_reg(REG_VERSION);
    return ESP_OK;
}

esp_err_t cc1101_start_rx_apollo(cc1101_rx_cb_t cb, void *user_ctx)
{
    s_cb = cb;
    s_ctx = user_ctx;

    cc1101_config_apollo();

    if (!s_rx_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            rx_task_fn,
            "cc1101_rx",
            4096,
            NULL,
            5,
            &s_rx_task,
            0
        );
        if (ok != pdPASS) return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "RX started (Apollo profile: 433.92 FSK + Manchester)");
    return ESP_OK;
}

esp_err_t cc1101_stop_rx(void)
{
    if (s_rx_task) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    strobe(CC1101_SIDLE);
    return ESP_OK;
}
