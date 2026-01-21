#include "sgp30_simple.h"

#include "esp_check.h"
#include "esp_rom_sys.h"

#define SGP30_CMD_IAQ_INIT 0x2003
#define SGP30_CMD_MEASURE_IAQ 0x2008
#define SGP30_CMD_SET_ABS_HUM 0x2061
#define SGP30_CMD_DELAY_US 15000
#define SGP30_I2C_SPEED_HZ 100000
#define SGP30_SCL_WAIT_US 10000

// Create the I2C device handle once and reuse it.
static esp_err_t sgp30_add_device(sgp30_t* s) {
    // Guard against a NULL pointer so we don't dereference it.
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // If the device handle already exists, don't create it again.
    if (s->dev != NULL) {
        return ESP_OK;
    }

    // Configure the device address and speed for this sensor.
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s->addr,
        .scl_speed_hz = SGP30_I2C_SPEED_HZ,
        .scl_wait_us = SGP30_SCL_WAIT_US,
    };
    // Register the device on the already-created I2C bus.
    return i2c_master_bus_add_device(s->bus, &cfg, &s->dev);
}

// Send a 16-bit command (big-endian) over I2C.
static esp_err_t sgp30_write_cmd(sgp30_t* s, uint16_t cmd) {
    // Convert 16-bit command into two bytes (MSB first).
    uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    // Ensure the device handle exists before any transfer.
    ESP_RETURN_ON_ERROR(sgp30_add_device(s), "", "");
    // Send the command bytes on the I2C bus.
    return i2c_master_transmit(s->dev, buf, sizeof(buf), 100);
}

// Sensirion CRC-8 for each 16-bit word.
static uint8_t sgp30_crc8(const uint8_t* data, size_t len) {
    // Sensirion CRC-8 starts with 0xFF, polynomial 0x31.
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        // XOR next byte into the CRC.
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            // Shift and apply polynomial when MSB is set.
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Read two 16-bit words plus CRC and validate them.
static esp_err_t sgp30_read_words_crc(sgp30_t* s, uint16_t* word0, uint16_t* word1) {
    // Read 2 words (4 bytes) + 2 CRC bytes.
    uint8_t rx[6] = {0};
    // Ensure the device handle exists before any transfer.
    ESP_RETURN_ON_ERROR(sgp30_add_device(s), "", "");
    // Receive the bytes from the sensor.
    ESP_RETURN_ON_ERROR(i2c_master_receive(s->dev, rx, sizeof(rx), 100), "", "");
    // Validate CRC for each 2-byte word.
    if (sgp30_crc8(rx, 2) != rx[2] || sgp30_crc8(&rx[3], 2) != rx[5]) {
        return ESP_ERR_INVALID_CRC;
    }
    // Combine MSB/LSB into 16-bit values.
    *word0 = (uint16_t)((rx[0] << 8) | rx[1]);
    *word1 = (uint16_t)((rx[3] << 8) | rx[4]);
    return ESP_OK;
}

// Prepare the device for use (handle creation only).
esp_err_t sgp30_init(sgp30_t* s) {
    // For now this only creates the I2C device handle.
    return sgp30_add_device(s);
}

// Start the on-chip baseline algorithm (required before IAQ readings).
esp_err_t sgp30_iaq_init(sgp30_t* s) {
    // The SGP30 expects this command once after power-up.
    return sgp30_write_cmd(s, SGP30_CMD_IAQ_INIT);
}

// Set humidity compensation using absolute humidity in g/m^3.
esp_err_t sgp30_set_absolute_humidity(sgp30_t* s, float abs_humidity_gm3) {
    if (abs_humidity_gm3 < 0.0f) {
        abs_humidity_gm3 = 0.0f;
    }
    if (abs_humidity_gm3 > 256.0f) {
        abs_humidity_gm3 = 256.0f; // Cap to avoid overflow of the 8.8 fixed-point value.
    }

    uint16_t ah_fixed = (uint16_t)(abs_humidity_gm3 * 256.0f + 0.5f);
    uint8_t payload[2] = {(uint8_t)(ah_fixed >> 8), (uint8_t)(ah_fixed & 0xFF)};
    uint8_t crc = sgp30_crc8(payload, sizeof(payload));

    uint8_t buf[5] = {
        (uint8_t)(SGP30_CMD_SET_ABS_HUM >> 8), (uint8_t)(SGP30_CMD_SET_ABS_HUM & 0xFF),
        payload[0], payload[1], crc,
    };

    ESP_RETURN_ON_ERROR(sgp30_add_device(s), "", "");
    return i2c_master_transmit(s->dev, buf, sizeof(buf), 100);
}

// Read eCO2 (ppm) and TVOC (ppb) from the sensor.
esp_err_t sgp30_measure_iaq(sgp30_t* s, uint16_t* eco2_ppm, uint16_t* tvoc_ppb) {
    // All output pointers are required.
    if (eco2_ppm == NULL || tvoc_ppb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Request the latest IAQ values.
    ESP_RETURN_ON_ERROR(sgp30_write_cmd(s, SGP30_CMD_MEASURE_IAQ), "", "");
    // Sensor needs a short processing time before the read.
    esp_rom_delay_us(SGP30_CMD_DELAY_US);

    uint16_t eco2_raw = 0;
    uint16_t tvoc_raw = 0;
    ESP_RETURN_ON_ERROR(sgp30_read_words_crc(s, &eco2_raw, &tvoc_raw), "", "");

    *eco2_ppm = eco2_raw;
    *tvoc_ppb = tvoc_raw;
    return ESP_OK;
}
