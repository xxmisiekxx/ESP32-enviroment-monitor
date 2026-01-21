#include "scd41_simple.h"

#include "esp_check.h"
#include "esp_rom_sys.h"

#define SCD41_CMD_START_PERIODIC 0x21B1
#define SCD41_CMD_STOP_PERIODIC 0x3F86
#define SCD41_CMD_DATA_READY 0xE4B8
#define SCD41_CMD_READ_MEASUREMENT 0xEC05
#define SCD41_CMD_WAKE_UP 0x36F6
#define SCD41_CMD_DELAY_US 5000
#define SCD41_I2C_SPEED_HZ 100000
#define SCD41_SCL_WAIT_US 10000

// Create the I2C device handle once and reuse it.
static esp_err_t scd41_add_device(scd41_t* s) {
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
        .scl_speed_hz = SCD41_I2C_SPEED_HZ,
        .scl_wait_us = SCD41_SCL_WAIT_US,
    };
    // Register the device on the already-created I2C bus.
    return i2c_master_bus_add_device(s->bus, &cfg, &s->dev);
}

// Send a 16-bit command (big-endian) over I2C.
static esp_err_t scd41_write_cmd(scd41_t* s, uint16_t cmd) {
    // Convert 16-bit command into two bytes (MSB first).
    uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    // Ensure the device handle exists before any transfer.
    ESP_RETURN_ON_ERROR(scd41_add_device(s), "", "");
    // Send the command bytes on the I2C bus.
    return i2c_master_transmit(s->dev, buf, sizeof(buf), 100);
}

// Some commands (like WAKE-UP) may not ACK; ignore that error.
static esp_err_t scd41_write_cmd_ignore_nack(scd41_t* s, uint16_t cmd) {
    esp_err_t ret = scd41_write_cmd(s, cmd);
    if (ret == ESP_ERR_INVALID_RESPONSE) {
        return ESP_OK;
    }
    return ret;
}

// Sensirion CRC-8 for each 16-bit word.
static uint8_t scd41_crc8(const uint8_t* data, size_t len) {
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

// Read one 16-bit word plus CRC and validate it.
static esp_err_t scd41_read_word_crc(scd41_t* s, uint16_t* value) {
    // Read 2 data bytes + 1 CRC byte.
    uint8_t rx[3] = {0};
    // Ensure the device handle exists before any transfer.
    ESP_RETURN_ON_ERROR(scd41_add_device(s), "", "");
    // Receive the bytes from the sensor.
    ESP_RETURN_ON_ERROR(i2c_master_receive(s->dev, rx, sizeof(rx), 100), "", "");
    // Verify CRC matches the two data bytes.
    if (scd41_crc8(rx, 2) != rx[2]) {
        return ESP_ERR_INVALID_CRC;
    }
    // Combine MSB/LSB into a 16-bit value.
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return ESP_OK;
}

// Prepare the device for use (handle creation only).
esp_err_t scd41_init(scd41_t* s) {
    // For now this only creates the I2C device handle.
    return scd41_add_device(s);
}

// Wake up the sensor from sleep/idle (no ACK expected by the datasheet).
esp_err_t scd41_wakeup(scd41_t* s) {
    return scd41_write_cmd_ignore_nack(s, SCD41_CMD_WAKE_UP);
}

// Start continuous measurement (new data about every 5 seconds).
esp_err_t scd41_start_periodic_measurement(scd41_t* s) {
    // Tells the sensor to start continuous measurements.
    return scd41_write_cmd(s, SCD41_CMD_START_PERIODIC);
}

// Stop the periodic measurement mode.
esp_err_t scd41_stop_periodic_measurement(scd41_t* s) {
    // Stops continuous measurements to save power or reconfigure.
    return scd41_write_cmd_ignore_nack(s, SCD41_CMD_STOP_PERIODIC);
}

// Query if a fresh measurement is ready (status low 11 bits).
esp_err_t scd41_data_ready(scd41_t* s, bool* ready) {
    // Caller must pass a valid output pointer.
    if (ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // Request the data-ready status word.
    ESP_RETURN_ON_ERROR(scd41_write_cmd(s, SCD41_CMD_DATA_READY), "", "");
    // Sensor needs a short processing time before the read.
    esp_rom_delay_us(SCD41_CMD_DELAY_US);
    uint16_t status = 0;
    // Read the status word and its CRC.
    ESP_RETURN_ON_ERROR(scd41_read_word_crc(s, &status), "", "");
    // For SCD4x, any non-zero value in the lower 11 bits means data ready.
    *ready = (status & 0x07FFu) != 0;
    return ESP_OK;
}

// Read CO2 (ppm), temperature (C), and humidity (%) from the sensor.
esp_err_t scd41_read_measurement(scd41_t* s, uint16_t* co2_ppm, float* temp_c, float* rh) {
    // All output pointers are required.
    if (co2_ppm == NULL || temp_c == NULL || rh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Request the latest measurement values.
    ESP_RETURN_ON_ERROR(scd41_write_cmd(s, SCD41_CMD_READ_MEASUREMENT), "", "");
    // Sensor needs a short processing time before the read.
    esp_rom_delay_us(SCD41_CMD_DELAY_US);

    // Read 3 words (CO2, temperature, humidity), each with a CRC byte.
    uint8_t rx[9] = {0};
    // Ensure the device handle exists before any transfer.
    ESP_RETURN_ON_ERROR(scd41_add_device(s), "", "");
    // Receive the 9 bytes from the sensor.
    ESP_RETURN_ON_ERROR(i2c_master_receive(s->dev, rx, sizeof(rx), 100), "", "");

    // Validate CRC for each 2-byte word.
    if (scd41_crc8(rx, 2) != rx[2] || scd41_crc8(&rx[3], 2) != rx[5] ||
        scd41_crc8(&rx[6], 2) != rx[8]) {
        return ESP_ERR_INVALID_CRC;
    }

    // Parse raw words from MSB/LSB pairs.
    uint16_t co2_raw = (uint16_t)((rx[0] << 8) | rx[1]);
    uint16_t t_raw = (uint16_t)((rx[3] << 8) | rx[4]);
    uint16_t rh_raw = (uint16_t)((rx[6] << 8) | rx[7]);

    // CO2 is already in ppm according to the datasheet.
    *co2_ppm = co2_raw;
    // Convert raw temperature to degrees C (datasheet formula).
    *temp_c = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    // Convert raw humidity to %RH (datasheet formula).
    *rh = 100.0f * ((float)rh_raw / 65535.0f);
    return ESP_OK;
}
