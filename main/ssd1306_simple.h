#pragma once
#include <stdint.h>
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev; // Created once in ssd1306_init and reused for all transfers.
    uint8_t addr;   //0x3C
    uint8_t width;  //128
    uint8_t height; //64

} ssd1306_t;

esp_err_t ssd1306_init(ssd1306_t* d);
esp_err_t ssd1306_clear(ssd1306_t* d);
esp_err_t ssd1306_set_cursor(ssd1306_t* d, uint8_t page, uint8_t col);
esp_err_t ssd1306_write_text_5x7(ssd1306_t* d, const char* text);
