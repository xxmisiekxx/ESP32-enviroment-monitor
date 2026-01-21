# Climate Logger (ESP32-S3)

ESP-IDF project for an ESP32-S3 that reads multiple environmental sensors on a shared I2C bus and displays the values on an SSD1306 OLED.

## Features

- SHT45 temperature and humidity.
- SCD41 CO2 + temperature + humidity.
- SGP30 eCO2 + TVOC with humidity compensation from SHT45.
- SSD1306 128x64 OLED display with a simple text layout.

## Hardware

- Target: ESP32-S3 (tested on ESP32-S3, ESP-IDF).
- I2C pins:
  - SDA: GPIO41
  - SCL: GPIO42
- I2C devices:
  - SSD1306 OLED @ 0x3C
  - SHT45 @ 0x44
  - SCD41 @ 0x62
  - SGP30 @ 0x58

Note: External I2C pull-ups are recommended for reliable operation.

## Display Layout

```
SHT45 TEMP: xx.xC
SHT45 HUM : xx.x%
---------------------
SCD41 CO2 : xxxxppm
SCD41 TEMP: xx.xC
SCD41 HUM : xx.x%
---------------------
eCO2: xxxx TVOC: xxxx
```

## Wiring (I2C)

All sensors share the same I2C bus. Power at 3.3V (unless your breakout specifies otherwise).

```
ESP32-S3            OLED / SHT45 / SCD41 / SGP30
---------           ------------------------------
3V3  ----------->   VCC
GND  ----------->   GND
GPIO41 (SDA) --->   SDA
GPIO42 (SCL) --->   SCL
```

If your breakouts do not include pull-ups, add 4.7k-10k resistors from SDA/SCL to 3.3V.

## Build and Flash

```
idf.py set-target esp32s3
idf.py build flash monitor
```

Specify the serial port if needed:

```
idf.py -p /dev/ttyUSB0 flash monitor
```

## Notes

- SCD41 is a real NDIR CO2 sensor; SGP30 reports eCO2 based on VOCs and will not match SCD41.
- SGP30 needs a long warm-up/baseline period for stable readings.
- Humidity compensation for SGP30 uses absolute humidity derived from SHT45 readings.
