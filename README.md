# ESP32-S3 Template

Reusable ESP-IDF template for ESP32-S3 with 16MB flash and 8MB octal PSRAM @ 80MHz.

## Prereqs

Export the ESP-IDF environment:

```
source /path/to/esp-idf/export.sh
# example:
# source /home/galibard/re/esp32s3/esp-idf/export.sh
```

## Quick start (recommended)

From the directory that contains this template:

```
./new_from_template.sh /path/to/my_new_project my_project_name
```

This copies the template, renames the project in `CMakeLists.txt`, updates the banner in `main/main.c`, and opens VS Code if available.

## Manual start

Copy the template:

```
cp -a esp32s3_template my_new_project
```

Rename the project in `CMakeLists.txt`:

```
project(my_project_name)
```

Optional: update the log TAG/banner in `main/main.c`.

## Build and flash

```
cd /path/to/my_new_project
idf.py set-target esp32s3
idf.py build flash monitor
```

Specify the serial port when flashing/monitoring:

```
idf.py -p /dev/ttyUSB0 flash monitor
```

## Verify

In the boot log you should see:

- SPI Flash Size : 16MB
- Found 8MB PSRAM device
- Speed: 80MHz
- template: ESP32-S3 template app started (or your renamed banner)

## Hardware defaults

- Flash size: 16MB
- PSRAM: 8MB, octal mode @ 80MHz

Defaults live in `sdkconfig.defaults`. Update that file (not `sdkconfig`) if you change hardware.
