# SSD1306 I2C Walkthrough (ESP-IDF)

This document explains how the SSD1306 OLED driver in this project works.
It is meant for someone learning C and ESP-IDF from scratch.

## Big Picture

The SSD1306 is a tiny graphics controller. You talk to it over I2C by sending:

- Command bytes (control byte 0x00, then a command)
- Data bytes (control byte 0x40, then pixel data)

The code in `main/ssd1306_simple.c` builds small I2C packets and sends them to
the display. The OLED stores pixels in "pages": each page is 8 pixels tall,
so a 128x64 screen has 8 pages.

## Files and Roles

- `main/main.c` sets up the I2C bus and calls the SSD1306 functions.
- `main/ssd1306_simple.h` defines the `ssd1306_t` struct and function API.
- `main/ssd1306_simple.c` implements the driver and font drawing.

## Data Layout: Pages and Columns

The display memory is organized like this:

- 8 pages (0..7), each page is 8 pixels tall
- 128 columns (0..127)
- Each byte you send writes 8 vertical pixels at the current column

So when you set the cursor to page 0, column 0 and send 6 bytes, you draw
6 vertical columns of 8 pixels each on the top row.

## The `ssd1306_t` Struct

See `main/ssd1306_simple.h`.

- `bus` is the I2C bus handle created in `main/main.c`
- `dev` is the SSD1306 device handle created once and reused
- `addr` is the I2C address (usually 0x3C or 0x3D)
- `width` and `height` describe the display geometry

## Core Driver Functions

### ssd1306_add_device

Creates the I2C device handle once (the new I2C driver expects this).
The handle is stored in `ssd1306_t.dev` and reused for all transactions.

### i2c_write

Sends a raw buffer to the OLED using the stored device handle.
This is used for command writes.

### cmd

Wraps a single command byte with the control byte 0x00.
The SSD1306 treats the next byte as a command.

### data_bytes

Sends the data control byte (0x40) and the pixel data in a single I2C
transaction. This is important: if you split them into two transactions,
the second one will be misread as commands or garbage, causing noise.

### ssd1306_init

Sends the initialization sequence to configure the display:

- turn display off
- set clock, multiplex, and display offset
- enable charge pump
- set page addressing mode
- set segment and COM remap
- set contrast and timings
- turn display on

### ssd1306_set_cursor

Selects which page (0..7) and column (0..127) the next data bytes will use.
The SSD1306 needs three commands:

- set page address (0xB0 | page)
- set low column address (0x00 | low nibble)
- set high column address (0x10 | high nibble)

### ssd1306_clear

Writes 128 zero bytes to each page to clear the screen.

### ssd1306_write_text_5x7

Draws text using a 5x7 font. Each character is 5 bytes wide, plus a blank
column for spacing. Each character is written as 6 bytes of pixel data.

The font table only includes ASCII 32..90 (space through 'Z'). Any other
character is replaced by '?'.

## Main Program Flow

See `main/main.c`:

1) Configure and create the I2C bus
2) Fill an `ssd1306_t` with bus/address/geometry
3) Initialize the display
4) Clear it
5) Set cursor and write text
6) Suspend the task so the screen stays on

## Common Issues and Fixes

- Random noise: usually caused by splitting the 0x40 data control byte
  and the data payload into separate I2C transactions.
- Wrong address: try 0x3C and 0x3D.
- No display: check SDA/SCL wiring and pullups.

## Ideas for Next Steps

- Add lowercase letters to the font table.
- Add a framebuffer and a `ssd1306_flush` function for full-screen drawing.
- Add `ssd1306_deinit` to remove the device handle when shutting down.
