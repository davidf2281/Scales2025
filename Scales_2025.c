/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

/* Example code to talk to a Max7219 driving an 32x8 pixel LED display via SPI

   NOTE: The device is driven at 5v, but SPI communications are at 3v3

   Connections on Raspberry Pi Pico board and a generic Max7219 board, other
   boards may vary.

   * GPIO 17 (pin 22) Chip select -> CS on Max7219 board
   * GPIO 18 (pin 24) SCK/spi0_sclk -> CLK on Max7219 board
   * GPIO 19 (pin 25) MOSI/spi0_tx -> DIN on Max7219 board
   * 5v (pin 40) -> VCC on Max7219 board
   * GND (pin 38)  -> GND on Max7219 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

*/

// This defines how many Max7219 modules we have cascaded together, in this case, we have 4 x 8x8 matrices giving a total of 32x8
#define NUM_MODULES 4

const uint8_t CMD_NOOP = 0;
const uint8_t CMD_SCANLINE_0 = 1;
const uint8_t CMD_SCANLINE_1 = 2;
const uint8_t CMD_SCANLINE_2 = 3;
const uint8_t CMD_SCANLINE_3 = 4;
const uint8_t CMD_SCANLINE_4 = 5;
const uint8_t CMD_SCANLINE_5 = 6;
const uint8_t CMD_SCANLINE_6 = 7;
const uint8_t CMD_SCANLINE_7 = 8;
const uint8_t CMD_DECODEMODE = 9;
const uint8_t CMD_BRIGHTNESS = 10;
const uint8_t CMD_SCANLIMIT = 11;
const uint8_t CMD_SHUTDOWN = 12;
const uint8_t CMD_DISPLAYTEST = 15;

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 100
#endif

int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define
    // PICO_DEFAULT_LED_PIN so we can use normal GPIO functionality to turn the
    // led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

void LEDBlink(int repeats) {
    for (int i = 0; i < repeats; i++) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
    sleep_ms(1);
}

static void write_register_all(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    cs_select();
    for (int i = 0; i < NUM_MODULES; i++) {
        spi_write_blocking(spi_default, buf, 2);
    }
    cs_deselect();
}

static void write_digit_all(uint8_t *buffer) {
    uint8_t buf[2];
    for (int i = 0; i < NUM_MODULES; i++) {
        for (int j = 0; j < 8; j++) {
            cs_select();
            buf[0] = CMD_SCANLINE_0 + j;
            buf[1] = buffer[j];
            spi_write_blocking(spi_default, buf, 2);
            cs_deselect();
        }
    }
}

static uint8_t mirror(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void digit_0(uint8_t *buffer) {
    buffer[0] = mirror(0b01110000);
    buffer[1] = mirror(0b10001000);
    buffer[2] = mirror(0b10001000);
    buffer[3] = mirror(0b10001000);
    buffer[4] = mirror(0b10001000);
    buffer[5] = mirror(0b10001000);
    buffer[6] = mirror(0b10001000);
    buffer[7] = mirror(0b01110000);
}

// static void digit_0(uint8_t *buffer) {
//     buffer[0] = mirror(0b01111000);
//     buffer[1] = 0;
//     buffer[2] = 0;
//     buffer[3] = 0;
//     buffer[4] = 0;
//     buffer[5] = 0;
//     buffer[6] = 0;
//     buffer[7] = 0;
// }

void initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
}

int main() {
    initialize();

    LEDBlink(2);

    // This example will use SPI0 at 1MHz.
    spi_init(spi_default, 1 * 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    // Send init sequence to device
    write_register_all(CMD_SHUTDOWN, 0);
    write_register_all(CMD_DISPLAYTEST, 0);
    write_register_all(CMD_SCANLIMIT, 7);   // Use all lines
    write_register_all(CMD_DECODEMODE, 0);  // No BCD decode, just use bit pattern.
    write_register_all(CMD_BRIGHTNESS, 4);
    write_register_all(CMD_SHUTDOWN, 1);

    bool on = true;

    uint8_t digitBuffer[8];
    digit_0(digitBuffer);

    while (true) {
        if (on) {
            write_register_all(CMD_SCANLINE_0, mirror(0b00000001));
            write_register_all(CMD_SCANLINE_1, 0);
            write_register_all(CMD_SCANLINE_2, 0);
            write_register_all(CMD_SCANLINE_3, 0);
            write_register_all(CMD_SCANLINE_4, 0);
            write_register_all(CMD_SCANLINE_5, 0);
            write_register_all(CMD_SCANLINE_6, 0);
            write_register_all(CMD_SCANLINE_7, 0);
        } else {
            // write_register_all(CMD_SCANLINE_0, digitBuffer[0]);
            // write_register_all(CMD_SCANLINE_1, 0);
            // write_register_all(CMD_SCANLINE_2, 0);
            // write_register_all(CMD_SCANLINE_3, 0);
            // write_register_all(CMD_SCANLINE_4, 0);
            // write_register_all(CMD_SCANLINE_5, 0);
            // write_register_all(CMD_SCANLINE_6, 0);
            // write_register_all(CMD_SCANLINE_7, 0);
            write_digit_all(digitBuffer);
        }

        sleep_ms(500);

        on = !on;
    }
}
