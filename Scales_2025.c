/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "characters.c"
#include "hardware/i2c.h"
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

void initI2C() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    initI2C();
}

void initDigitCharacterBuffers(uint8_t *digitCharacterBuffer) {
    for (int i = 0; i < 10; i++) {
        fillDigitBuffer(i, digitCharacterBuffer + (i * 8));
    }
}

void initNonDigitCharacterBuffers(char *nonDigitCharacterBuffer) {
    for (uint i = 0; i < 58; i++) {
        fillCharBuffer((char)(i + 65), nonDigitCharacterBuffer + (i * 8));
    }
}

int main() {
    initialize();

    LEDBlink(2);

    // uint32_t displayFrameBuffer[8];
    // uint8_t digitCharacterBuffer[80];      // Buffer for dot-matrix encodings of the digits 0-9 (eight bytes per character)
    // char nonDigitCharacterBuffer[58 * 8];  // Buffer for dot-matrix encodings of characters A-Z & a-z (eight bytes per character)
    // initDigitCharacterBuffers(digitCharacterBuffer);
    // initNonDigitCharacterBuffers(nonDigitCharacterBuffer);
    // uint8_t *ptr = digitCharacterBuffer;

    int deviceAddress = 0x70;

    uint8_t deviceDisplayAddressPointer = 0b00000000;
    uint8_t deviceClockEnable =   0b00100001;
    uint8_t deviceRowIntSet =     0b10100000;
    uint8_t deviceDimmingSet =    0b11101111; // Full brightness
    uint8_t deviceDisplayOnSet =  0b10000001; // Device ON, blinking OFF.
    // uint8_t deviceDisplayOffSet = 0b10000000; // Device OFF, blinking OFF.

    uint8_t digit0 = 0b10000000;
    uint8_t digit1 = 0b00000000;
    uint8_t digit2 = 0b00000000;
    uint8_t digit3 = 0b00000000;
    uint8_t digit4 = 0b00000000;

    uint8_t dataWriteBuffer[17];
    dataWriteBuffer[0] = deviceDisplayAddressPointer;
    dataWriteBuffer[1] = digit0;
    dataWriteBuffer[2] = digit1;
    dataWriteBuffer[3] = digit2;
    dataWriteBuffer[4] = digit3;
    dataWriteBuffer[5] = digit4;
    dataWriteBuffer[6] = 0;
    dataWriteBuffer[7] = 0;
    dataWriteBuffer[8] = 0;
    dataWriteBuffer[9] = 0;
    dataWriteBuffer[10] = 0;
    dataWriteBuffer[11] = 0;
    dataWriteBuffer[12] = 0;
    dataWriteBuffer[13] = 0;
    dataWriteBuffer[14] = 0;
    dataWriteBuffer[15] = 0;
    dataWriteBuffer[16] = 0;

    i2c_write_blocking(i2c_default, deviceAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);

    // i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayAddressPointer, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);

    while (true) {
        for (int i = 0; i < 8; i++) {
            dataWriteBuffer[1] = (uint8_t)(1 << i); // Digit 1
            dataWriteBuffer[3] = (uint8_t)(1 << i); // Digit 2
            dataWriteBuffer[5] = 0b00000010; // Colon
    //  dataWriteBuffer[6] = (uint8_t)(1 << i);
            dataWriteBuffer[7] = (uint8_t)(1 << i);
    //         dataWriteBuffer[8] = (uint8_t)(1 << i);
            dataWriteBuffer[9] = (uint8_t)(1 << i);
    //         dataWriteBuffer[10] = (uint8_t)(1 << i);
            i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
            sleep_ms(500);
        }
    }
}

static void fillDisplayArrayBuffer(float value, uint8_t *buffer, size_t bufferSize) {
    if (value > 999) {
        snprintf(buffer, bufferSize, "%.0f", value);
        return;
    }

    if (value > 99) {
        snprintf(buffer, bufferSize, "%.1f", value);
        return;
    }

    if (value > 9) {
        snprintf(buffer, bufferSize, "%.2f", value);
        return;
    }

    snprintf(buffer, bufferSize, "%.3f", value);
}

static void renderFrameBuffer(uint32_t *frameBuffer, uint8_t negative, uint8_t digit0, uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t dpPos) {
}
