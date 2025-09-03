
#include <stdio.h>
#include <string.h>

#include "characters.c"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 100
#endif

const int deviceAddress = 0x70;

const uint8_t digitPattern0 = 0b00111111;
const uint8_t digitPattern1 = 0b00000110;
const uint8_t digitPattern2 = 0b01011011;
const uint8_t digitPattern3 = 0b01001111;
const uint8_t digitPattern4 = 0b01100110;
const uint8_t digitPattern5 = 0b01101101;
const uint8_t digitPattern6 = 0b01111101;
const uint8_t digitPattern7 = 0b00000111;
const uint8_t digitPattern8 = 0b01111111;
const uint8_t digitPattern9 = 0b01101111;
const uint8_t digitPatternOff = 0b00000000;
const uint8_t digitPatternDash = 0b00111111;
const uint8_t colonPatternOn = 0b00000010;
const uint8_t colonPatternOff = 0b00000000;

const uint8_t deviceDisplayAddressPointer = 0b00000000;
const uint8_t deviceClockEnable = 0b00100001;
const uint8_t deviceRowIntSet = 0b10100000;
const uint8_t deviceDimmingSet = 0b11101111;    // Full brightness
const uint8_t deviceDisplayOnSet = 0b10000001;  // Device ON, blinking OFF.

uint8_t dataWriteBuffer[17];

uint8_t *const addrDigit0 = dataWriteBuffer + 1;
uint8_t *const addrDigit1 = dataWriteBuffer + 3;
uint8_t *const addrDigit2 = dataWriteBuffer + 7;
uint8_t *const addrDigit3 = dataWriteBuffer + 9;
uint8_t *const addrColon = dataWriteBuffer + 5;

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

int nthDigit(const int n, const int value) {
    // Calculate 10^(n-1)
    int tenthPower = 1;
    for (int i = 0; i < n - 1; i++) {
        tenthPower *= 10;
    }

    // Get the nth digit
    int nth_digit = (value / tenthPower) % 10;

    return nth_digit;
}

uint8_t patternForDigit(int digit) {
    switch (digit) {
        case 0:
            return digitPattern0;
            break;
        case 1:
            return digitPattern1;
            break;
        case 2:
            return digitPattern2;
            break;
        case 3:
            return digitPattern3;
            break;
        case 4:
            return digitPattern4;
            break;
        case 5:
            return digitPattern5;
            break;
        case 6:
            return digitPattern6;
            break;
        case 7:
            return digitPattern7;
            break;
        case 8:
            return digitPattern8;
            break;
        case 9:
            return digitPattern9;
            break;
    }

    return 0;
}

static void writeDataBuffer() {
    i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
}

static void writeInteger(int value) {
    *addrDigit0 = patternForDigit(nthDigit(4, value));
    *addrDigit1 = patternForDigit(nthDigit(3, value));
    *addrDigit2 = patternForDigit(nthDigit(2, value));
    *addrDigit3 = patternForDigit(nthDigit(1, value));

    writeDataBuffer();
}

int main() {
    initialize();

    LEDBlink(2);

    dataWriteBuffer[0] = deviceDisplayAddressPointer;

    i2c_write_blocking(i2c_default, deviceAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);

    // i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayAddressPointer, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);

    while (true) {
        for (int i = 0; i < 10000; i++) {
            writeInteger(i);
            sleep_ms(500);
        }
    }
}
    
// snprintf(buffer, bufferSize, "%.3f", value);
