#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

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
const uint8_t digitPatternDash = 0b01000000;
const uint8_t digitPatternAllSegmentsOn = 0b11111111;
const uint8_t digitMaskDecimalPointOn = 0b10000000;
const uint8_t digitMaskDecimalPointOff = 0b01111111;
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

const uint CLOCK_PIN = 14;
const uint DATA_PIN = 15;

static int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

static void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

static void LEDBlink(int repeats) {
    for (int i = 0; i < repeats; i++) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }
}

static void initI2C() {
    i2c_init(i2c_default, 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

static void initGPIO() {
    gpio_init(CLOCK_PIN);
    gpio_set_dir(CLOCK_PIN, GPIO_OUT);
    gpio_put(CLOCK_PIN, 0);

    gpio_init(DATA_PIN);
    gpio_set_dir(DATA_PIN, GPIO_IN);
}

static void writeDataBuffer() {
    i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
}

static uint32_t readHX711() {
    uint32_t reading = 0;

    // Wait for reading to be ready
    while (gpio_get(DATA_PIN));  // Data pin is high to indicate not ready; low to indicate ready.

    // HX711 goes into standby if the clock stays high for more
    // than 60uS, so disable interrupts to make sure the CPU
    // doesn't go off and service one mid-reading.
    const uint32_t interruptState = save_and_disable_interrupts();

    // Shift in our 24-bit reading:
    for (int i = 0; i < 24; i++) {
        gpio_put(CLOCK_PIN, 1);
        sleep_us(1);
        reading = reading << 1;
        if (gpio_get(DATA_PIN) == 1) {
            reading++;
        }  // TODO: Get rid of the conditional
        gpio_put(CLOCK_PIN, 0);
        sleep_us(1);
    }

    // Pulse the clock pin 1-3 times to set gain and channel for next reading:
    // 1 pulse:  Input channel A with gain of 128
    // 2 pulses: Input channel B with gain of 32
    // 3 pulses: Input channel A with gain of 64

    for (int i = 0; i < 1; i++) {
        gpio_put(CLOCK_PIN, 1);
        sleep_us(1);
        gpio_put(CLOCK_PIN, 0);
    }

    // Restore interrupts passing in our saved state
    restore_interrupts_from_disabled(interruptState);

    // Convert the signed two's complement output from the HX711 to unsigned:
    reading = reading ^ 0x800000;

    return reading;
}

static void resetHX711() {
    // Taking the clock pin high for more than 60uS and
    // then taking it low resets the HX711.
    gpio_put(CLOCK_PIN, 1);
    sleep_us(100);
    gpio_put(CLOCK_PIN, 0);

    // Datasheet says the settling time after reset is 400ms
    sleep_ms(500);

    // Take a single reading to set the channel & gain
    readHX711();
}

uint32_t averageHX711(uint8_t samples) {
    uint32_t tally = 0;
    for (int i = 0; i < samples; i++) {
        tally += readHX711();
    }
    return tally / samples;
}

static void initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    initGPIO();
    initI2C();
}

// Retrieves nth digit starting from the right and moving left, eg:
// nthDigit(1, 5678) will return 8.
// nthDigit(4, 5678) will return 5.
static int nthDigit(const int n, const int value) {
    // Calculate 10^(n-1)
    int tenthPower = 1;
    for (int i = 0; i < n - 1; i++) {
        tenthPower *= 10;
    }

    // Get the nth digit
    int nth_digit = (value / tenthPower) % 10;

    return nth_digit;
}

static uint8_t patternForDigit(int digit) {
    switch (digit) {
        case 0:
            return digitPattern0;
        case 1:
            return digitPattern1;
        case 2:
            return digitPattern2;
        case 3:
            return digitPattern3;
        case 4:
            return digitPattern4;
        case 5:
            return digitPattern5;
        case 6:
            return digitPattern6;
        case 7:
            return digitPattern7;
        case 8:
            return digitPattern8;
        case 9:
            return digitPattern9;
    }

    return digitPatternDash;
}

static void clearDecimalPoint() {
    *addrDigit0 = *addrDigit0 & digitMaskDecimalPointOff;
    *addrDigit1 = *addrDigit1 & digitMaskDecimalPointOff;
    *addrDigit2 = *addrDigit2 & digitMaskDecimalPointOff;
    *addrDigit3 = *addrDigit3 & digitMaskDecimalPointOff;
}

static void setDecimalPoint(int position) {
    clearDecimalPoint();

    switch (position) {
        case 0:
            *addrDigit0 = *addrDigit0 | digitMaskDecimalPointOn;
            return;

        case 1:
            *addrDigit1 = *addrDigit1 | digitMaskDecimalPointOn;
            return;

        case 2:
            *addrDigit2 = *addrDigit2 | digitMaskDecimalPointOn;
            return;

        case 3:
            *addrDigit3 = *addrDigit3 | digitMaskDecimalPointOn;
            return;
    }
}

static void setInteger(int value) {
    *addrDigit0 = patternForDigit(nthDigit(4, value));
    *addrDigit1 = patternForDigit(nthDigit(3, value));
    *addrDigit2 = patternForDigit(nthDigit(2, value));
    *addrDigit3 = patternForDigit(nthDigit(1, value));
}

static void setOverflow() {
    *addrDigit0 = digitPatternDash;
    *addrDigit1 = digitPatternDash;
    *addrDigit2 = digitPatternDash;
    *addrDigit3 = digitPatternDash;
}

static int truncateToIntWithRounding(double value) {
    const int temp = (int)(value * 10);

    if (nthDigit(1, temp) > 4) {
        return (int)(value) + 1;
    }

    return (int)value;
}

static void setDouble(double value) {
    if (value >= 10000) {
        setOverflow();
        return;
    }

    // TODO: Handle negative values by setting first digit "-" and reducing to three-digit precision

    if (value < 10) {
        setInteger(truncateToIntWithRounding(value * 1000));
        setDecimalPoint(0);
    } else if (value < 100) {
        setInteger(truncateToIntWithRounding(value * 100));
        setDecimalPoint(1);
    } else if (value < 1000) {
        setInteger(truncateToIntWithRounding(value * 10));
        setDecimalPoint(2);
    } else if (value >= 1000) {
        setInteger(truncateToIntWithRounding(value));
        clearDecimalPoint();
    }
}

int main() {
    initialize();

    LEDBlink(2);

    dataWriteBuffer[0] = deviceDisplayAddressPointer;

    i2c_write_blocking(i2c_default, deviceAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);
    i2c_write_blocking(i2c_default, deviceAddress, dataWriteBuffer, 17, false);
    i2c_write_blocking(i2c_default, deviceAddress, &deviceDisplayOnSet, 1, false);

    setOverflow();
    writeDataBuffer();

    resetHX711();
    averageHX711(64);
    uint32_t calibration = averageHX711(64);

    while (true) {
        uint32_t reading = averageHX711(64);
        double display = (double)((reading - calibration));
        setDouble(display);
        writeDataBuffer();
        printf("Reading: %lu\n", reading);
        printf("Display: %f\n\n", display);
    }
}
