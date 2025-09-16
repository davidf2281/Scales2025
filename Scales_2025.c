#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

const int displayI2CAddress = 0x70;
const int ADCI2CAddress = 0x2A;

// ADC constants
const uint8_t R0x00_address = 0x00;
const uint8_t R0x01_address = 0x01;
const uint8_t R0x02_address = 0x02;
const uint8_t R0x1C_address = 0x1C;
const uint8_t R0x12_address = 0x12;          // Start address of three-byte ADC reading
const uint8_t R0x15_address = 0x15;          // ADC control register
const uint8_t R0x1B_address = 0x1B;          // PGA (programmable gain amplifier) control register
const uint8_t R0x00_RR_Bit = 0b00000001;     // Register reset. 1 = Register Reset, reset all register except RR. 0 = Normal Operation (default). RR is a level trigger reset control. RR=1, enter reset state, RR=0, leave reset state back to normal state.
const uint8_t R0x00_PUD_Bit = 0b00000010;    // Power up digital circuit. 1 = Power up the chip digital logic, 0 = power down (default)
const uint8_t R0x00_PUA_Bit = 0b00000100;    // Power up analog circuit. 1 = Power up the chip analog circuits (PUD must be 1). 0 = Power down (default)
const uint8_t R0x00_PUR_Bit = 0b00001000;    // Power up ready (Read Only Status). 1 = Power Up ready, 0 = Power down, not ready
const uint8_t R0x00_CS_Bit = 0b00010000;     // Cycle start. Synchronize conversion to the rising edge of this registe
const uint8_t R0x00_CR_Bit = 0b00100000;     // Cycle ready (Read only Status). 1 = ADC DATA is ready
const uint8_t R0x00_OSCS_Bit = 0b01000000;   // System clock source select. 1 = External Crystal, 0 = Internal RC oscillator (default)
const uint8_t R0x00_AVDDS_Bit = 0b10000000;  // AVDD source select. 1 = Internal LDO, 0 = AVDD pin input (default)
const uint8_t R0x02_CALS_Bit = 0b00000100;
const uint8_t R0x02_CAL_ERR_Bit = 0b00001000;

const uint8_t R0x1B_PGACHPDIS_Bit = 0b00000001;      // Chopper disable bit. 1 = disabled
const uint8_t R0x1B_PGA_BUFFER_ENABLE = 0b00100000;  // Enable PGA output buffer
const uint8_t R0x1B_PGA_LDO_MODE = 0b01000000;       // LDO mode 1 = "improved stability and lower DC gain, can accommodate ESR < 5 ohms (output capacitance)"

// Display constants
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
    i2c_init(i2c_default, 100000);
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

static void initDisplay() {
    dataWriteBuffer[0] = deviceDisplayAddressPointer;

    i2c_write_blocking(i2c_default, displayI2CAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, displayI2CAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, displayI2CAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, displayI2CAddress, &deviceDisplayOnSet, 1, false);
    i2c_write_blocking(i2c_default, displayI2CAddress, dataWriteBuffer, 17, false);
    i2c_write_blocking(i2c_default, displayI2CAddress, &deviceDisplayOnSet, 1, false);
}

static void writeADCI2CByte(uint8_t registerAddress, uint8_t data) {
    uint8_t writeBuffer[2];
    writeBuffer[0] = registerAddress;
    writeBuffer[1] = data;
    i2c_write_blocking(i2c_default, ADCI2CAddress, writeBuffer, 2, false);
}

static uint8_t readADCI2CByte(uint8_t registerAddress) {
    uint8_t result[1];
    i2c_write_blocking(i2c_default, ADCI2CAddress, &registerAddress, 1, false);
    i2c_read_blocking(i2c_default, ADCI2CAddress, result, 1, false);
    return result[0];
}

static void readADCI2CBytes(uint8_t registerAddress, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c_default, ADCI2CAddress, &registerAddress, 1, false);
    i2c_read_blocking(i2c_default, ADCI2CAddress, buffer, length, false);
}

// Returns true if calibration successful; false otherwise
static bool calibrateADC() {
    uint8_t writeBuffer[2];

    // Kick off device internal calibration
    writeADCI2CByte(R0x02_address, R0x02_CALS_Bit);
    LEDBlink(1);
    // Wait for calibration and check it's complete
    sleep_ms(500);
    bool done = false;
    while (!done) {
        done = (readADCI2CByte(R0x02_address) & R0x02_CALS_Bit) == 0;
        sleep_us(200);
    }
    LEDBlink(1);
    // Get the calibration error flag:
    uint8_t calibrationResultByte = readADCI2CByte(R0x02_address);
    bool success = ((calibrationResultByte & R0x02_CAL_ERR_Bit) == 0);

    return success;
}

static int32_t readADC() {
    // Read the three ADC bytes
    uint8_t reading[3];
    readADCI2CBytes(R0x12_address, reading, 3);

    int32_t value = (int32_t)((reading[0] << 16) | (reading[1] << 8) | reading[2]);

    // Extend the sign into the topmost byte with a bitwise left-shift
    // followed by an arithmetic right-shift:
    value = (value << 8) >> 8;

    // uint32_t result = ((uint32_t)reading[0] << 16) | ((uint32_t)reading[1] << 8) | (uint32_t)reading[2];

    return value;
}

static void initADC() {
    uint8_t writeBuffer[2];

    // 1. Set the RR bit to 1 in R0x00, to guarantee a reset of all register values.
    writeADCI2CByte(R0x00_address, R0x00_RR_Bit);

    // 2. Set the RR bit to 0 and PUD bit 1, in R0x00, to enter normal operation
    writeADCI2CByte(R0x00_address, (R0x00_RR_Bit & 0b00000000) | R0x00_PUD_Bit);

    // 3. After about 200 microseconds, the PWRUP bit will be Logic 1,
    // indicating the device is ready for the remaining programming setup.

    sleep_us(300);
    while(!(readADCI2CByte(R0x00_address) & R0x00_PUR_Bit));

    // Write register R0x00 configuration:
    writeADCI2CByte(R0x00_address, 0xAE); // Internal LDO selected; internal oscillator; power up analog and digital circuits.

    // Enable PGA output bypass capacitor (which the Adafruit NAU7802 board has installed):
    const uint8_t R0x1C_PGA_CAP_EN_Bit = 0b10000000;
    writeADCI2CByte(R0x1C_address, R0x1C_PGA_CAP_EN_Bit);

    /*
        Set device LDO output voltage and analogue input gain:
            Bits 5 to 3 of register 0x01 select LDO voltage. 100 == 3.3V
            Bits 2 to 0 of register 0x01 select device gain. 111 == 128x
    */
    const uint8_t LDOVoltageMask = 0b00010000;
    const uint8_t gainSelectMask = 0b00000111;
    writeADCI2CByte(R0x01_address, LDOVoltageMask | gainSelectMask);

    // Ensure LDO mode bit is zero
    writeADCI2CByte(R0x1B_address, 0);

    // Disable ADC chopper
    writeADCI2CByte(R0x15_address, 0b00110000);

    // Set sample rate to 80SPS
    writeADCI2CByte(R0x02_address,0b01100000);

    // Wait for LDO to stablize
    sleep_ms(300);

    // Flush
    readADC();

    // Kick off device internal calibration
    bool calibrated = false;
    while (!calibrated) {
        calibrated = calibrateADC();
        if (!calibrated) {
            printf("Cal failed\n");
            sleep_ms(500);
        }
    }
}

#define ADC_DELAY 12500

int32_t averageADC(uint8_t samples) {
    int32_t tally = 0;
    for (int i = 0; i < samples; i++) {
        tally += readADC();
        sleep_us(ADC_DELAY);
    }
    return tally / samples;
}

static bool initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    initGPIO();
    initI2C();
    initDisplay();
    initADC();
}

static void writeDataBuffer() {
    i2c_write_blocking(i2c_default, displayI2CAddress, dataWriteBuffer, 17, false);
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

    setOverflow();
    writeDataBuffer();

    printf("Starting.\n");

    sleep_ms(3000);

    // int32_t calibration = averageADC(32);

    // while (true) {
    //     int32_t reading = averageADC(32);
    //     double display = (double)(reading - calibration) / 408;
    //     setDouble(display);
    //     writeDataBuffer();
    //     printf("Reading: %d\n", reading);
    //     printf("Display: %f\n\n", display);
    // }

    // while (true) {
    //     int32_t reading = readADC();
    //     printf("Reading: %d\n", reading);
    //     sleep_ms(500);
    // }
    int count = 100;
    int32_t readings[count];
    int32_t readingAcc = 0;
    // int32_t average = averageADC(count);

    for (int i = 0; i < count; i++) {
        int32_t reading = readADC();
        sleep_us(ADC_DELAY);
        readings[i] = reading;
    }

    int32_t max = INT32_MIN;
    int32_t min = INT32_MAX;

    for (int i = 0; i < count; i++) {
        int32_t reading = readings[i];

        readingAcc += reading;

        if (reading > max) {
            max = reading;
        }

        if (reading < min) {
            min = reading;
        }
    }

    int range = abs(max - min);
    int average = readingAcc / count;
    printf("Min: %d, max: %d, range: %d, average: %d\n", min, max, range, average);

    // setInteger(range);
    // writeDataBuffer();

    // for (int i = 0; i < count; i++) {
    //     printf("%d\n", readings[i]);
    // }

    // printf("Done.\n");

    int readingCount = 0;
    while (true) {
        int32_t reading = averageADC(32);
        double display = (double)(reading - average) / 420;  // 408
        setDouble(display);
        writeDataBuffer();
        readingCount++;
        if (readingCount > 100) {
            setOverflow();
            writeDataBuffer();
            bool calibrated = false;
            while (!calibrated) {
                calibrated = calibrateADC();
                if (!calibrated) {
                    printf("Cal failed\n");
                    sleep_ms(500);
                } else {
                    printf("Cal succeeded\n");
                }
            }
            readingCount = 0;
        }
    }
}
