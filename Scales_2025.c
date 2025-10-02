#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define ADC_DELAY_uS 100000

const int displayI2CAddress = 0x70;
const int ADCI2CAddress = 0x2A;

// ADC constants
const uint8_t R0x00_address = 0x00;
const uint8_t R0x01_address = 0x01;
const uint8_t R0x02_address = 0x02;
const uint8_t R0x11_address = 0x11;  // I2C control
const uint8_t R0x1C_address = 0x1C;
const uint8_t R0x1F_address = 0x1F;
const uint8_t R0x12_address = 0x12;  // Start address of three-byte ADC reading
const uint8_t R0x15_address = 0x15;  // ADC control register
const uint8_t R0x1B_address = 0x1B;  // PGA (programmable gain amplifier) control register

const uint8_t R0x00_RR_Bit = 0b00000001;     // Register reset. 1 = Register Reset, reset all register except RR. 0 = Normal Operation (default). RR is a level trigger reset control. RR=1, enter reset state, RR=0, leave reset state back to normal state.
const uint8_t R0x00_PUD_Bit = 0b00000010;    // Power up digital circuit. 1 = Power up the chip digital logic, 0 = power down (default)
const uint8_t R0x00_PUA_Bit = 0b00000100;    // Power up analog circuit. 1 = Power up the chip analog circuits (PUD must be 1). 0 = Power down (default)
const uint8_t R0x00_PUR_Bit = 0b00001000;    // Power up ready (Read Only Status). 1 = Power Up ready, 0 = Power down, not ready
const uint8_t R0x00_CS_Bit = 0b00010000;     // Cycle start. Synchronize conversion to the rising edge of this registe
const uint8_t R0x00_CR_Bit = 0b00100000;     // Cycle ready (Read only Status). 1 = ADC DATA is ready
const uint8_t R0x00_OSCS_Bit = 0b01000000;   // System clock source select. 1 = External Crystal, 0 = Internal RC oscillator (default)
const uint8_t R0x00_AVDDS_Bit = 0b10000000;  // AVDD source select. 1 = Internal LDO, 0 = AVDD pin input (default)
const uint8_t R0x01_VLDO_Bits = 0b00111000;
const uint8_t R0x02_CHS_Bit = 0b10000000;
const uint8_t R0x02_CALS_Bit = 0b00000100;
const uint8_t R0x02_CAL_ERR_Bit = 0b00001000;

const uint8_t R0x11_TS_Bit = 0b00000010;

const uint8_t R0x1B_PGACHPDIS_Bit = 0b00000001;      // Chopper disable bit. 1 = disabled
const uint8_t R0x1B_PGA_BUFFER_ENABLE = 0b00100000;  // Enable PGA output buffer
const uint8_t R0x1B_PGA_LDO_MODE = 0b01000000;       // LDO mode 1 = "improved stability and lower DC gain, can accommodate ESR < 5 ohms (output capacitance)"

const uint8_t R0x1C_CAP_ENABLE = 0b10000000;  // Enables PGA output bypass capacitor connected across pins Vin2P Vin2N

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

// I2C constants
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

static void LEDBlinkLong(int repeats, int delay_ms) {
    for (int i = 0; i < repeats; i++) {
        pico_set_led(true);
        sleep_ms(delay_ms);
        pico_set_led(false);
        sleep_ms(delay_ms);
    }
}

static void initI2C() {
    i2c_init(i2c_default, 100000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
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
    int errorByte = 0;

    errorByte = i2c_write_blocking(i2c_default, ADCI2CAddress, writeBuffer, 2, false);

    if (errorByte == PICO_ERROR_GENERIC) {
        printf("Write error.\n");
    }
}

static uint8_t readADCI2CByte(uint8_t registerAddress) {
    uint8_t result[1];
    int errorByte = 0;
    errorByte = i2c_write_blocking(i2c_default, ADCI2CAddress, &registerAddress, 1, false);

    if (errorByte == PICO_ERROR_GENERIC) {
        printf("Write error.\n");
    }

    errorByte = i2c_read_blocking(i2c_default, ADCI2CAddress, result, 1, false);

    if (errorByte == PICO_ERROR_GENERIC) {
        printf("Read error.\n");
    }

    return result[0];
}

static void readADCI2CBytes(uint8_t registerAddress, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c_default, ADCI2CAddress, &registerAddress, 1, false);
    i2c_read_blocking(i2c_default, ADCI2CAddress, buffer, length, false);
}

static void setADCI2Cbit(uint8_t regAddress, uint8_t byteMask, uint8_t value) {
    uint8_t initialRegValue = readADCI2CByte(regAddress);
    uint8_t bitZeroedRegvalue = initialRegValue & ~byteMask;
    uint8_t bitSetRegValue = bitZeroedRegvalue | (value ? byteMask : 0);
    writeADCI2CByte(regAddress, bitSetRegValue);
}

static void setADCI2Cbits(uint8_t regAddress, uint8_t byteMask, uint8_t value) {
    uint8_t initialRegValue = readADCI2CByte(regAddress);
    uint8_t bitZeroedRegvalue = initialRegValue & ~byteMask;
    uint8_t bitSetRegValue = bitZeroedRegvalue | value;
    writeADCI2CByte(regAddress, bitSetRegValue);
}

// Channel = 0 or 1
static void selectADCChannel(uint8_t channel) {
    setADCI2Cbit(R0x02_address, R0x02_CHS_Bit, channel);
}

static void setADCGain128() {
    setADCI2Cbits(R0x01_address, 0b00000111, 0b00000111);
}

static void setADCGain64() {
    setADCI2Cbits(R0x01_address, 0b00000111, 0b00000110);
}

static void setADCGain32() {
    setADCI2Cbits(R0x01_address, 0b00000111, 0b00000101);
}

// Returns true if calibration successful; false otherwise
static bool calibrateADC() {
    // Capture initial initial register state:
    uint8_t initialRegValue = readADCI2CByte(R0x02_address);

    uint8_t writeBuffer[2];

    // Kick off device internal calibration
    setADCI2Cbit(R0x02_address, R0x02_CALS_Bit, 1);

    // Wait for calibration and check it's complete
    sleep_ms(500);
    bool done = false;
    while (!done) {
        done = (readADCI2CByte(R0x02_address) & R0x02_CALS_Bit) == 0;
        sleep_us(200);
    }

    // Get the calibration error flag:
    uint8_t calibrationResultByte = readADCI2CByte(R0x02_address);
    bool success = ((calibrationResultByte & R0x02_CAL_ERR_Bit) == 0);

    // Restore initial register state:
    writeADCI2CByte(R0x02_address, initialRegValue);

    return success;
}

static int32_t readADC() {
    // while (!(readADCI2CByte(R0x00_address) & R0x00_CR_Bit)) {
    sleep_us(ADC_DELAY_uS);
    // }

    // Read the three ADC bytes
    uint8_t reading[3];
    readADCI2CBytes(R0x12_address, reading, 3);

    int32_t value = (int32_t)((reading[0] << 16) | (reading[1] << 8) | reading[2]);

    // Extend the sign into the topmost byte with a bitwise left-shift
    // followed by an arithmetic right-shift:
    value = (value << 8) >> 8;

    return value;
}

static void flushADC() {
    for (int i = 0; i < 6; i++) {
        readADC();
    }
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
    while (!(readADCI2CByte(R0x00_address) & R0x00_PUR_Bit));

    // Write register R0x00 configuration:
    writeADCI2CByte(R0x00_address, 0xAE);  // Internal LDO selected; internal oscillator; power up analog and digital circuits.

    // Enable PGA output bypass capacitor (which the Adafruit NAU7802 board has installed):
    writeADCI2CByte(R0x1C_address, R0x1C_CAP_ENABLE);

    // Disable PGA output bypass capacitor (which the Adafruit NAU7802 board has installed) for two-channel operation:
    // writeADCI2CByte(R0x1C_address, 0);

    /*
        Set device LDO output voltage and analogue input gain:
            Bits 5 to 3 of register 0x01 select LDO voltage. 100 == 3.3V
            Bits 2 to 0 of register 0x01 select device gain. 111 == 128x
    */
    const uint8_t LDOVoltageMask = 0b00100000;
    const uint8_t gainSelectMask = 0b00000111;
    writeADCI2CByte(R0x01_address, LDOVoltageMask);
    // writeADCI2CByte(R0x01_address, LDOVoltageMask);
    setADCGain128();

    // Ensure LDO mode bit is zero
    writeADCI2CByte(R0x1B_address, 0);

    // Disable ADC chopper
    writeADCI2CByte(R0x15_address, 0b00110000);

    // Set sample rate to 10SPS (bits 6 to 0):
    // 111 = 320SPS
    // 011 = 80SPS
    // 010 = 40SPS
    // 001 = 20SPS
    // 000 = 10SPS
    writeADCI2CByte(R0x02_address, 0b00000000);

    // Wait for LDO to stablize
    sleep_ms(300);

    // Flush
    flushADC();

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

int32_t averageADC(uint8_t samples) {
    int32_t tally = 0;
    for (int i = 0; i < samples; i++) {
        tally += readADC();
    }
    return tally / samples;
}

int32_t averageADCTopAndTail() {
    const uint8_t sampleCount = 4;
    int32_t samples[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
        samples[i]= readADC();
    }

    uint8_t minIndex;
    uint8_t maxIndex;
    int32_t min = INT32_MAX;
    int32_t max = INT32_MIN;
    for (int i = 0; i < sampleCount; i++) {
        const int32_t sample = samples[i];
        if (sample < min) {
            min = sample;
            minIndex = i;
        }
        if(sample > max) {
            max = sample;
            maxIndex = i;
        }
    }

    int32_t tally = 0;
    for (int i = 0; i < sampleCount; i++) {
        if (i != minIndex && i != maxIndex) {
            tally += samples[i];
        }
    }

    return tally / (sampleCount - 2);
}

static void writeDataBuffer() {
    i2c_write_blocking(i2c_default, displayI2CAddress, dataWriteBuffer, 17, false);
}

static void setOverflow() {
    *addrDigit0 = digitPatternDash;
    *addrDigit1 = digitPatternDash;
    *addrDigit2 = digitPatternDash;
    *addrDigit3 = digitPatternDash;
}

static bool initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    initGPIO();
    initI2C();
    initDisplay();
    setOverflow();
    writeDataBuffer();
    initADC();
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

static int truncateToIntWithRounding(double value) {
    const int temp = (int)(value * 10);

    if (nthDigit(1, temp) > 4) {
        return (int)(value) + 1;
    }

    return (int)value;
}

// Maximum sleep_ms is just over a minute, so
// this function allows larger sleep times
static void sleep_minutes(uint32_t minutes) {
    for (int i = 0; i < minutes; i++) {
        sleep_ms(60000);
    }
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

static void checkPGACAP() {
    if ((readADCI2CByte(R0x1C_address) & R0x1C_CAP_ENABLE) == R0x1C_CAP_ENABLE) {
        printf("PGA cap is enabled\n");
    } else {
        printf("PGA cap is disabled\n");
    }
}

int main() {
    initialize();

    LEDBlink(2);

    selectADCChannel(0);
    setADCGain128();
    calibrateADC();
    flushADC();
    sleep_ms(300);
    const int averagingCount = 3;
    const int32_t zeroScaleReading = averageADC(averagingCount);

    // selectADCChannel(1);
    // setADCGain64();
    // calibrateADC();
    // checkPGACAP();
    // flushADC();
    // sleep_ms(300);
    // int32_t zeroTemperatureReading = averageADC(averagingCount);

    setOverflow();
    writeDataBuffer();
    
    sleep_ms(10000);

    printf("Starting.\n");

    const int sampleCount = 2000;
    int32_t samples[sampleCount];
    absolute_time_t startTime = get_absolute_time();

    for (int i = 0; i < sampleCount; i++) {
        // Scale reading
        // selectADCChannel(0);
        // setADCGain128();
        // calibrateADC();
        // sleep_ms(300);
        // flushADC();
        samples[i] = averageADC(averagingCount);
        // double scaleReading = averageADC(averagingCount);

        // double mass = (double)(scaleReading - zeroScaleReading) / 408.0;

        // Temperature reading
        // selectADCChannel(1);
        // setADCGain64();
        // calibrateADC();
        // flushADC();
        // sleep_ms(300);
        // checkPGACAP();
        // double temperatureReading = averageADC(averagingCount);

        // printf("%.3f, %.3f\n", mass /*, temperatureReading - zeroTemperatureReading*/);

        // double temperature = getTemperature(&calibData);
        // double tempDiff = temperature - initialTemperature;
        // printf("%.3f, %.3f, %.3f\n", mass, tempDiff, temperature);
        // sleep_ms(500);
    }

    absolute_time_t endTime = get_absolute_time();
    uint32_t startMillis = to_ms_since_boot(startTime);
    uint32_t endMillis = to_ms_since_boot(endTime);

    printf("Done. Took %.1fs\n\n", (float)(endMillis - startMillis) / 1000.0);

    for (int i = 0; i < sampleCount; i++) {
        double mass = (double)(samples[i] - zeroScaleReading) / 408.0;
        printf("%.3f\n", mass);
    }

    // while (true) {
    //     LEDBlink(4);
    //     sleep_ms(1000);
    // }

    // printf("Done.\n");
}
