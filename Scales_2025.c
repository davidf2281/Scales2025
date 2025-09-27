#include <stdio.h>
#include <stdlib.h>

#include "BME280.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define ADC_DELAY 12500

const int displayI2CAddress = 0x70;
const int ADCI2CAddress = 0x2A;
const int BME280I2CAddress = 0x76;

// ADC constants
const uint8_t R0x00_address = 0x00;
const uint8_t R0x01_address = 0x01;
const uint8_t R0x02_address = 0x02;
const uint8_t R0x11_address = 0x11;  // I2C control
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

const uint8_t R0x11_TS_Bit = 0b00000010;

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

// BME280 (temperature sensor) constants
const uint8_t BME280_control_reset = 0xE0;
const uint8_t BME280_control_deviceID = 0xD0;
const uint8_t BME280_control_hum = 0xF2;
const uint8_t BME280_control_meas = 0xF4;
const uint8_t BME280_cal_reg_digT1 = 0x88;
const uint8_t BME280_cal_reg_digT2 = 0x8A;
const uint8_t BME280_cal_reg_digT3 = 0x8C;
const uint8_t BME280_cal_reg_digP1 = 0x8E;
const uint8_t BME280_cal_reg_digP2 = 0x90;
const uint8_t BME280_cal_reg_digP3 = 0x92;
const uint8_t BME280_cal_reg_digP4 = 0x94;
const uint8_t BME280_cal_reg_digP5 = 0x96;
const uint8_t BME280_cal_reg_digP6 = 0x98;
const uint8_t BME280_cal_reg_digP7 = 0x9A;
const uint8_t BME280_cal_reg_digP8 = 0x9C;
const uint8_t BME280_cal_reg_digP9 = 0x9E;
const uint8_t BME280_cal_reg_digH1 = 0xA1;
const uint8_t BME280_cal_reg_digH2 = 0xE1;
const uint8_t BME280_cal_reg_digH3 = 0xE3;
const uint8_t BME280_cal_reg_digH4 = 0xE4;
const uint8_t BME280_cal_reg_digH5 = 0xE5;
const uint8_t BME280_cal_reg_digH6 = 0xE7;
const uint8_t BME280_reg_pressure_data = 0xF7;
const uint8_t BME280_reg_temperature_data = 0xFA;

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

static void setADCI2Cbit(uint8_t regAddress, uint8_t byteMask, bool value) {
    uint8_t currentRegValue = readADCI2CByte(regAddress);
    uint8_t bitZeroedRegvalue = currentRegValue ^ byteMask;
    uint8_t bitSetRegValue = bitZeroedRegvalue | (value ? byteMask : 0);
    writeADCI2CByte(regAddress, bitSetRegValue);
}

// Returns true if calibration successful; false otherwise
static bool calibrateADC() {
    uint8_t writeBuffer[2];

    // Kick off device internal calibration
    writeADCI2CByte(R0x02_address, R0x02_CALS_Bit);

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

static void flushADC() {
    for (int i = 0; i < 6; i++) {
        readADC();
        sleep_us(ADC_DELAY);
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
    const uint8_t R0x1C_PGA_CAP_EN_Bit = 0b10000000;
    writeADCI2CByte(R0x1C_address, R0x1C_PGA_CAP_EN_Bit);

    /*
        Set device LDO output voltage and analogue input gain:
            Bits 5 to 3 of register 0x01 select LDO voltage. 100 == 3.3V
            Bits 2 to 0 of register 0x01 select device gain. 111 == 128x
    */
    const uint8_t LDOVoltageMask = 0b00100000;
    const uint8_t gainSelectMask = 0b00000111;
    writeADCI2CByte(R0x01_address, LDOVoltageMask | gainSelectMask);

    // Ensure LDO mode bit is zero
    writeADCI2CByte(R0x1B_address, 0);

    // Disable ADC chopper
    writeADCI2CByte(R0x15_address, 0b00110000);

    // Set sample rate to 80SPS
    writeADCI2CByte(R0x02_address, 0b01100000);

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
        sleep_us(ADC_DELAY);
    }
    return tally / samples;
}


static void writeBME280Byte(uint8_t registerAddress, uint8_t data) {
    uint8_t writeBuffer[2];
    writeBuffer[0] = registerAddress;
    writeBuffer[1] = data;
    i2c_write_blocking(i2c_default, BME280I2CAddress, writeBuffer, 2, false);
}

static void resetBME280() {
    writeBME280Byte(BME280_control_reset, 0xB6);
    sleep_ms(100);
}

static uint8_t readBME280I2CByte(uint8_t registerAddress) {
    uint8_t result[1];
    i2c_write_blocking(i2c_default, BME280I2CAddress, &registerAddress, 1, false);
    i2c_read_blocking(i2c_default, BME280I2CAddress, result, 1, false);
    return result[0];
}

static uint16_t readBME280I2CWord(uint8_t registerAddress) {
    uint8_t result[2];
    i2c_write_blocking(i2c_default, BME280I2CAddress, &registerAddress, 1, false);
    i2c_read_blocking(i2c_default, BME280I2CAddress, result, 2, false);
    return (uint16_t)((uint16_t)result[1] << 8 | (uint16_t)result[0]);  // LSB first
}

static bme280_calib_data readBME280CalibrationData() {
    bme280_calib_data calibData;

    calibData.dig_t1 = readBME280I2CWord(BME280_cal_reg_digT1);
    calibData.dig_t2 = (int16_t)readBME280I2CWord(BME280_cal_reg_digT2);
    calibData.dig_t3 = (int16_t)readBME280I2CWord(BME280_cal_reg_digT3);

    // printf("Calib data t1: %u, t2: %i, t3: %i\n", calibData.dig_t1, calibData.dig_t2, calibData.dig_t3);

    return calibData;
}

double getTemperature(bme280_calib_data *calibData) {
    // Write humidity config, which acccording to the BME280 datasheet must be done before writing measurement config
    uint8_t humidtyConfig = 0;
    writeBME280Byte(BME280_control_hum, humidtyConfig);

    // Write measurement config to put the device into forced mode, which will start a measurement
    uint8_t ctrlMeasConfig = 0b01101110;  // 4x temperature oversampling, 4x pressure oversample, sensor to forced mode.
    writeBME280Byte(BME280_control_meas, ctrlMeasConfig);

    // Wait for measurement
    sleep_ms(100);

    // Read temperature
    uint8_t temperatureByte1 = readBME280I2CByte(BME280_reg_temperature_data);  // MSB
    uint8_t temperatureByte2 = readBME280I2CByte(BME280_reg_temperature_data + 1);
    uint8_t temperatureByte3 = readBME280I2CByte(BME280_reg_temperature_data + 2);  // LSB (top four bits only)

    // printf("Temp byte1: %u, byte2: %u, byte3: %u\n", temperatureByte1, temperatureByte2, temperatureByte3);

    // Temperature readout is the top 20 bits of the three bytes
    uint32_t uncompTemperatureValue = (uint32_t)((uint32_t)temperatureByte1 << 12 | (uint32_t)temperatureByte2 << 4 | (uint32_t)temperatureByte3 >> 4);

    // printf("Uncomp temp here: %u\n", uncompTemperatureValue);

    // Note: compensate_temperature() must be called before compensate_pressure()
    // because it calculates and sets t_fine in the calibData struct
    struct bme280_uncomp_data uncompData;
    uncompData.temperature = uncompTemperatureValue;
    // printf("Uncomp temp here too : %u\n", uncompData.temperature);

    double compTemperature = compensate_temperature(&uncompData, calibData);

    return compTemperature;
}

static void initBME280() {
    resetBME280();
}

static bool initialize() {
    stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    initGPIO();
    initI2C();
    initDisplay();
    // initADC();
    // initBME280();
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

int main() {
    initialize();

    LEDBlink(2);

    // bme280_calib_data calibData = readBME280CalibrationData();

    // flushADC();
    
    // int32_t zeroReading = averageADC(64);

    // double initialTemperature = getTemperature(&calibData);

    setOverflow();
    writeDataBuffer();

    // sleep_ms(20000);

    printf("Starting.\n");

    // printf("Initial ADC reading: %i, initial temp reading: %.2f\n", zeroReading, initialTemperature);

    // printf("Waiting 30 mins for temp equalisation\n");

    // sleep_minutes(30);

    // for (int i = 0; i < 1440; i++) {
    //     double reading = averageADC(64);
    //     double mass = (double)(reading - zeroReading) / 408.0;
    //     double temperature = getTemperature(&calibData);
    //     double tempDiff = temperature - initialTemperature;
    //     printf("%.3f, %.3f, %.3f\n", mass, tempDiff, temperature);
    //     sleep_ms(1000);
    // }

    while(true) {
        LEDBlink(4);
        sleep_ms(1000);
    }

    // printf("Done.\n");
}
