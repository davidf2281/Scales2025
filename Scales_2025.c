// #pragma GCC optimize("O0")

#include <stdio.h>
#include <stdlib.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// The GPIO SDK refers to logical not physical pins,
// eg, GPIO 'pin' 6 is physical pin 9 on the Pico itself.
#define DRDY_PIN 6
#define PS_PIN 23  // Controls voltage regulator. Set high to force PWM mode which reduces supply ripple (at the expense of lower efficiency)
#define VSYS_INPUT_PIN 29
#define TARE_PIN 22

const int display0I2CAddress = 0x70;
const int display1I2CAddress = 0x71;
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
const uint8_t R0x00_CS_Bit = 0b00010000;     // Cycle start. Synchronize conversion to the rising edge of this register
const uint8_t R0x00_CR_Bit = 0b00100000;     // Cycle ready (Read only Status). 1 = ADC DATA is ready
const uint8_t R0x00_OSCS_Bit = 0b01000000;   // System clock source select. 1 = External Crystal, 0 = Internal RC oscillator (default)
const uint8_t R0x00_AVDDS_Bit = 0b10000000;  // AVDD source select. 1 = Internal LDO, 0 = AVDD pin input (default)

const uint8_t R0x01_VLDO_Bits = 0b00111000;

const uint8_t R0x02_CHS_Bit = 0b10000000;
const uint8_t R0x02_CALS_Bit = 0b00000100;
const uint8_t R0x02_CAL_ERR_Bit = 0b00001000;

const uint8_t R0x11_TS_Bit = 0b00000010;

const uint8_t R0x1B_PGACHPDIS_Bit = 0b00000001;      // Chopper disable bit. 1 = disabled TODO: Unused, consider deletion.
const uint8_t R0x1B_PGA_BUFFER_ENABLE = 0b00100000;  // Enable PGA output buffer. TODO: Unused, consider deletion.
const uint8_t R0x1B_PGA_LDO_MODE = 0b01000000;       // LDO mode 1 = "improved stability and lower DC gain, can accommodate ESR < 5 ohms (output capacitance)". TODO: Unused, consider deletion.

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
const uint8_t deviceClockDisable = 0b00100000;
const uint8_t deviceRowIntSet = 0b10100000;
const uint8_t deviceDimmingSet = 0b11101111;     // Full brightness
const uint8_t deviceDisplayOnSet = 0b10000001;   // Device ON, blinking OFF.
const uint8_t deviceDisplayOffSet = 0b10000000;  // Device OFF.

enum TareSwitchState {
    open,
    closed,
    held
};

enum TareSwitchProcessedState {
    processedState_none,
    momentaryReleased,
    longHoldEntered,
    longHold,
    longHoldExited
};

enum TimerDisplayAction {
    displayAction_none,
    acknowledgeLongHold,
    startPending,
    start,
    run
};

enum MassDisplayAction {
    showMass,
    showOverflow,
    tare
};

enum TimerDisplayAction computeTimerDisplayAction(enum TareSwitchProcessedState switchState, double mass) {
    static enum TareSwitchProcessedState previousSwitchState = processedState_none;
    static enum TimerDisplayAction previousState = displayAction_none;

    enum TimerDisplayAction action;

    static const double massStartThreshold = 2.0;

    if ((switchState == longHoldEntered)) {
        previousState = acknowledgeLongHold;
        action = acknowledgeLongHold;
    } else if (switchState == longHoldExited) {
        previousState = startPending;
        action = startPending;
    } else if (previousState == startPending && (mass < massStartThreshold)) {
        previousState = startPending;
        action = displayAction_none;
    } else if (previousState == startPending && (mass >= massStartThreshold)) {
        previousState = start;
        action = start;
    } else if (previousState == start) {
        previousState = run;
        action = run;
    } else if (previousState == run) {
        action = run;
    } else {
        action = displayAction_none;
    }

    return action;
}

enum MassDisplayAction computeMassDisplayAction(enum TareSwitchProcessedState switchState) {
    switch (switchState) {
        case momentaryReleased:
            return tare;
            break;

        case longHoldEntered:
            return showOverflow;
            break;

        case longHold:
            return showOverflow;
            break;

        case longHoldExited:
            return tare;
            break;

        default:
            return showMass;
    }
}

// Global vars
enum TareSwitchState global_tareSwitchState = open;
enum TareSwitchProcessedState global_tareSwitchProcessedState = processedState_none;
struct repeating_timer global_tareSwitchPollingTimer;

uint8_t display0DataBuffer[17];  // TODO: See if we can de-globalise this
uint8_t display1DataBuffer[17];  // TODO: See if we can de-globalise this

// Global constants
uint8_t* const display0DigitAddresses[] = {
    display0DataBuffer + 1,
    display0DataBuffer + 3,
    display0DataBuffer + 7,
    display0DataBuffer + 9,
    display1DataBuffer + 5 /* <- colon */
};

uint8_t* const display1DigitAddresses[] = {
    display1DataBuffer + 1,
    display1DataBuffer + 3,
    display1DataBuffer + 7,
    display1DataBuffer + 9,
    display1DataBuffer + 5 /* <- colon */
};

uint8_t* const addrDisplay0Colon = display0DataBuffer + 5;
uint8_t* const addrDisplay1Colon = display1DataBuffer + 5;

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

static bool tareSwitchDebounceTimerCallback(__unused struct repeating_timer* t) {
    static bool switchClosed = false;
    static bool switchHeld = false;

    static const uint8_t switchDebounceQueueThreshold = 0x7f;
    static uint8_t switchDebounceQueue = 0xFF;

    static const uint switchHoldThreshold = 100;
    static uint switchHoldAccumulator = 0;

    const bool switchPinLow = !gpio_get(TARE_PIN);  // Pin is normally electrically HIGH

    switchDebounceQueue = (switchDebounceQueue << 1) + (switchPinLow ? 1 : 0);

    if (switchDebounceQueue >= switchDebounceQueueThreshold) {
        switchClosed = true;
    } else if (switchDebounceQueue == 0) {
        switchClosed = false;
    }

    switchHoldAccumulator += (global_tareSwitchState == closed ? 1 : 0);

    if (switchClosed == true && switchHoldAccumulator >= switchHoldThreshold) {
        switchHoldAccumulator == switchHoldThreshold;
        switchHeld = true;
    } else if (switchClosed == false) {
        switchHoldAccumulator = 0;
        switchHeld = false;
    }

    if (switchClosed == false) {
        global_tareSwitchState = open;
    } else if (switchHeld == true) {
        global_tareSwitchState = held;
    } else {
        global_tareSwitchState = closed;
    }
}

static enum TareSwitchProcessedState processTareSwitchState() {
    static enum TareSwitchState previousSwitchState = open;

    const enum TareSwitchState switchState = global_tareSwitchState;  // Capture value

    enum TareSwitchProcessedState processedState;

    if (previousSwitchState != held && switchState == held) {
        processedState = longHoldEntered;
    } else if (previousSwitchState == held && switchState == held) {
        processedState = longHold;
    } else if (previousSwitchState == held && switchState != held) {
        processedState = longHoldExited;
    } else if (switchState == open && previousSwitchState == closed) {
        processedState = momentaryReleased;
    } else {
        processedState = processedState_none;
    }

    previousSwitchState = switchState;

    return processedState;
}

static void initI2C() {
    i2c_init(i2c_default, 100000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

static void initGPIO() {
    gpio_init(DRDY_PIN);
    gpio_set_dir(DRDY_PIN, GPIO_IN);

    gpio_init(TARE_PIN);
    gpio_set_dir(TARE_PIN, GPIO_IN);
    gpio_pull_up(TARE_PIN);

    // TODO: Hook up the scope and look at ripple, then peak-to-peak noise, then VSYS reading without, then with PS set HIGH.
    // gpio_init(PS_PIN);
    // gpio_set_dir(PS_PIN, GPIO_OUT);
    // gpio_put(PS_PIN, 1);
}

// This is the Pico's own internal ADC used to
// measure VSYS (ie, battery voltage), not
// the main ADC used to measure the load cell
static void initPicoADC() {
    adc_init();
    adc_gpio_init(VSYS_INPUT_PIN);
    adc_select_input(3);  // Select ADC input 3 (GPIO29), which is connected to VSYS
}

static float getVSYS_volts() {
    const float conversion_factor = 3.3f / (1 << 12);
    const uint32_t sampleCount = 1024;
    uint32_t tally = 0;
    for (int i = 0; i < sampleCount; i++) {
        tally += (uint32_t)adc_read();
    }

    return (float)(tally / sampleCount) * conversion_factor * 3;  // Multiply by three because GPIO29 gives Vsys / 3
}

static void initDisplays() {
    display0DataBuffer[0] = deviceDisplayAddressPointer;  // This is just zero, which is the HT16K33 display-driver 'write display' command
    i2c_write_blocking(i2c_default, display0I2CAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, display0I2CAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, display0I2CAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, display0I2CAddress, display0DataBuffer, 17, false);
    i2c_write_blocking(i2c_default, display0I2CAddress, &deviceDisplayOnSet, 1, false);

    display1DataBuffer[0] = deviceDisplayAddressPointer;  // This is just zero, which is the HT16K33 display-driver 'write display' command
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceRowIntSet, 1, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceDimmingSet, 1, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, display1DataBuffer, 17, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceDisplayOnSet, 1, false);
}

static void disableTimerDisplay() {
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceDisplayOffSet, 1, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceClockDisable, 1, false);
}

static void enableTimerDisplay() {
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceClockEnable, 1, false);
    i2c_write_blocking(i2c_default, display1I2CAddress, &deviceDisplayOnSet, 1, false);
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

static void readADCI2CBytes(uint8_t registerAddress, uint8_t* buffer, uint8_t length) {
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
    // Capture initial register state:
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
    while (!gpio_get(DRDY_PIN)) {
        sleep_us(1);
    }

    // Read the three ADC bytes
    uint8_t reading[3];
    readADCI2CBytes(R0x12_address, reading, 3);

    int32_t value = (int32_t)((reading[0] << 16) | (reading[1] << 8) | reading[2]);

    // Extend the sign into the topmost byte with a bitwise
    // left-shift followed by an arithmetic right-shift:
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

    /*
        Set device LDO output voltage and analogue input gain.

            Bits 5 to 3 of register 0x01 select LDO voltage.
            111 == 2.4V
            110 == 2.7V
            101 == 3.0V
            100 == 3.3V
            011 == 3.6
            010 == 3.9V
            001 == 4.2V
            000 == 4.5V

            Bits 2 to 0 of register 0x01 select device gain.
            111 == 128x
            110 == 64x
            101 == 32x
            100 == 16x
            011 == 8x
            010 == 4x
            001 == 2x
            000 == 1x
    */
    const uint8_t LDOVoltageMask = 0b00010000;  // LDO 3.9V
    writeADCI2CByte(R0x01_address, LDOVoltageMask);
    setADCGain128();

    // Ensure LDO mode bit is zero
    writeADCI2CByte(R0x1B_address, 0);

    // Disable ADC chopper, as recommended in datasheet; ADC output
    // is useless if it's left in its default enabled state.
    writeADCI2CByte(R0x15_address, 0b00110000);

    // Set sample rate (Rx02 register bits 6 to 0):
    // 111 = 320SPS
    // 011 = 80SPS
    // 010 = 40SPS
    // 001 = 20SPS
    // 000 = 10SPS
    writeADCI2CByte(R0x02_address, 0b00000000);  // 10SPS

    // Wait for LDO to stabilize
    sleep_ms(300);
}

int32_t averageADC(uint16_t samples, int32_t* peakToPeak) {
    int32_t tally = 0;
    int32_t min = INT32_MAX;
    int32_t max = INT32_MIN;
    for (int i = 0; i < samples; i++) {
        int32_t reading = readADC();
        tally += reading;
        if (reading < min) {
            min = reading;
        }
        if (reading > max) {
            max = reading;
        }
    }

    if (peakToPeak != NULL) {
        *peakToPeak = abs(max - min);
    }

    return tally / samples;
}

static void writeDisplay0DataBuffer() {
    i2c_write_blocking(i2c_default, display0I2CAddress, display0DataBuffer, 17, false);
}

static void writeDisplay1DataBuffer() {
    i2c_write_blocking(i2c_default, display1I2CAddress, display1DataBuffer, 17, false);
}

static void setDisplayOverflow(uint8_t* const* displayDigitAddresses) {
    *displayDigitAddresses[0] = digitPatternDash;
    *displayDigitAddresses[1] = digitPatternDash;
    *displayDigitAddresses[2] = digitPatternDash;
    *displayDigitAddresses[3] = digitPatternDash;
}

static void setTimerDisplayColon(bool on, uint8_t* const* displayDigitAddresses) {
    *displayDigitAddresses[4] = on ? colonPatternOn : colonPatternOff;
}

void beginTareSwitchPolling() {
    add_repeating_timer_ms(10, tareSwitchDebounceTimerCallback, NULL, &global_tareSwitchPollingTimer);
}

static void initialize() {
    stdio_init_all();
    pico_led_init();
    initGPIO();
    initPicoADC();
    initI2C();
    initDisplays();
    initADC();
    beginTareSwitchPolling();
}

// Retrieves nth digit from the right, eg:
// nthDigit(1, 5678) will return 8.
// nthDigit(4, 5678) will return 5.
static int nthDigit(const int n, int value) {
    if (value < 0) {
        value *= -1;
    }

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

static void clearDisplayDecimalPoint(uint8_t* const* displayDigitAddresses) {
    *displayDigitAddresses[0] = *displayDigitAddresses[0] & digitMaskDecimalPointOff;
    *displayDigitAddresses[1] = *displayDigitAddresses[1] & digitMaskDecimalPointOff;
    *displayDigitAddresses[2] = *displayDigitAddresses[2] & digitMaskDecimalPointOff;
    *displayDigitAddresses[3] = *displayDigitAddresses[3] & digitMaskDecimalPointOff;
}

static void setDisplayDecimalPoint(int position, uint8_t* const* displayDigitAddresses) {
    clearDisplayDecimalPoint(displayDigitAddresses);

    switch (position) {
        case 0:
            *displayDigitAddresses[0] = *displayDigitAddresses[0] | digitMaskDecimalPointOn;
            return;

        case 1:
            *displayDigitAddresses[1] = *displayDigitAddresses[1] | digitMaskDecimalPointOn;
            return;

        case 2:
            *displayDigitAddresses[2] = *displayDigitAddresses[2] | digitMaskDecimalPointOn;
            return;

        case 3:
            *displayDigitAddresses[3] = *displayDigitAddresses[3] | digitMaskDecimalPointOn;
            return;
    }
}

static void setDisplayInteger(int value, uint8_t* const* displayDigitAddresses) {
    if (value >= 0) {
        *displayDigitAddresses[0] = patternForDigit(nthDigit(4, value));
        *displayDigitAddresses[1] = patternForDigit(nthDigit(3, value));
        *displayDigitAddresses[2] = patternForDigit(nthDigit(2, value));
        *displayDigitAddresses[3] = patternForDigit(nthDigit(1, value));
    } else {
        *displayDigitAddresses[0] = digitPatternDash;
        *displayDigitAddresses[1] = patternForDigit(nthDigit(3, value));
        *displayDigitAddresses[2] = patternForDigit(nthDigit(2, value));
        *displayDigitAddresses[3] = patternForDigit(nthDigit(1, value));
    }
}

static int truncateToIntWithRounding(double value) {
    const int temp = (int)(value * 10);

    if (nthDigit(1, temp) > 4) {
        return (int)(value) + 1;
    }

    return (int)value;
}

static void setDisplayElapsedTime(int64_t microSeconds, uint8_t* const* displayDigitAddresses) {
    setTimerDisplayColon(true, displayDigitAddresses);
    const int seconds = microSeconds / 1000000;
    const int displaySeconds = seconds % 60;
    const int displayMinutes = seconds / 60;

    if (displayMinutes > 99) {
        setDisplayOverflow(displayDigitAddresses);
        return;
    }

    // Construct an integer representation showing minutes in leftmost two digits and seconds in rightmost two:
    const int displayInteger = (displayMinutes * 100) + displaySeconds;
    setDisplayInteger(displayInteger, displayDigitAddresses);
}

// TODO: Fix the '00.01' bug: setDisplayDouble(-0.009) incorrectly shows 00.01 on display
static void setDisplayDouble(double value, uint8_t* const* displayDigitAddresses) {
    if ((value >= 10000) || (value <= -1000)) {
        setDisplayOverflow(displayDigitAddresses);
        return;
    }

    if (value >= 0) {
        if (value < 10) {  // Results in display of, eg, 1.032
            setDisplayInteger(truncateToIntWithRounding(value * 1000), displayDigitAddresses);
            setDisplayDecimalPoint(0, displayDigitAddresses);
        } else if (value < 100) {  // Results in display of, eg, 10.32
            setDisplayInteger(truncateToIntWithRounding(value * 100), displayDigitAddresses);
            setDisplayDecimalPoint(1, displayDigitAddresses);
        } else if (value < 1000) {  // Results in display of, eg, 103.2
            setDisplayInteger(truncateToIntWithRounding(value * 10), displayDigitAddresses);
            setDisplayDecimalPoint(2, displayDigitAddresses);
        } else if (value >= 1000) {  // Results in display of, eg, 1032
            setDisplayInteger(truncateToIntWithRounding(value), displayDigitAddresses);
            clearDisplayDecimalPoint(displayDigitAddresses);
        }
    } else {
        // Negative numbers
        if (value > -10) {
            setDisplayInteger(truncateToIntWithRounding(value * 100), displayDigitAddresses);
            setDisplayDecimalPoint(1, displayDigitAddresses);
        } else if (value > -100) {
            setDisplayInteger(truncateToIntWithRounding(value * 10), displayDigitAddresses);
            setDisplayDecimalPoint(2, displayDigitAddresses);
        } else if (value > -1000) {
            setDisplayInteger(truncateToIntWithRounding(value), displayDigitAddresses);
            clearDisplayDecimalPoint(displayDigitAddresses);
        }
    }
}

double getMass(const int averagingCount, const int32_t zeroScaleReading, int32_t* peakToPeak) {
    static const double calibrationValue = 420.0;
    const int32_t adcReading = averageADC(averagingCount, peakToPeak);
    const double mass = (double)(adcReading - zeroScaleReading) / calibrationValue;
    return mass;
}

int main() {
    initialize();

    setDisplayOverflow(display0DigitAddresses);
    setDisplayOverflow(display1DigitAddresses);
    writeDisplay0DataBuffer();
    writeDisplay1DataBuffer();

    selectADCChannel(0);
    setADCGain128();
    calibrateADC();
    flushADC();

    const int averagingCount = 1;
    const int lpFilterCount = 10;
    const int sampleCount = 1;

    double lpSamples[lpFilterCount];
    int32_t peakToPeakResults[sampleCount];

    int32_t zeroScaleReading = averageADC(averagingCount + lpFilterCount, NULL);

    absolute_time_t timerStartTime; // Used for calculation of elapsed time on timer display

    disableTimerDisplay();

    while (true) {
        double mass = getMass(averagingCount, zeroScaleReading, NULL);

        const enum TareSwitchProcessedState tareSwitchState = processTareSwitchState();

        const enum MassDisplayAction massDisplayAction = computeMassDisplayAction(tareSwitchState);

        switch (massDisplayAction) {
            case showMass:
                for (int j = lpFilterCount - 1; j > 0; j--) {
                    lpSamples[j] = lpSamples[j - 1];
                }

                lpSamples[0] = mass;

                double lpFilteredReading = 0;
                for (int k = 0; k < lpFilterCount; k++) {
                    lpFilteredReading += (lpSamples[k] / lpFilterCount);
                }

                // When close to zero, clamp display reading to zeroo
                setDisplayDouble(((lpFilteredReading < 0.05) && (lpFilteredReading > -0.05)) ? 0.0 : lpFilteredReading, display0DigitAddresses);
                writeDisplay0DataBuffer();
                break;

            case showOverflow:
                setDisplayOverflow(display0DigitAddresses);
                writeDisplay0DataBuffer();
                break;

            case tare:
                setDisplayOverflow(display0DigitAddresses);
                writeDisplay0DataBuffer();
                sleep_ms(500);  // Wait for any movement to settle. TODO: Think about finessing this by reading ADC until oscillations (ie peak to peak result) die down
                zeroScaleReading = averageADC(averagingCount + lpFilterCount, NULL);

                // Clear out & override now-invalid sample history with zeroed mass reading:
                mass = getMass(averagingCount, zeroScaleReading, NULL);
                for (int l = 0; l < lpFilterCount; l++) {
                    lpSamples[l] = mass;
                }
                break;
        }

        const enum TimerDisplayAction timerDisplayAction = computeTimerDisplayAction(tareSwitchState, mass);

        switch (timerDisplayAction) {
            case displayAction_none:
                break;

            case acknowledgeLongHold:
                enableTimerDisplay();
                setDisplayOverflow(display1DigitAddresses);
                writeDisplay1DataBuffer();
                break;

            case startPending:
                setDisplayElapsedTime(0, display1DigitAddresses);
                writeDisplay1DataBuffer();
                break;

            case start:
                timerStartTime = get_absolute_time();
                break;

            case run:
                const int64_t elapsedTime = absolute_time_diff_us(timerStartTime, get_absolute_time());
                setDisplayElapsedTime(elapsedTime, display1DigitAddresses);
                writeDisplay1DataBuffer();
                break;
        }
    }

    /*
        // printf("Vsys is %.2f\n", getVSYS_volts());
        // setDisplayDouble((double)getVSYS_volts(), display1DigitAddresses);
        // writeDisplay1DataBuffer();
    */

    /*
    // double massReadings[sampleCount];
    // int32_t peakToPeakResult = 0;
    // const int32_t adcReading = averageADC(averagingCount, &peakToPeakResult);
    // peakToPeakResults[i] = peakToPeakResult;
    // massReadings[i] = mass;
*/

    /*
        // const absolute_time_t startTime = get_absolute_time();


        // Measurements done.

        const absolute_time_t endTime = get_absolute_time();
        const uint32_t startMillis = to_ms_since_boot(startTime);
        const uint32_t endMillis = to_ms_since_boot(endTime);

        double timeSeconds = (double)(endMillis - startMillis) / 1000.0;

        printf("Done. Took %.1fs\n\n", timeSeconds);

        setDisplay0Double(timeSeconds);
        writeDisplay0DataBuffer();

        int64_t peakToPeakTally = 0;
        int32_t peakToPeakMax = INT32_MIN;

        for (int i = 0; i < sampleCount; i++) {
            int32_t peakToPeakResult = peakToPeakResults[i];
            peakToPeakTally += peakToPeakResult;
            if (peakToPeakResult > peakToPeakMax) {
                peakToPeakMax = peakToPeakResult;
            }
        }
        int32_t peakToPeakAverage = (int32_t)(peakToPeakTally / sampleCount);

        printf("Peak to peak average: %i, max: %i\n", peakToPeakAverage, peakToPeakMax);

        // setDisplayInteger(peakToPeakAverage);
        // writeDisplayDataBuffer();

        for (int i = 0; i < sampleCount; i++) {
            printf("%.3f\n", massReadings[i]);
        }
            */
}
