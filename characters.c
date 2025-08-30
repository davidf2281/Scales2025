#include "pico/stdlib.h"

static uint8_t mirror(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// UPPERCASE A–Z (5×8, right-aligned in code, bottom-aligned)
// Scanline order inverted: top row -> buffer[7], …, bottom row -> buffer[0].
// Each scanline is wrapped in mirror().

static void fillUpperABuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00011111);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperBBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001111);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillUpperCBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00010000);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperDBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010010);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010010);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperEBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00011111);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00011111);
}

static void fillUpperFBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00011111);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00010000);
}

static void fillUpperGBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00010000);
    buffer[2] = mirror(0b00010011);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillUpperHBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00011111);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperIBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00000100);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperJBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000011);
    buffer[5] = mirror(0b00000001);
    buffer[4] = mirror(0b00000001);
    buffer[3] = mirror(0b00000001);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperKBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010010);
    buffer[4] = mirror(0b00010100);
    buffer[3] = mirror(0b00011000);
    buffer[2] = mirror(0b00010100);
    buffer[1] = mirror(0b00010010);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperLBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010000);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00010000);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00011111);
}

static void fillUpperMBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00011011);
    buffer[4] = mirror(0b00010101);
    buffer[3] = mirror(0b00010101);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperNBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00011001);
    buffer[4] = mirror(0b00010101);
    buffer[3] = mirror(0b00010011);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperOBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperPBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001111);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00010000);
}

static void fillUpperQBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001110);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010101);
    buffer[1] = mirror(0b00010010);
    buffer[0] = mirror(0b00001101);
}

static void fillUpperRBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001111);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00010100);
    buffer[1] = mirror(0b00010010);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperSBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001111);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00001110);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00000001);
    buffer[0] = mirror(0b00001111);
}

static void fillUpperTBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00011111);
    buffer[5] = mirror(0b00000100);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00000100);
}

static void fillUpperUBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillUpperVBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001010);
    buffer[1] = mirror(0b00001010);
    buffer[0] = mirror(0b00000100);
}

static void fillUpperWBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010101);
    buffer[2] = mirror(0b00010101);
    buffer[1] = mirror(0b00011011);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperXBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00001010);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00001010);
    buffer[0] = mirror(0b00010001);
}

static void fillUpperYBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00001010);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00000100);
}

static void fillUpperZBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00011111);
    buffer[5] = mirror(0b00000001);
    buffer[4] = mirror(0b00000010);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00001000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00011111);
}

// LOWERCASE a–z (5×8, right-aligned in code, bottom-aligned)
// Scanline order inverted; each scanline wrapped in mirror().

static void fillLowerABuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00000000);
    buffer[4] = mirror(0b00001110);
    buffer[3] = mirror(0b00000001);
    buffer[2] = mirror(0b00001111);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillLowerBBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010000);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00001111);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillLowerCBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00000000);
    buffer[4] = mirror(0b00001110);
    buffer[3] = mirror(0b00010000);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerDBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000001);
    buffer[5] = mirror(0b00000001);
    buffer[4] = mirror(0b00001111);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillLowerEBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00000000);
    buffer[4] = mirror(0b00001110);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00011111);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerFBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000110);
    buffer[5] = mirror(0b00001000);
    buffer[4] = mirror(0b00001000);
    buffer[3] = mirror(0b00001110);
    buffer[2] = mirror(0b00001000);
    buffer[1] = mirror(0b00001000);
    buffer[0] = mirror(0b00001000);
}

static void fillLowerGBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001111);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001111);
    buffer[1] = mirror(0b00000001);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerHBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010000);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00001111);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillLowerIBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000100);  // dot
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001100);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerJBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000010);  // dot
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00000110);
    buffer[4] = mirror(0b00000010);
    buffer[3] = mirror(0b00000010);
    buffer[2] = mirror(0b00000010);
    buffer[1] = mirror(0b00010010);
    buffer[0] = mirror(0b00000110);
}

static void fillLowerKBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00010000);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00010010);
    buffer[3] = mirror(0b00010100);
    buffer[2] = mirror(0b00011000);
    buffer[1] = mirror(0b00010100);
    buffer[0] = mirror(0b00010010);
}

static void fillLowerLBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001100);
    buffer[5] = mirror(0b00000100);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerMBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00011010);
    buffer[4] = mirror(0b00010101);
    buffer[3] = mirror(0b00010101);
    buffer[2] = mirror(0b00010101);
    buffer[1] = mirror(0b00010101);
    buffer[0] = mirror(0b00010101);
}

static void fillLowerNBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001111);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00010001);
}

static void fillLowerOBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001110);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerPBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001111);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001111);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00010000);
}

static void fillLowerQBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001111);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001111);
    buffer[1] = mirror(0b00000001);
    buffer[0] = mirror(0b00000001);
}

static void fillLowerRBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001011);
    buffer[4] = mirror(0b00001100);
    buffer[3] = mirror(0b00010000);
    buffer[2] = mirror(0b00010000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00010000);
}

static void fillLowerSBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00001111);
    buffer[4] = mirror(0b00010000);
    buffer[3] = mirror(0b00001110);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00000001);
    buffer[0] = mirror(0b00001111);
}

static void fillLowerTBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00001000);
    buffer[5] = mirror(0b00001000);
    buffer[4] = mirror(0b00001110);
    buffer[3] = mirror(0b00001000);
    buffer[2] = mirror(0b00001000);
    buffer[1] = mirror(0b00001000);
    buffer[0] = mirror(0b00000110);
}

static void fillLowerUBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001111);
}

static void fillLowerVBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001010);
    buffer[1] = mirror(0b00001010);
    buffer[0] = mirror(0b00000100);
}

static void fillLowerWBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010101);
    buffer[2] = mirror(0b00010101);
    buffer[1] = mirror(0b00011011);
    buffer[0] = mirror(0b00010001);
}

static void fillLowerXBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00001010);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00001010);
    buffer[0] = mirror(0b00010001);
}

static void fillLowerYBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00001111);
    buffer[1] = mirror(0b00000001);
    buffer[0] = mirror(0b00001110);
}

static void fillLowerZBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00011111);
    buffer[4] = mirror(0b00000010);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00001000);
    buffer[0] = mirror(0b00011111);
}

// DIGITS 0–9 and '-' (5×8, right-aligned in code, bottom-aligned)
// Scanline order inverted; each scanline wrapped in mirror().

static void fillDigit0Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit1Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000100);
    buffer[6] = mirror(0b00001100);
    buffer[5] = mirror(0b00000100);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00000100);
    buffer[1] = mirror(0b00000100);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit2Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00000001);
    buffer[4] = mirror(0b00000010);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00001000);
    buffer[1] = mirror(0b00010000);
    buffer[0] = mirror(0b00011111);
}

static void fillDigit3Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00000001);
    buffer[4] = mirror(0b00000110);
    buffer[3] = mirror(0b00000001);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit4Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000010);
    buffer[6] = mirror(0b00000110);
    buffer[5] = mirror(0b00001010);
    buffer[4] = mirror(0b00010010);
    buffer[3] = mirror(0b00011111);
    buffer[2] = mirror(0b00000010);
    buffer[1] = mirror(0b00000010);
    buffer[0] = mirror(0b00000010);
}

static void fillDigit5Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00011111);
    buffer[6] = mirror(0b00010000);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00001111);
    buffer[3] = mirror(0b00000001);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit6Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010000);
    buffer[4] = mirror(0b00001111);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit7Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00011111);
    buffer[6] = mirror(0b00000001);
    buffer[5] = mirror(0b00000010);
    buffer[4] = mirror(0b00000100);
    buffer[3] = mirror(0b00000100);
    buffer[2] = mirror(0b00001000);
    buffer[1] = mirror(0b00001000);
    buffer[0] = mirror(0b00001000);
}

static void fillDigit8Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00001110);
    buffer[3] = mirror(0b00010001);
    buffer[2] = mirror(0b00010001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillDigit9Buffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00001110);
    buffer[6] = mirror(0b00010001);
    buffer[5] = mirror(0b00010001);
    buffer[4] = mirror(0b00010001);
    buffer[3] = mirror(0b00001111);
    buffer[2] = mirror(0b00000001);
    buffer[1] = mirror(0b00010001);
    buffer[0] = mirror(0b00001110);
}

static void fillCharacterMinusBuffer(uint8_t *buffer) {
    buffer[7] = mirror(0b00000000);
    buffer[6] = mirror(0b00000000);
    buffer[5] = mirror(0b00000000);
    buffer[4] = mirror(0b00011111);
    buffer[3] = mirror(0b00000000);
    buffer[2] = mirror(0b00000000);
    buffer[1] = mirror(0b00000000);
    buffer[0] = mirror(0b00000000);
}

static void fillDigitBuffer(int digit, uint8_t *buffer) {
    switch (digit) {
        case 0:
            fillDigit0Buffer(buffer);
            break;
        case 1:
            fillDigit1Buffer(buffer);
            break;
        case 2:
            fillDigit2Buffer(buffer);
            break;
        case 3:
            fillDigit3Buffer(buffer);
            break;
        case 4:
            fillDigit4Buffer(buffer);
            break;
        case 5:
            fillDigit5Buffer(buffer);
            break;
        case 6:
            fillDigit6Buffer(buffer);
            break;
        case 7:
            fillDigit7Buffer(buffer);
            break;
        case 8:
            fillDigit8Buffer(buffer);
            break;
        case 9:
            fillDigit9Buffer(buffer);
            break;
    }
}

// Non-digit characters, including '-' and A–Z/a–z.
static void fillCharBuffer(char ch, uint8_t *buffer) {
    switch (ch) {
        case '-':
            fillCharacterMinusBuffer(buffer);
            break;

        // Uppercase A–Z
        case 'A':
            fillUpperABuffer(buffer);
            break;
        case 'B':
            fillUpperBBuffer(buffer);
            break;
        case 'C':
            fillUpperCBuffer(buffer);
            break;
        case 'D':
            fillUpperDBuffer(buffer);
            break;
        case 'E':
            fillUpperEBuffer(buffer);
            break;
        case 'F':
            fillUpperFBuffer(buffer);
            break;
        case 'G':
            fillUpperGBuffer(buffer);
            break;
        case 'H':
            fillUpperHBuffer(buffer);
            break;
        case 'I':
            fillUpperIBuffer(buffer);
            break;
        case 'J':
            fillUpperJBuffer(buffer);
            break;
        case 'K':
            fillUpperKBuffer(buffer);
            break;
        case 'L':
            fillUpperLBuffer(buffer);
            break;
        case 'M':
            fillUpperMBuffer(buffer);
            break;
        case 'N':
            fillUpperNBuffer(buffer);
            break;
        case 'O':
            fillUpperOBuffer(buffer);
            break;
        case 'P':
            fillUpperPBuffer(buffer);
            break;
        case 'Q':
            fillUpperQBuffer(buffer);
            break;
        case 'R':
            fillUpperRBuffer(buffer);
            break;
        case 'S':
            fillUpperSBuffer(buffer);
            break;
        case 'T':
            fillUpperTBuffer(buffer);
            break;
        case 'U':
            fillUpperUBuffer(buffer);
            break;
        case 'V':
            fillUpperVBuffer(buffer);
            break;
        case 'W':
            fillUpperWBuffer(buffer);
            break;
        case 'X':
            fillUpperXBuffer(buffer);
            break;
        case 'Y':
            fillUpperYBuffer(buffer);
            break;
        case 'Z':
            fillUpperZBuffer(buffer);
            break;

        // Lowercase a–z (function names use uppercase letter)
        case 'a':
            fillLowerABuffer(buffer);
            break;
        case 'b':
            fillLowerBBuffer(buffer);
            break;
        case 'c':
            fillLowerCBuffer(buffer);
            break;
        case 'd':
            fillLowerDBuffer(buffer);
            break;
        case 'e':
            fillLowerEBuffer(buffer);
            break;
        case 'f':
            fillLowerFBuffer(buffer);
            break;
        case 'g':
            fillLowerGBuffer(buffer);
            break;
        case 'h':
            fillLowerHBuffer(buffer);
            break;
        case 'i':
            fillLowerIBuffer(buffer);
            break;
        case 'j':
            fillLowerJBuffer(buffer);
            break;
        case 'k':
            fillLowerKBuffer(buffer);
            break;
        case 'l':
            fillLowerLBuffer(buffer);
            break;
        case 'm':
            fillLowerMBuffer(buffer);
            break;
        case 'n':
            fillLowerNBuffer(buffer);
            break;
        case 'o':
            fillLowerOBuffer(buffer);
            break;
        case 'p':
            fillLowerPBuffer(buffer);
            break;
        case 'q':
            fillLowerQBuffer(buffer);
            break;
        case 'r':
            fillLowerRBuffer(buffer);
            break;
        case 's':
            fillLowerSBuffer(buffer);
            break;
        case 't':
            fillLowerTBuffer(buffer);
            break;
        case 'u':
            fillLowerUBuffer(buffer);
            break;
        case 'v':
            fillLowerVBuffer(buffer);
            break;
        case 'w':
            fillLowerWBuffer(buffer);
            break;
        case 'x':
            fillLowerXBuffer(buffer);
            break;
        case 'y':
            fillLowerYBuffer(buffer);
            break;
        case 'z':
            fillLowerZBuffer(buffer);
            break;

        default:
            for (int i = 0; i < 8; ++i) buffer[i] = 0;
            break;
    }
}
