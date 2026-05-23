#pragma once
#include <stdint.h>
#include <stddef.h>

// One CRC16-CCITT step. `crc` carries state; returns updated crc.
inline uint16_t crc16_step(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xor-out).
// Standard "XModem" CRC; well-known check value for "123456789" is 0x29B1.
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) crc = crc16_step(crc, data[i]);
    return crc;
}
