#pragma once
#include <stdint.h>
#include <stddef.h>

// CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xor-out).
// Standard "XModem" CRC; well-known check value for "123456789" is 0x29B1.
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
