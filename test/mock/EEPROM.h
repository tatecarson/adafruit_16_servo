#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

class EEPROMClass {
 public:
  static constexpr size_t SIZE = 8192;
  uint8_t buf[SIZE];
  EEPROMClass() { memset(buf, 0xFF, SIZE); }
  uint8_t read(int addr) const { return (addr >= 0 && (size_t)addr < SIZE) ? buf[addr] : 0xFF; }
  void write(int addr, uint8_t v) { if (addr >= 0 && (size_t)addr < SIZE) buf[addr] = v; }
  void update(int addr, uint8_t v) { if (read(addr) != v) write(addr, v); }
  size_t length() const { return SIZE; }
};

extern EEPROMClass EEPROM;
inline void EEPROM_reset() { memset(EEPROM.buf, 0xFF, EEPROMClass::SIZE); }
