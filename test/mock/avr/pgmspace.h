#pragma once
#include <string.h>

#define PROGMEM
#define F(x) (x)
#define PSTR(x) (x)

inline void* memcpy_P(void* dst, const void* src, size_t n) { return memcpy(dst, src, n); }
inline unsigned char pgm_read_byte(const void* addr) { return *(const unsigned char*)addr; }
