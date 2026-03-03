#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <avr/pgmspace.h>

typedef unsigned char byte;
typedef bool boolean;

#define PI 3.14159265358979f

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

// Controllable clock for tests
inline unsigned long _mock_millis = 0;
inline unsigned long millis() { return _mock_millis; }
inline void delay(unsigned long) {}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct MockSerial {
    void begin(long) {}
    void println() {}
    void print(const char* s) { (void)s; }
    void println(const char* s) { (void)s; }
    template<typename T> void print(T) {}
    template<typename T> void println(T) {}
    int available() { return 0; }
    int read() { return -1; }
};
inline MockSerial Serial;
