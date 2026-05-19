#pragma once
#include <stdint.h>

class Adafruit_PWMServoDriver {
public:
    Adafruit_PWMServoDriver() {}
    void begin() {}
    void setOscillatorFrequency(uint32_t) {}
    void setPWMFreq(float) {}
    void setPWM(uint8_t, uint8_t, uint16_t) {}
};
