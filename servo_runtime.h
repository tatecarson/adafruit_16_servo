#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define NUM_SERVOS 16
#define SERVO_FREQ 50

struct ServoConfig {
  uint16_t minPulse;
  uint16_t maxPulse;
  bool continuous;
  uint16_t stopPulse;
  uint16_t totalDegrees;
  bool allowRelease;
  uint16_t upDegrees;    // degree position for fully-up (default 0)
  uint16_t downDegrees;  // degree position for fully-down (0 = use totalDegrees)
  bool reverseDir;       // true = higher degrees is physically UP
};

struct ServoState {
  uint16_t posPulse;
  bool stopped;

  uint16_t targetPulse;
  uint16_t startPulse;
  unsigned long moveStartMs;
  uint32_t moveDurationMs;
  bool moving;

  int8_t targetSpeed;
  int8_t startSpeed;
  unsigned long speedRampStartMs;
  uint32_t speedRampDurationMs;
  bool speedRamping;
};

struct Keyframe {
  uint8_t servo;
  uint16_t degrees;
  uint16_t time;
  uint16_t duration;
};

struct SpeedFrame {
  uint8_t servo;
  int8_t speed;
  uint16_t time;
  uint16_t rampMs;
};

extern Adafruit_PWMServoDriver pwm;

extern ServoConfig servoConfig[NUM_SERVOS];
extern ServoState servoState[NUM_SERVOS];

extern bool waveActive;
extern uint8_t waveStartServo;
extern uint8_t waveEndServo;
extern uint16_t waveSpeed;
extern uint8_t wavePhaseOffset;
extern uint16_t waveAmplitude;
extern uint16_t waveCenter;
extern unsigned long waveStartTime;

extern uint16_t timeMultiplier;

extern bool sequenceActive;
extern bool sequenceLoop;
extern const Keyframe* currentSequence;
extern uint8_t currentSequenceLength;
extern unsigned long sequenceStartTime;
extern uint8_t lastTriggeredKeyframe;

extern bool speedSeqActive;
extern bool speedSeqLoop;
extern const SpeedFrame* currentSpeedSeq;
extern uint8_t currentSpeedSeqLength;
extern unsigned long speedSeqStartTime;
extern uint8_t lastTriggeredSpeedFrame;

void initServoDefaults();

uint16_t degreesToPulse(uint8_t servo, uint16_t degrees);
uint16_t percentToDegrees(uint8_t servo, uint8_t percent);
uint16_t upPercentToDegrees(uint8_t servo, uint8_t percentUp);
uint16_t speedToPulse(uint8_t servo, int8_t speed);
float easeInOutCubic(float t);
uint16_t lerpEased(uint16_t start, uint16_t end, float progress);

void setServoPulse(uint8_t servo, uint16_t pulse);
void setServoDegrees(uint8_t servo, uint16_t degrees);
void setServoSpeed(uint8_t servo, int8_t speed);
void rampServoSpeed(uint8_t servo, int8_t targetSpeed, uint32_t rampMs);
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint32_t duration);
void moveServoDegrees(uint8_t servo, uint16_t degrees, uint32_t duration);
void stopActivePatterns();
void clearServoStop(uint8_t servo);
void stopServoNow(uint8_t servo);
void setServoPercent(uint8_t servo, uint8_t percent);
void moveServoPercent(uint8_t servo, uint8_t percent, uint32_t duration);
void setServoPercentUp(uint8_t servo, uint8_t percentUp);
void moveServoPercentUp(uint8_t servo, uint8_t percentUp, uint32_t duration);
void setAllProtectedWinchesPercent(bool percentUp, uint8_t percent);
void moveAllProtectedWinchesPercent(bool percentUp, uint8_t percent, uint32_t duration);
void servoOff(uint8_t servo);
void releaseServo(uint8_t servo);

void sweepServo(uint8_t servo);
void setCalibration(uint8_t servo, uint16_t minVal, uint16_t maxVal);
void showStatus();
void showHelp();
void processCommand(char* cmd);
void updateAnimations();
void updateSpeedRamps();
void updateWave();
void updateSequence();
void updateSpeedSequence();
