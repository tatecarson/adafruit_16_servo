#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define NUM_SERVOS 16
#define SERVO_FREQ 50

struct ServoConfig {
  uint16_t minPulse;
  uint16_t maxPulse;
  uint16_t stopPulse;
  uint16_t totalDegrees;
  bool allowRelease;
  uint16_t upDegrees;
  uint16_t downDegrees;
  bool reverseDir;
};

struct ServoState {
  uint16_t posPulse;
  bool stopped;

  uint16_t targetPulse;
  uint16_t startPulse;
  unsigned long moveStartMs;
  uint32_t moveDurationMs;
  bool moving;
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

struct ProgramSequenceStep {
  uint8_t sequenceNumber;
  uint16_t repeatCount;
};

struct SequenceProgramDefinition {
  const ProgramSequenceStep* positionSteps;
  uint8_t positionLength;
  const ProgramSequenceStep* speedSteps;
  uint8_t speedLength;
};

#define MOTION_ID_MAX_LEN 32
#define MOTION_MAX_TRACKS 17
#define MOTION_MAX_KEYFRAMES 64
#define MOTION_TRACK_NONE 0
#define MOTION_TRACK_SERVO 1
#define MOTION_TRACK_DC 2
#define MOTION_VALUE_UNSET 32767

struct MotionKeyframe {
  uint32_t atMs;
  int16_t value;
};

struct MotionTrack {
  uint8_t kind;
  uint8_t channel;
  uint8_t firstKeyframe;
  uint8_t keyframeCount;
  uint8_t segmentIndex;
  int16_t lastAppliedValue;
};

struct MotionRuntime {
  char id[MOTION_ID_MAX_LEN + 1];
  uint32_t durationMs;
  uint8_t trackCount;
  uint8_t keyframeCount;
  MotionTrack tracks[MOTION_MAX_TRACKS];
  MotionKeyframe keyframes[MOTION_MAX_KEYFRAMES];
  unsigned long startMs;
  bool active;
};

// Browser-baked Sequence (schema v1 §3): an ordered list of commands
// with millis()-based timing. Each step dispatches its cmd through
// the existing processCommand() parser, so any RUN <id> step composes
// PLAY/SPLAY/ROTATE/MOTION/MOVE/Sn/STOP/TIMESCALE for free.
#define SEQ_ID_MAX_LEN 32
#define SEQ_CMD_MAX_LEN 96
#define SEQ_MAX_STEPS 16

struct SequenceStep {
  char cmd[SEQ_CMD_MAX_LEN + 1];
  uint32_t durationMs;
  uint8_t target;  // 0 = all boards; 1..3 = specific boardId; 255 = malformed (skip)
};

struct SequenceRuntime {
  char id[SEQ_ID_MAX_LEN + 1];
  uint8_t stepCount;
  uint8_t currentStep;
  unsigned long stepStartMs;
  bool active;
  bool loop;
  SequenceStep steps[SEQ_MAX_STEPS];
};

extern Adafruit_PWMServoDriver pwm;

extern ServoConfig servoConfig[NUM_SERVOS];
extern ServoState servoState[NUM_SERVOS];

// WAVE globals removed (servo-dz7). See animation_engine.h.

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

extern bool programActive;
extern bool programLoop;
extern const SequenceProgramDefinition* currentProgram;
extern bool programPositionDone;
extern bool programSpeedDone;
extern uint8_t currentProgramPositionStepIndex;
extern uint16_t currentProgramPositionIteration;
extern uint8_t currentProgramSpeedStepIndex;
extern uint16_t currentProgramSpeedIteration;
extern MotionRuntime motionRuntime;
extern SequenceRuntime sequenceRunner;

void initServoDefaults();

uint16_t degreesToPulse(uint8_t servo, uint16_t degrees);
uint16_t sequenceDegreesToPulse(uint8_t servo, uint16_t degrees);
uint16_t percentToDegrees(uint8_t servo, uint8_t percent);
uint16_t upPercentToDegrees(uint8_t servo, uint8_t percentUp);
float easeInOutCubic(float t);
uint16_t lerpEased(uint16_t start, uint16_t end, float progress);

void setServoPulse(uint8_t servo, uint16_t pulse);
void setServoDegrees(uint8_t servo, uint16_t degrees);
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint32_t duration);
void moveServoDegrees(uint8_t servo, uint16_t degrees, uint32_t duration);
void moveSequenceDegrees(uint8_t servo, uint16_t degrees, uint32_t duration);
void stopActivePatterns();
void clearServoStop(uint8_t servo);
void stopServoNow(uint8_t servo);
void setServoPercent(uint8_t servo, uint8_t percent);
void moveServoPercent(uint8_t servo, uint8_t percent, uint32_t duration);
void setServoPercentUp(uint8_t servo, uint8_t percentUp);
void moveServoPercentUp(uint8_t servo, uint8_t percentUp, uint32_t duration);
void setAllProtectedWinchesPercent(bool percentUp, uint8_t percent);
void moveAllProtectedWinchesPercent(bool percentUp, uint8_t percent, uint32_t duration);
void setTestPulse(uint16_t pulse);
bool startPositionSequence(uint8_t seqNum, bool loop, bool announce);
bool startSpeedSequence(uint8_t seqNum, bool loop, bool announce);
bool startSequenceProgram(uint8_t programNum, bool loop);
bool startMotionFromStorage(const char* motionId, bool announce);
void cancelMotionPlayback();
void updateMotion();
bool startSequenceFromStorage(const char* sequenceId, bool loop, bool announce);
void cancelSequencePlayback();
void updateSequenceRunner();
void updateSequenceProgram();
void servoOff(uint8_t servo);
void releaseServo(uint8_t servo);

void sweepServo(uint8_t servo);
void setCalibration(uint8_t servo, uint16_t minVal, uint16_t maxVal);
void showStatus();
void showHelp();
void processCommand(char* cmd);
void updateAnimations();
void updateSpeedRamps();
void updateSequence();
void updateSpeedSequence();
