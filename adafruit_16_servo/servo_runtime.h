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
  // Signed degree offset applied by degreesToPulse so a horn installed
  // N degrees off-center can be trimmed without re-shimming the
  // hardware. Populated from EEPROM at boot via servo_calibration.h;
  // 0 = no trim (default). Whole degrees to match totalDegrees.
  int16_t offsetDeg;
};

struct ServoState {
  uint16_t posPulse;
  bool stopped;

  uint16_t targetPulse;
  uint16_t startPulse;
  unsigned long moveStartMs;
  uint32_t moveDurationMs;
  bool moving;
  // false (default): easeInOutCubic — slow→fast→slow, right shape for SWEEP
  //                  / sequence playback / one-off gestures.
  // true: linear progress — constant velocity within the segment, the
  //       shape DMOVE / UMOVE / motion-editor playback need so multi-
  //       segment motions don't look like they're accelerating then
  //       decelerating at every keyframe (servo-79q).
  bool linearMove;
};

// Legacy numbered PLAY/SPLAY/RUN-n sequence types removed in servo-voc
// (reclaim OTA flash; schema-v1 Motions/Sequences/Setlists are the only
// playback path now). See docs/PROGRESS.md.

#define MOTION_ID_MAX_LEN 32
#define MOTION_MAX_TRACKS 17
#define MOTION_MAX_KEYFRAMES 64
#define MOTION_TRACK_NONE 0
#define MOTION_TRACK_SERVO 1
#define MOTION_TRACK_DC 2
#define MOTION_DC_SPEED_MIN -50
#define MOTION_DC_SPEED_MAX 50
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

// ---- Setlist scheduler (servo-dos, schema §4) --------------------
// RUN AUTO runs the active Setlist forever: pick the next entry per mode,
// play its sequence `repeat` times, dwell `gapMs`, repeat. Only the leader
// board runs the scheduler; followers ride the existing RUN/STOP command
// mirror. Sized for the UNO R4's RAM budget.
#define SETLIST_ID_MAX_LEN 32
#define SETLIST_MAX_ENTRIES 12
#define SETLIST_PHASE_PLAYING 0
#define SETLIST_PHASE_GAP 1

struct SetlistEntry {
  char     seqId[SEQ_ID_MAX_LEN + 1];
  uint16_t repeat;   // >= 1
  uint32_t gapMs;
  uint16_t weight;   // >= 1 (shuffle only; ignored in ordered)
};

struct SetlistRuntime {
  char     id[SETLIST_ID_MAX_LEN + 1];
  uint8_t  entryCount;
  SetlistEntry entries[SETLIST_MAX_ENTRIES];
  bool     shuffle;          // mode == "shuffle"
  uint8_t  minGapEntries;
  uint32_t rngState;         // xorshift32 PRNG state
  bool     active;
  uint8_t  currentEntry;
  uint16_t playsDone;        // repeats completed for currentEntry
  uint8_t  phase;            // SETLIST_PHASE_PLAYING | SETLIST_PHASE_GAP
  unsigned long gapStartMs;
  uint8_t  recent[SETLIST_MAX_ENTRIES];  // ring buffer of recent picks
  uint8_t  recentCount;
  uint8_t  recentHead;
};

extern Adafruit_PWMServoDriver pwm;

extern ServoConfig servoConfig[NUM_SERVOS];
extern ServoState servoState[NUM_SERVOS];

// WAVE globals removed (servo-dz7). Legacy PLAY/SPLAY/RUN-n + TIMESCALE
// (timeMultiplier) state removed in servo-voc.

extern MotionRuntime motionRuntime;
extern SequenceRuntime sequenceRunner;
extern SetlistRuntime setlistScheduler;

void initServoDefaults();

uint16_t degreesToPulse(uint8_t servo, uint16_t degrees);
uint16_t percentToDegrees(uint8_t servo, uint8_t percent);
uint16_t upPercentToDegrees(uint8_t servo, uint8_t percentUp);
float easeInOutCubic(float t);
uint16_t lerpEased(uint16_t start, uint16_t end, float progress);

void setServoPulse(uint8_t servo, uint16_t pulse);
void setServoDegrees(uint8_t servo, uint16_t degrees);
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint32_t duration, bool linear = false);
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
bool startMotionFromStorage(const char* motionId, bool announce);
void cancelMotionPlayback();
void updateMotion();
bool startSequenceFromStorage(const char* sequenceId, bool loop, bool announce);
void cancelSequencePlayback();
void updateSequenceRunner();
void servoOff(uint8_t servo);
void releaseServo(uint8_t servo);

void sweepServo(uint8_t servo);
void setCalibration(uint8_t servo, uint16_t minVal, uint16_t maxVal);
void showStatus();
void showHelp();
void processCommand(char* cmd);
void updateAnimations();
