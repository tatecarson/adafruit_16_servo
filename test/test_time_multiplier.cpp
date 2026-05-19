// Arduino mocks must come first
#include "mock/Arduino.h"

#include <stdio.h>
#include <stdexcept>
#include <string>

// --- Minimal test framework ---
static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;

#define ASSERT_EQ(a, b) do { \
    if ((long long)(a) != (long long)(b)) { \
        char _buf[256]; \
        snprintf(_buf, sizeof(_buf), "FAIL at line %d: got %lld, expected %lld", \
                 __LINE__, (long long)(a), (long long)(b)); \
        throw std::runtime_error(_buf); \
    } \
} while(0)

#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)

#define RUN(name) do { \
    _tests_run++; \
    printf("  " #name " ... "); fflush(stdout); \
    try { test_##name(); _tests_passed++; printf("PASS\n"); } \
    catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } \
} while(0)

// Pull in the real source (all globals and functions defined here)
#include "../adafruit_16_servo.ino"

// --- Test helpers ---
static void reset_sequence() {
    initServoDefaults();
    sequenceActive = false;
    speedSeqActive = false;
    waveActive = false;
    lastTriggeredKeyframe = 0;
    lastTriggeredSpeedFrame = 0;
    sequenceStartTime = 0;
    speedSeqStartTime = 0;
    _mock_millis = 0;
}

// Test sequences: PROGMEM is a no-op in mock, so these are plain arrays
static const Keyframe kfAt1000ms[] PROGMEM = {
    {0, 900, 1000, 500},
    {SEQUENCE_END_MARKER_SERVO, 0, 2000, 0}
};

static const Keyframe kfAt0ms[] PROGMEM = {
    {0, 900, 0, 500},
    {SEQUENCE_END_MARKER_SERVO, 0, 1000, 0}
};

// --- Tests ---

// Multiplier defaults to 1 (no scaling)
void test_default_timescale_is_1() {
    ASSERT_EQ(timeMultiplier, 1);
}

// With multiplier=1, keyframe triggers at its authored time
void test_multiplier_1_triggers_at_authored_time() {
    reset_sequence();
    timeMultiplier = 1;
    currentSequence = kfAt1000ms;
    currentSequenceLength = 2;
    sequenceActive = true;

    _mock_millis = 999;
    updateSequence();
    ASSERT_FALSE(servoState[0].moving);

    _mock_millis = 1000;
    updateSequence();
    ASSERT_TRUE(servoState[0].moving);
    ASSERT_EQ(servoState[0].moveDurationMs, 500);  // 500 * 1 = 500
}

// With multiplier=60, keyframe trigger is delayed 60x
void test_multiplier_60_delays_trigger() {
    reset_sequence();
    timeMultiplier = 60;
    currentSequence = kfAt1000ms;
    currentSequenceLength = 2;
    sequenceActive = true;

    _mock_millis = 59999;   // 1000 * 60 - 1
    updateSequence();
    ASSERT_FALSE(servoState[0].moving);

    _mock_millis = 60000;   // 1000 * 60 = exactly on time
    updateSequence();
    ASSERT_TRUE(servoState[0].moving);
}

// With multiplier=60, move duration is scaled 60x
void test_multiplier_60_scales_duration() {
    reset_sequence();
    timeMultiplier = 60;
    currentSequence = kfAt0ms;  // triggers at t=0 immediately
    currentSequenceLength = 2;
    sequenceActive = true;

    updateSequence();
    ASSERT_TRUE(servoState[0].moving);
    ASSERT_EQ(servoState[0].moveDurationMs, 30000);  // 500 * 60 = 30000
}

// Large multiplier exercises the uint32_t requirement (500 * 200 = 100000 > uint16_t max)
void test_large_multiplier_requires_uint32_duration() {
    reset_sequence();
    timeMultiplier = 200;
    currentSequence = kfAt0ms;
    currentSequenceLength = 2;
    sequenceActive = true;

    updateSequence();
    ASSERT_TRUE(servoState[0].moving);
    ASSERT_EQ(servoState[0].moveDurationMs, 100000);  // 500 * 200 = 100000
}

// TIMESCALE command sets the multiplier
void test_timescale_command_sets_multiplier() {
    timeMultiplier = 1;
    char cmd[] = "TIMESCALE 60";
    processCommand(cmd);
    ASSERT_EQ(timeMultiplier, 60);
}

// TIMESCALE 1 resets scaling
void test_timescale_command_resets_to_1() {
    timeMultiplier = 60;
    char cmd[] = "TIMESCALE 1";
    processCommand(cmd);
    ASSERT_EQ(timeMultiplier, 1);
}

int main() {
    printf("=== Time Multiplier Tests ===\n");
    RUN(default_timescale_is_1);
    RUN(multiplier_1_triggers_at_authored_time);
    RUN(multiplier_60_delays_trigger);
    RUN(multiplier_60_scales_duration);
    RUN(large_multiplier_requires_uint32_duration);
    RUN(timescale_command_sets_multiplier);
    RUN(timescale_command_resets_to_1);
    printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
    return _tests_failed > 0 ? 1 : 0;
}
