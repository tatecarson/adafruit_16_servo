#include "mock/Arduino.h"
#include "mock/EEPROM.h"
#include "mock/Adafruit_PWMServoDriver.h"

#include <stdio.h>
#include <stdexcept>
#include <string.h>
#include <vector>

EEPROMClass EEPROM;

#include "../adafruit_16_servo/storage_crc.h"
#include "../adafruit_16_servo/servo_runtime.h"
#include "../adafruit_16_servo/dc_motor.h"
#include "../adafruit_16_servo/storage.h"

Adafruit_PWMServoDriver pwm;
ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];
MotorState motorState;

MotionRuntime motionRuntime;
SequenceRuntime sequenceRunner;
SetlistRuntime setlistScheduler;

void cancelMotionPlayback() {}

inline void cancelSetlistPlayback();

// Same dispatch rig as test_setlist_scheduler: record commands and simulate a
// sequence starting on "RUN <id>" so startActiveSetlist's first fire is
// observable here.
struct DispatchedCmd { unsigned long ts; std::string cmd; };
static std::vector<DispatchedCmd> dispatched;
void processCommand(char* cmd) { (void)cmd; }
void dispatchCommand(const char* cmd, bool fromNetwork) {
  (void)fromNetwork;
  dispatched.push_back({ _mock_millis, std::string(cmd) });
  if (strncmp(cmd, "RUN ", 4) == 0 && strcmp(cmd, "RUN AUTO") != 0) {
    sequenceRunner.active = (strstr(cmd, "ghost") == nullptr);
  } else if (strcmp(cmd, "STOP") == 0) {
    sequenceRunner.active = false;
    cancelSetlistPlayback();
  }
}

#include "../adafruit_16_servo/sequence_engine.h"
#include "../adafruit_16_servo/setlist_scheduler.h"
#include "../adafruit_16_servo/gallery_mode.h"

GalleryGraceState galleryGrace;

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _b[256]; snprintf(_b, sizeof(_b), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_b);} } while(0)
#define ASSERT_STREQ(a, b) do { if (std::string(a) != std::string(b)) { char _b[512]; snprintf(_b, sizeof(_b), "FAIL at line %d: got '%s', expected '%s'", __LINE__, std::string(a).c_str(), std::string(b).c_str()); throw std::runtime_error(_b);} } while(0)
#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)
#define RUN(name) do { _tests_run++; printf("  " #name " ... "); fflush(stdout); try { test_##name(); _tests_passed++; printf("PASS\n"); } catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } } while(0)

// Active setlist "evening" (ordered) fires "RUN arc" first. graceMs default
// 10000; a second blob overrides it to 3000.
static const char* kBlob =
  "{\"schemaVersion\":1,\"motions\":[],"
  "\"sequences\":["
    "{\"id\":\"arc\",\"name\":\"Arc\",\"steps\":[{\"cmd\":\"S0 90\",\"durationMs\":100,\"target\":\"all\"}]}"
  "],"
  "\"setlists\":["
    "{\"id\":\"evening\",\"name\":\"Evening\",\"mode\":\"ordered\",\"entries\":["
      "{\"seqId\":\"arc\",\"repeat\":1,\"gapMs\":0}"
    "]}"
  "],"
  "\"activeSetlistId\":\"evening\","
  "\"schedulerConfig\":{\"leaderBoardId\":1,\"graceMs\":10000}}";

static const char* kBlobGrace3000 =
  "{\"schemaVersion\":1,\"motions\":[],"
  "\"sequences\":["
    "{\"id\":\"arc\",\"name\":\"Arc\",\"steps\":[{\"cmd\":\"S0 90\",\"durationMs\":100,\"target\":\"all\"}]}"
  "],"
  "\"setlists\":["
    "{\"id\":\"evening\",\"name\":\"Evening\",\"mode\":\"ordered\",\"entries\":["
      "{\"seqId\":\"arc\",\"repeat\":1,\"gapMs\":0}"
    "]}"
  "],"
  "\"activeSetlistId\":\"evening\","
  "\"schedulerConfig\":{\"leaderBoardId\":1,\"graceMs\":3000}}";

static void reset_state(const char* blob = nullptr) {
  EEPROM_reset();
  storageInit();
  storageSetBoardId(1);
  memset(&sequenceRunner, 0, sizeof(sequenceRunner));
  memset(&setlistScheduler, 0, sizeof(setlistScheduler));
  memset(&galleryGrace, 0, sizeof(galleryGrace));
  dispatched.clear();
  _mock_millis = 0;
  if (blob) ASSERT_TRUE(storageWriteSlot((const uint8_t*)blob, strlen(blob)));
}

// ---- persistent flag ---------------------------------------------

static void test_gallery_mode_default_off_on_fresh_device() {
  reset_state();
  ASSERT_FALSE(storageGalleryMode());
}

static void test_gallery_mode_set_and_get() {
  reset_state();
  ASSERT_TRUE(storageSetGalleryMode(true));
  ASSERT_TRUE(storageGalleryMode());
  storageInit();                      // re-init must preserve the flag
  ASSERT_TRUE(storageGalleryMode());
  ASSERT_TRUE(storageSetGalleryMode(false));
  ASSERT_FALSE(storageGalleryMode());
}

// ---- schedulerConfig.graceMs parse -------------------------------

static void test_grace_ms_parse_default_and_custom() {
  ASSERT_EQ(schedulerGraceMs((const uint8_t*)kBlob, strlen(kBlob)), 10000);
  ASSERT_EQ(schedulerGraceMs((const uint8_t*)kBlobGrace3000, strlen(kBlobGrace3000)), 3000);
  const char* noGrace =
    "{\"schemaVersion\":1,\"motions\":[],\"sequences\":[],\"setlists\":[],"
    "\"activeSetlistId\":null,\"schedulerConfig\":{\"leaderBoardId\":1}}";
  ASSERT_EQ(schedulerGraceMs((const uint8_t*)noGrace, strlen(noGrace)), 10000);
}

// ---- grace state machine -----------------------------------------

static void test_grace_does_not_arm_when_gallery_off() {
  reset_state(kBlob);                 // gallery flag still off
  galleryGraceArm();
  ASSERT_FALSE(galleryGrace.pending);
  _mock_millis += 20000;
  updateGalleryGrace();
  ASSERT_EQ(dispatched.size(), 0);    // never auto-runs
}

static void test_grace_arms_with_config_window_when_on() {
  reset_state(kBlobGrace3000);
  storageSetGalleryMode(true);
  galleryGraceArm();
  ASSERT_TRUE(galleryGrace.pending);
  ASSERT_EQ(galleryGrace.graceMs, 3000);
}

static void test_grace_defaults_window_when_unbaked() {
  reset_state();                      // no baked library
  storageSetGalleryMode(true);
  galleryGraceArm();
  ASSERT_TRUE(galleryGrace.pending);
  ASSERT_EQ(galleryGrace.graceMs, GALLERY_GRACE_DEFAULT_MS);
}

static void test_grace_fires_run_auto_after_window() {
  reset_state(kBlobGrace3000);
  storageSetGalleryMode(true);
  galleryGraceArm();

  _mock_millis += 2999;               // just before the window closes
  updateGalleryGrace();
  ASSERT_EQ(dispatched.size(), 0);
  ASSERT_TRUE(galleryGrace.pending);

  _mock_millis += 1;                  // window elapsed
  updateGalleryGrace();
  ASSERT_FALSE(galleryGrace.pending);
  ASSERT_EQ(dispatched.size(), 1);
  ASSERT_STREQ(dispatched[0].cmd, "RUN arc");
}

static void test_grace_canceled_by_command_skips_auto_run() {
  reset_state(kBlobGrace3000);
  storageSetGalleryMode(true);
  galleryGraceArm();
  ASSERT_TRUE(galleryGrace.pending);

  galleryGraceCancel();               // browser/operator intervened
  ASSERT_FALSE(galleryGrace.pending);

  _mock_millis += 10000;              // window would have elapsed
  updateGalleryGrace();
  ASSERT_EQ(dispatched.size(), 0);
}

static void test_grace_arm_is_one_shot() {
  reset_state(kBlobGrace3000);
  storageSetGalleryMode(true);
  galleryGraceArm();
  galleryGraceCancel();               // pending cleared (e.g. a command)
  galleryGraceArm();                  // a later WiFi reconnect must not re-arm
  ASSERT_FALSE(galleryGrace.pending);
  _mock_millis += 10000;
  updateGalleryGrace();
  ASSERT_EQ(dispatched.size(), 0);
}

int main() {
  printf("=== Gallery Mode Tests ===\n");
  RUN(gallery_mode_default_off_on_fresh_device);
  RUN(gallery_mode_set_and_get);
  RUN(grace_ms_parse_default_and_custom);
  RUN(grace_does_not_arm_when_gallery_off);
  RUN(grace_arms_with_config_window_when_on);
  RUN(grace_defaults_window_when_unbaked);
  RUN(grace_fires_run_auto_after_window);
  RUN(grace_canceled_by_command_skips_auto_run);
  RUN(grace_arm_is_one_shot);
  printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
  return _tests_failed ? 1 : 0;
}
