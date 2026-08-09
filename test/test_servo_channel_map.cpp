// Host tests for the per-board servo channel map.
//
// Board 1's PCA9685 header 1 is dead — a known-good servo works on header 2
// and not on 1, and a resolder did not bring it back. Rather than renumber the
// wands everywhere (tracks, motions, sequences, bakes all address B1.S0/S1/S2)
// the firmware redirects the last write: logical servo 1 drives header 2, and
// logical servo 2 drives header 3. Header 1 is never written.
//
// This is hardware-specific and temporary. It is tested because the numbers
// are the whole content of it — an off-by-one here drives the wrong wand, and
// the only way to notice on the bench is that the wrong thing moves.
//
// Build: make -C test channelmap

#include "mock/Arduino.h"

#include <stdio.h>
#include <stdexcept>
#include <string.h>

#include "../adafruit_16_servo/servo_runtime.h"

// servo_setup.h reads the board id through storageInit()'s accessor. The map
// is a pure function of (boardId, logical), so the test supplies the id
// directly rather than standing up storage.
static uint8_t _boardId = 0;
uint8_t storageBoardId() { return _boardId; }

#include "../adafruit_16_servo/servo_setup.h"

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _buf[256]; snprintf(_buf, sizeof(_buf), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_buf);} } while(0)

static void run(const char* name, void (*fn)()) {
  _tests_run++;
  try { fn(); printf("  PASS  %s\n", name); _tests_passed++; }
  catch (const std::exception& e) { printf("  FAIL: %s\n    %s\n", name, e.what()); _tests_failed++; }
}

// --- board 1: the remap ----------------------------------------------------

static void test_wand_one_stays_on_header_zero() {
  ASSERT_EQ(servoPhysicalChannel(1, 0), 0);
}

static void test_wand_two_moves_to_header_two() {
  ASSERT_EQ(servoPhysicalChannel(1, 1), 2);
}

static void test_wand_three_moves_to_header_three() {
  ASSERT_EQ(servoPhysicalChannel(1, 2), 3);
}

static void test_no_wand_lands_on_the_dead_header() {
  // The three wands specifically. Logical 3 is parked on the dead header on
  // purpose — see test_the_freed_channel_parks_on_the_dead_header — so this
  // asks the narrower question that actually matters: can a wand command
  // reach a header that drives nothing?
  for (uint8_t wand = 0; wand < 3; wand++) {
    if (servoPhysicalChannel(1, wand) == BOARD1_DEAD_HEADER) {
      throw std::runtime_error("a wand still maps onto the dead header");
    }
  }
}

// --- every other board is untouched ----------------------------------------
// The map is one board's hardware fault, not a new layer everything pays for.

static void test_curtain_is_identity() {
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) ASSERT_EQ(servoPhysicalChannel(3, ch), ch);
}

static void test_field_is_identity() {
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) ASSERT_EQ(servoPhysicalChannel(2, ch), ch);
}

static void test_unprogrammed_board_is_identity() {
  // boardId 0 is "never set". It must not inherit board 1's fault.
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) ASSERT_EQ(servoPhysicalChannel(0, ch), ch);
}

// --- the map stays inside the driver ---------------------------------------

static void test_channels_beyond_the_wands_are_untouched_on_board_one() {
  // Only the wand channels and the freed one move; 4..15 are not the wands'
  // to touch, and one of the low headers is now carrying wand III.
  for (uint8_t ch = 4; ch < NUM_SERVOS; ch++) ASSERT_EQ(servoPhysicalChannel(1, ch), ch);
}

static void test_the_freed_channel_parks_on_the_dead_header() {
  // Logical 3 drives nothing. It must not land on wand III's header, where a
  // stray S3 from the terminal would move a wand.
  ASSERT_EQ(servoPhysicalChannel(1, 3), BOARD1_DEAD_HEADER);
}

static void test_every_mapping_is_a_real_channel() {
  for (uint8_t board = 0; board <= 3; board++) {
    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
      uint8_t phys = servoPhysicalChannel(board, ch);
      if (phys >= NUM_SERVOS) throw std::runtime_error("mapped past the end of the driver");
    }
  }
}

static void test_board_one_map_is_injective() {
  // Two wands sharing a header would silently drive one servo from two tracks.
  bool seen[NUM_SERVOS] = { false };
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    uint8_t phys = servoPhysicalChannel(1, ch);
    if (seen[phys]) throw std::runtime_error("two logical servos share a header");
    seen[phys] = true;
  }
}

int main() {
  printf("=== Servo channel map ===\n");
  run("wand I stays on header 0", test_wand_one_stays_on_header_zero);
  run("wand II moves to header 2", test_wand_two_moves_to_header_two);
  run("wand III moves to header 3", test_wand_three_moves_to_header_three);
  run("no wand lands on the dead header", test_no_wand_lands_on_the_dead_header);
  run("the curtain is unmapped", test_curtain_is_identity);
  run("the field is unmapped", test_field_is_identity);
  run("an unprogrammed board is unmapped", test_unprogrammed_board_is_identity);
  run("channels past the wands are unmapped", test_channels_beyond_the_wands_are_untouched_on_board_one);
  run("the freed channel parks on the dead header", test_the_freed_channel_parks_on_the_dead_header);
  run("every mapping is a real channel", test_every_mapping_is_a_real_channel);
  run("no two wands share a header", test_board_one_map_is_injective);

  printf("\n%d run, %d passed, %d failed\n", _tests_run, _tests_passed, _tests_failed);
  return _tests_failed ? 1 : 0;
}
