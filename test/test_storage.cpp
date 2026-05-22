#include "mock/Arduino.h"
#include "mock/EEPROM.h"
#include <stdio.h>
#include <stdexcept>
#include <string>

EEPROMClass EEPROM;

#include "../adafruit_16_servo/storage_crc.h"
#include "../adafruit_16_servo/storage.h"

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _buf[256]; snprintf(_buf, sizeof(_buf), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_buf);} } while(0)
#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)
#define RUN(name) do { _tests_run++; printf("  " #name " ... "); fflush(stdout); try { test_##name(); _tests_passed++; printf("PASS\n"); } catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } } while(0)

static void test_mock_eeprom_read_write() {
    EEPROM_reset();
    EEPROM.write(0, 0xAB);
    EEPROM.write(1, 0xCD);
    ASSERT_EQ(EEPROM.read(0), 0xAB);
    ASSERT_EQ(EEPROM.read(1), 0xCD);
    ASSERT_EQ(EEPROM.length(), 8192);
}

static void test_crc_empty() {
    ASSERT_EQ(crc16_ccitt(nullptr, 0), 0xFFFF);
}
static void test_crc_known_vector() {
    // CRC16-CCITT (poly 0x1021, init 0xFFFF) of "123456789" is 0x29B1.
    const char* s = "123456789";
    ASSERT_EQ(crc16_ccitt((const uint8_t*)s, 9), 0x29B1);
}
static void test_crc_detects_single_bit_flip() {
    uint8_t a[16]; for (int i = 0; i < 16; i++) a[i] = i;
    uint16_t c1 = crc16_ccitt(a, 16);
    a[7] ^= 0x01;
    uint16_t c2 = crc16_ccitt(a, 16);
    ASSERT_TRUE(c1 != c2);
}

static void test_storage_init_on_fresh_device() {
    EEPROM_reset();  // all 0xFF
    storageInit();
    // Fresh device: no active slot, no boardId.
    ASSERT_EQ(storageBoardId(), 0);
    ASSERT_FALSE(storageHasActive());
    ASSERT_FALSE(storageHasPrevious());
}
static void test_storage_set_and_get_board_id() {
    EEPROM_reset();
    storageInit();
    ASSERT_TRUE(storageSetBoardId(2));
    ASSERT_EQ(storageBoardId(), 2);
    // Re-init should preserve.
    storageInit();
    ASSERT_EQ(storageBoardId(), 2);
}
static void test_storage_rejects_invalid_board_id() {
    EEPROM_reset();
    storageInit();
    ASSERT_FALSE(storageSetBoardId(0));
    ASSERT_FALSE(storageSetBoardId(4));
    ASSERT_FALSE(storageSetBoardId(255));
    ASSERT_EQ(storageBoardId(), 0);
}

static void seedSlot(int slotIdx, const uint8_t* payload, uint16_t len) {
    int off = (slotIdx == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;
    EEPROM.write(off + 0, (uint8_t)(len >> 8));
    EEPROM.write(off + 1, (uint8_t)(len & 0xFF));
    // CRC over the length bytes + payload
    uint8_t tmp[4082]; tmp[0] = (uint8_t)(len >> 8); tmp[1] = (uint8_t)(len & 0xFF);
    for (uint16_t i = 0; i < len; i++) tmp[2 + i] = payload[i];
    uint16_t crc = crc16_ccitt(tmp, 2 + len);
    EEPROM.write(off + 2, (uint8_t)(crc >> 8));
    EEPROM.write(off + 3, (uint8_t)(crc & 0xFF));
    for (uint16_t i = 0; i < len; i++) EEPROM.write(off + 4 + i, payload[i]);
}

static void test_read_empty_when_no_active() {
    EEPROM_reset(); storageInit();
    uint8_t buf[64];
    int n = storageReadActive(buf, sizeof(buf));
    ASSERT_EQ(n, -1);  // no active slot
}
static void test_read_active_when_valid() {
    EEPROM_reset(); storageInit();
    const char* p = "hello";
    seedSlot(0, (const uint8_t*)p, 5);
    EEPROM.update(STORAGE_OFF_ACTIVE, 0);
    uint8_t buf[64];
    int n = storageReadActive(buf, sizeof(buf));
    ASSERT_EQ(n, 5);
    ASSERT_EQ(buf[0], 'h'); ASSERT_EQ(buf[4], 'o');
}
static void test_read_rejects_corrupted_crc() {
    EEPROM_reset(); storageInit();
    const char* p = "hello";
    seedSlot(0, (const uint8_t*)p, 5);
    EEPROM.update(STORAGE_OFF_ACTIVE, 0);
    // Flip a bit in the payload — CRC must fail.
    EEPROM.write(STORAGE_SLOT_0_OFF + 4 + 2, 'X');
    uint8_t buf[64];
    int n = storageReadActive(buf, sizeof(buf));
    ASSERT_EQ(n, -1);
}
static void test_read_rejects_too_large_length() {
    EEPROM_reset(); storageInit();
    EEPROM.write(STORAGE_SLOT_0_OFF + 0, 0xFF);  // length = 0xFFFF, way over max
    EEPROM.write(STORAGE_SLOT_0_OFF + 1, 0xFF);
    EEPROM.update(STORAGE_OFF_ACTIVE, 0);
    uint8_t buf[64];
    ASSERT_EQ(storageReadActive(buf, sizeof(buf)), -1);
}

static void test_first_write_goes_to_slot_0() {
    EEPROM_reset(); storageInit();
    const char* p = "library-v1";
    ASSERT_TRUE(storageWriteSlot((const uint8_t*)p, 10));
    uint8_t buf[64];
    ASSERT_EQ(storageReadActive(buf, sizeof(buf)), 10);
    ASSERT_EQ(buf[0], 'l');
    // Active pointer should be 0.
    ASSERT_EQ(EEPROM.read(STORAGE_OFF_ACTIVE), 0);
}
static void test_second_write_goes_to_slot_1() {
    EEPROM_reset(); storageInit();
    storageWriteSlot((const uint8_t*)"v1", 2);
    storageWriteSlot((const uint8_t*)"v2-bigger", 9);
    ASSERT_EQ(EEPROM.read(STORAGE_OFF_ACTIVE), 1);
    uint8_t buf[64];
    ASSERT_EQ(storageReadActive(buf, sizeof(buf)), 9);
    ASSERT_EQ(buf[0], 'v'); ASSERT_EQ(buf[1], '2');
    // Previous should still be valid (v1).
    ASSERT_TRUE(storageHasPrevious());
    ASSERT_EQ(storageReadPrevious(buf, sizeof(buf)), 2);
    ASSERT_EQ(buf[1], '1');
}
static void test_write_rejects_too_large() {
    EEPROM_reset(); storageInit();
    static uint8_t big[STORAGE_PAYLOAD_MAX + 1] = {0};
    ASSERT_FALSE(storageWriteSlot(big, STORAGE_PAYLOAD_MAX + 1));
    ASSERT_FALSE(storageHasActive());
}
static void test_write_then_crash_before_pointer_flip_leaves_previous_intact() {
    EEPROM_reset(); storageInit();
    storageWriteSlot((const uint8_t*)"first-good", 10);
    uint8_t activeBefore = EEPROM.read(STORAGE_OFF_ACTIVE);
    // Manually corrupt the *inactive* slot to simulate a partial write that
    // never got the active-pointer flip.
    int inactive = (activeBefore == 0) ? STORAGE_SLOT_1_OFF : STORAGE_SLOT_0_OFF;
    for (int i = 0; i < 16; i++) EEPROM.write(inactive + i, 0xAA);
    // Active still points at first slot.
    uint8_t buf[64];
    ASSERT_EQ(storageReadActive(buf, sizeof(buf)), 10);
    ASSERT_EQ(buf[0], 'f');
}

int main() {
    printf("=== Storage Tests ===\n");
    RUN(mock_eeprom_read_write);
    RUN(crc_empty);
    RUN(crc_known_vector);
    RUN(crc_detects_single_bit_flip);
    RUN(storage_init_on_fresh_device);
    RUN(storage_set_and_get_board_id);
    RUN(storage_rejects_invalid_board_id);
    RUN(read_empty_when_no_active);
    RUN(read_active_when_valid);
    RUN(read_rejects_corrupted_crc);
    RUN(read_rejects_too_large_length);
    RUN(first_write_goes_to_slot_0);
    RUN(second_write_goes_to_slot_1);
    RUN(write_rejects_too_large);
    RUN(write_then_crash_before_pointer_flip_leaves_previous_intact);
    printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
    return _tests_failed ? 1 : 0;
}
