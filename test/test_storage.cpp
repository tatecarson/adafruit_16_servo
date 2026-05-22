#include "mock/Arduino.h"
#include "mock/EEPROM.h"
#include <stdio.h>
#include <stdexcept>
#include <string>

EEPROMClass EEPROM;

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

int main() {
    printf("=== Storage Tests ===\n");
    RUN(mock_eeprom_read_write);
    printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
    return _tests_failed ? 1 : 0;
}
