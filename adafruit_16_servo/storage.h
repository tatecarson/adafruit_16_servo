#pragma once
#include <stdint.h>
#include <stddef.h>
#include <EEPROM.h>
#include "storage_crc.h"

#define STORAGE_MAGIC_0    0x53  // 'S'
#define STORAGE_MAGIC_1    0x31  // '1' = schemaVersion 1
#define STORAGE_TOTAL      8192
#define STORAGE_HDR_LEN    8
#define STORAGE_SLOT_LEN   4084
#define STORAGE_PAYLOAD_MAX 4080
#define STORAGE_SLOT_0_OFF 8
#define STORAGE_SLOT_1_OFF 4092
#define STORAGE_OFF_MAGIC0 0
#define STORAGE_OFF_MAGIC1 1
#define STORAGE_OFF_ACTIVE 2
#define STORAGE_OFF_BOARDID 3

// Initialize storage. If header magic is missing, write it.
// Does not touch slot data.
inline void storageInit() {
    uint8_t m0 = EEPROM.read(STORAGE_OFF_MAGIC0);
    uint8_t m1 = EEPROM.read(STORAGE_OFF_MAGIC1);
    if (m0 != STORAGE_MAGIC_0 || m1 != STORAGE_MAGIC_1) {
        EEPROM.update(STORAGE_OFF_MAGIC0, STORAGE_MAGIC_0);
        EEPROM.update(STORAGE_OFF_MAGIC1, STORAGE_MAGIC_1);
        EEPROM.update(STORAGE_OFF_ACTIVE, 0xFF);     // no active slot
        EEPROM.update(STORAGE_OFF_BOARDID, 0);       // unassigned
    }
}

inline uint8_t storageBoardId() {
    uint8_t v = EEPROM.read(STORAGE_OFF_BOARDID);
    return (v >= 1 && v <= 3) ? v : 0;
}

inline bool storageSetBoardId(uint8_t id) {
    if (id < 1 || id > 3) return false;
    EEPROM.update(STORAGE_OFF_BOARDID, id);
    return true;
}

// Stubs to be replaced in Task 4. Returning false here means the Task 3
// tests can compile and observe a fresh device as having no slots.
inline bool storageHasActive() { return false; }
inline bool storageHasPrevious() { return false; }
