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

// Shared 4 KB scratch buffer in BSS. Used by storageHasPrevious() for its
// CRC scan and by transport-layer receive paths that need a same-sized
// staging area. Single-threaded contract: callers MUST NOT use this buffer
// across calls that could themselves invoke storageHasPrevious() (or any
// other consumer of this buffer). Holding a pointer into it across a yield
// to other code is unsafe. Defined inline so multiple translation units
// share the same instance via the linker's COMDAT semantics on supported
// toolchains; on the Renesas R4 toolchain this is plain ODR (the header is
// included into one .ino and one .cpp at most).
inline uint8_t* storageScratchBuffer() {
    static uint8_t scratch[STORAGE_PAYLOAD_MAX];
    return scratch;
}

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

// Internal: read raw slot, validate CRC + length. Returns payload length, or -1.
inline int _storageReadSlot(int slotIdx, uint8_t* out, int maxLen) {
    int off = (slotIdx == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;
    uint16_t len = ((uint16_t)EEPROM.read(off + 0) << 8) | EEPROM.read(off + 1);
    if (len == 0 || len > STORAGE_PAYLOAD_MAX) return -1;
    if ((int)len > maxLen) return -1;
    uint16_t storedCrc = ((uint16_t)EEPROM.read(off + 2) << 8) | EEPROM.read(off + 3);
    // Streaming CRC over (length-bytes || payload). Read payload into out
    // simultaneously to avoid a second EEPROM pass.
    uint16_t crc = 0xFFFF;
    auto step = [&](uint8_t b){ crc = crc16_step(crc, b); };
    step(EEPROM.read(off + 0)); step(EEPROM.read(off + 1));
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = EEPROM.read(off + 4 + i);
        out[i] = b;
        step(b);
    }
    if (crc != storedCrc) return -1;
    return (int)len;
}

inline bool storageHasActive() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    return (a == 0 || a == 1);
}

// Returns the byte count stored in the active slot's length header, or 0 if
// no active slot or the header reports an out-of-range length. Does not
// validate the slot's CRC — call storageHasActive() if you also need that.
inline int storageActiveBytesUsed() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return 0;
    int off = (a == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;
    int n = ((int)EEPROM.read(off) << 8) | EEPROM.read(off + 1);
    if (n < 0 || n > (int)STORAGE_PAYLOAD_MAX) return 0;
    return n;
}

inline bool storageHasPrevious() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return false;
    int prev = (a == 0) ? 1 : 0;
    // Uses the shared 4 KB BSS scratch buffer (see storageScratchBuffer). Not
    // reentrancy-safe — do not call storageHasPrevious() from an ISR or while
    // another consumer of the shared scratch (e.g., handleSequencesPost's
    // receive loop) is still using it. Single-threaded loop() use is fine.
    uint8_t* tmp = storageScratchBuffer();
    return _storageReadSlot(prev, tmp, STORAGE_PAYLOAD_MAX) >= 0;
}

inline int storageReadActive(uint8_t* out, int maxLen) {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return -1;
    return _storageReadSlot((int)a, out, maxLen);
}
inline int storageReadPrevious(uint8_t* out, int maxLen) {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return -1;
    return _storageReadSlot(a == 0 ? 1 : 0, out, maxLen);
}

// Returns true on success. Writes to the inactive slot first, then atomically
// flips the active-slot pointer. A power loss anywhere before the pointer flip
// leaves the previous active slot intact.
inline bool storageWriteSlot(const uint8_t* payload, uint16_t len) {
    if (len == 0 || len > STORAGE_PAYLOAD_MAX) return false;
    uint8_t cur = EEPROM.read(STORAGE_OFF_ACTIVE);
    int target = (cur == 0) ? 1 : 0;  // 0xFF (fresh) -> target slot 0
    int off = (target == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;

    // Compute CRC over length bytes + payload.
    uint8_t lenHi = (uint8_t)(len >> 8);
    uint8_t lenLo = (uint8_t)(len & 0xFF);
    uint16_t crc = 0xFFFF;
    auto step = [&](uint8_t b){ crc = crc16_step(crc, b); };
    step(lenHi); step(lenLo);
    for (uint16_t i = 0; i < len; i++) step(payload[i]);

    // Write length, payload, then CRC. CRC last so an interrupted write fails
    // verification on the next boot.
    EEPROM.write(off + 0, lenHi);
    EEPROM.write(off + 1, lenLo);
    for (uint16_t i = 0; i < len; i++) EEPROM.write(off + 4 + i, payload[i]);
    EEPROM.write(off + 2, (uint8_t)(crc >> 8));
    EEPROM.write(off + 3, (uint8_t)(crc & 0xFF));

    // Atomic pointer flip - single byte.
    EEPROM.update(STORAGE_OFF_ACTIVE, (uint8_t)target);
    return true;
}

// Atomically flip the active-slot pointer back to the previous slot - but only
// if the previous slot's CRC validates. No data is moved; this is a one-byte
// pointer flip.
inline bool storageRollback() {
    if (!storageHasPrevious()) return false;
    uint8_t cur = EEPROM.read(STORAGE_OFF_ACTIVE);
    EEPROM.update(STORAGE_OFF_ACTIVE, (uint8_t)(cur == 0 ? 1 : 0));
    return true;
}
