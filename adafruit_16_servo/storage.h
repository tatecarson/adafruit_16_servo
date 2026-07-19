#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "storage_crc.h"

#define STORAGE_MAGIC_0    0x53  // 'S'
#define STORAGE_MAGIC_1    0x31  // '1' = schemaVersion 1
#define STORAGE_TOTAL      8192
#define STORAGE_HDR_LEN    8
#define STORAGE_SLOT_LEN   4084
#define STORAGE_DUAL_PAYLOAD_MAX 4080
#define STORAGE_LARGE_PAYLOAD_MAX 6000
#define STORAGE_PAYLOAD_MAX STORAGE_LARGE_PAYLOAD_MAX
#define STORAGE_SLOT_0_OFF 8
#define STORAGE_SLOT_1_OFF 4092
#define STORAGE_LARGE_OFF  8
#define STORAGE_ACTIVE_LARGE 2
#define STORAGE_OFF_MAGIC0 0
#define STORAGE_OFF_MAGIC1 1
#define STORAGE_OFF_ACTIVE 2
#define STORAGE_OFF_BOARDID 3
#define STORAGE_OFF_GALLERY 4   // gallery_mode boot flag (servo-4gl): 1 = on

// Shared rollback-slot scratch buffer in BSS. Large mode leases a temporary
// heap buffer instead: keeping a 6 KB buffer permanently in BSS would collide
// with the UNO R4 core's reserved heap/stack regions. Single-threaded contract:
// callers MUST NOT use this buffer
// across calls that could themselves invoke storageHasPrevious() (or any
// other consumer of this buffer). Holding a pointer into it across a yield
// to other code is unsafe. Defined inline so multiple translation units
// share the same instance via the linker's COMDAT semantics on supported
// toolchains; on the Renesas R4 toolchain this is plain ODR (the header is
// included into one .ino and one .cpp at most).
inline uint8_t* storageScratchBuffer() {
    static uint8_t scratch[STORAGE_DUAL_PAYLOAD_MAX];
    return scratch;
}

class StorageBufferLease {
public:
    uint8_t* data;
    int capacity;
    bool heapAllocated;

    explicit StorageBufferLease(int required)
      : data(nullptr), capacity(required), heapAllocated(false) {
        if (required <= STORAGE_DUAL_PAYLOAD_MAX) {
            data = storageScratchBuffer();
            capacity = STORAGE_DUAL_PAYLOAD_MAX;
        } else if (required <= STORAGE_LARGE_PAYLOAD_MAX) {
            data = (uint8_t*)malloc((size_t)required);
            heapAllocated = data != nullptr;
        }
    }

    ~StorageBufferLease() {
        if (heapAllocated) free(data);
    }

    StorageBufferLease(const StorageBufferLease&) = delete;
    StorageBufferLease& operator=(const StorageBufferLease&) = delete;
};

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
        EEPROM.update(STORAGE_OFF_GALLERY, 0);       // gallery mode off
    }
}

// Gallery mode boot flag (servo-4gl). When on, the device auto-runs the active
// setlist after a boot grace period. Only the byte value 1 means on, so a
// device whose header predates this flag (offset still 0xFF) reads as off —
// the safe default for an unattended auto-run feature.
inline bool storageGalleryMode() {
    return EEPROM.read(STORAGE_OFF_GALLERY) == 1;
}

inline bool storageSetGalleryMode(bool on) {
    EEPROM.update(STORAGE_OFF_GALLERY, on ? 1 : 0);
    return true;
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
inline int _storageReadRecord(int off, uint16_t payloadMax, uint8_t* out, int maxLen) {
    uint16_t len = ((uint16_t)EEPROM.read(off + 0) << 8) | EEPROM.read(off + 1);
    if (len == 0 || len > payloadMax) return -1;
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

inline int _storageReadSlot(int slotIdx, uint8_t* out, int maxLen) {
    int off = (slotIdx == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;
    return _storageReadRecord(off, STORAGE_DUAL_PAYLOAD_MAX, out, maxLen);
}

inline int _storageReadLarge(uint8_t* out, int maxLen) {
    return _storageReadRecord(STORAGE_LARGE_OFF, STORAGE_LARGE_PAYLOAD_MAX, out, maxLen);
}

inline bool storageHasActive() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    return (a == 0 || a == 1 || a == STORAGE_ACTIVE_LARGE);
}

inline bool storageActiveIsLarge() {
    return EEPROM.read(STORAGE_OFF_ACTIVE) == STORAGE_ACTIVE_LARGE;
}

// Returns the byte count stored in the active slot's length header, or 0 if
// no active slot or the header reports an out-of-range length. Does not
// validate the slot's CRC — call storageHasActive() if you also need that.
inline int storageActiveBytesUsed() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1 && a != STORAGE_ACTIVE_LARGE) return 0;
    int off = (a == 0) ? STORAGE_SLOT_0_OFF
            : (a == 1) ? STORAGE_SLOT_1_OFF : STORAGE_LARGE_OFF;
    int n = ((int)EEPROM.read(off) << 8) | EEPROM.read(off + 1);
    int maxPayload = (a == STORAGE_ACTIVE_LARGE)
        ? STORAGE_LARGE_PAYLOAD_MAX : STORAGE_DUAL_PAYLOAD_MAX;
    if (n < 0 || n > maxPayload) return 0;
    return n;
}

inline bool storageHasPrevious() {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return false;
    int prev = (a == 0) ? 1 : 0;
    // Uses the shared BSS scratch buffer (see storageScratchBuffer). Not
    // reentrancy-safe — do not call storageHasPrevious() from an ISR or while
    // another consumer of the shared scratch (e.g., handleSequencesPost's
    // receive loop) is still using it. Single-threaded loop() use is fine.
    uint8_t* tmp = storageScratchBuffer();
    return _storageReadSlot(prev, tmp, STORAGE_DUAL_PAYLOAD_MAX) >= 0;
}

inline int storageReadActive(uint8_t* out, int maxLen) {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a == STORAGE_ACTIVE_LARGE) return _storageReadLarge(out, maxLen);
    if (a != 0 && a != 1) return -1;
    return _storageReadSlot((int)a, out, maxLen);
}
inline int storageReadPrevious(uint8_t* out, int maxLen) {
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a != 0 && a != 1) return -1;
    return _storageReadSlot(a == 0 ? 1 : 0, out, maxLen);
}

// Returns true on success. Payloads through 4080 bytes use the original two
// slots: write inactive first, then atomically flip the pointer, retaining a
// previous bake. Larger payloads use one record across the sequencer region.
// That mode cannot retain rollback or power-loss atomicity because two large
// records cannot coexist in 8168 bytes; the browser calls this out explicitly.
inline bool storageWriteSlot(const uint8_t* payload, uint16_t len) {
    if (len == 0 || len > STORAGE_PAYLOAD_MAX) return false;
    uint8_t cur = EEPROM.read(STORAGE_OFF_ACTIVE);
    bool large = len > STORAGE_DUAL_PAYLOAD_MAX;
    int target = large ? STORAGE_ACTIVE_LARGE : ((cur == 0) ? 1 : 0);
    int off = large ? STORAGE_LARGE_OFF
            : ((target == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF);

    // Any transition involving the overlapping large record is intentionally
    // non-atomic. Mark it invalid while bytes are changing so a reset cannot
    // advertise a partially overwritten record as active. When returning to
    // dual mode, also invalidate slot 1: its header lies inside the old large
    // payload and could otherwise coincidentally resemble a valid record.
    if (large || cur == STORAGE_ACTIVE_LARGE) {
        EEPROM.update(STORAGE_OFF_ACTIVE, 0xFE);
        if (!large) {
            EEPROM.write(STORAGE_SLOT_1_OFF + 0, 0);
            EEPROM.write(STORAGE_SLOT_1_OFF + 1, 0);
        }
    }

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
