#pragma once

// ============================================================
// Gallery Mode boot grace period (servo-4gl)
//
// gallery_mode is a persistent EEPROM flag (storage.h). When it is on, the
// device arms a one-shot grace countdown the first time network services come
// up at boot; when the window elapses it auto-runs the active Setlist via
// RUN AUTO (startActiveSetlist). The window exists so a browser can connect
// and take manual control before unattended operation begins — any command
// dispatched during the window cancels the pending auto-run.
//
// The window duration is schedulerConfig.graceMs from the active bake
// (default GALLERY_GRACE_DEFAULT_MS when absent or unbaked).
// ============================================================

#include "storage.h"
#include "setlist_scheduler.h"

#define GALLERY_GRACE_DEFAULT_MS 10000UL

struct GalleryGraceState {
  bool armed;        // arm() has run once this boot (one-shot guard)
  bool pending;      // countdown active, auto-run not yet fired
  uint32_t startMs;
  uint32_t graceMs;
};

extern GalleryGraceState galleryGrace;

// Arm the grace countdown once per boot. No-op if already armed (so a later
// WiFi reconnect doesn't restart it) or if gallery_mode is off. Reads graceMs
// from the active bake's schedulerConfig; defaults when there's no bake.
inline void galleryGraceArm() {
  if (galleryGrace.armed) return;
  galleryGrace.armed = true;
  if (!storageGalleryMode()) return;

  uint8_t* buf = storageScratchBuffer();
  int len = storageReadActive(buf, STORAGE_PAYLOAD_MAX);
  galleryGrace.graceMs = (len > 0) ? schedulerGraceMs(buf, len) : GALLERY_GRACE_DEFAULT_MS;
  galleryGrace.startMs = millis();
  galleryGrace.pending = true;

  Serial.print(F("Gallery mode: auto RUN AUTO in "));
  Serial.print(galleryGrace.graceMs);
  Serial.println(F("ms (send any command to cancel)"));
}

// Cancel a pending auto-run (a command arrived during the grace window, so an
// operator/browser is in control). Safe to call when nothing is pending.
inline void galleryGraceCancel() {
  if (!galleryGrace.pending) return;
  galleryGrace.pending = false;
  Serial.println(F("Gallery mode: grace canceled by command"));
}

// Fire the auto-run once the window elapses. Call every loop().
inline void updateGalleryGrace() {
  if (!galleryGrace.pending) return;
  if ((uint32_t)(millis() - galleryGrace.startMs) < galleryGrace.graceMs) return;
  galleryGrace.pending = false;
  Serial.println(F("Gallery mode: grace elapsed, starting RUN AUTO"));
  startActiveSetlist();
}
