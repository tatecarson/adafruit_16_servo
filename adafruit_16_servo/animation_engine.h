#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"

void updateAnimations() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (servoState[i].stopped || !servoState[i].moving) continue;

    unsigned long elapsed = now - servoState[i].moveStartMs;

    if (elapsed >= servoState[i].moveDurationMs) {
      servoState[i].posPulse = servoState[i].targetPulse;
      pwm.setPWM(i, 0, servoState[i].posPulse);
      servoState[i].moving = false;
    } else {
      float progress = (float)elapsed / (float)servoState[i].moveDurationMs;
      // DMOVE/UMOVE set linearMove=true so segments connect at constant
      // velocity. SWEEP / motion playback leave linearMove=false so they
      // get the gentler ease-in-out shape.
      uint16_t newPos = servoState[i].linearMove
        ? lerpLinear(servoState[i].startPulse, servoState[i].targetPulse, progress)
        : lerpEased (servoState[i].startPulse, servoState[i].targetPulse, progress);
      if (newPos != servoState[i].posPulse) {
        servoState[i].posPulse = newPos;
        pwm.setPWM(i, 0, newPos);
      }
    }
  }
}

void updateSpeedRamps() {
  updateMotorRamp();
}

// WAVE command was removed for OTA-flash partition headroom (servo-dz7).
// Legacy numbered PLAY/SPLAY/RUN-n playback (updateSequence, updateSpeedSequence,
// updateSequenceProgram, startNextProgram*) was removed in servo-voc — schema-v1
// browser-baked Motions/Sequences/Setlists are the only playback path now.
