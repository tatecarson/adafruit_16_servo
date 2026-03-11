#pragma once

#include "servo_runtime.h"

void updateAnimations() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoState[i].moving) continue;

    unsigned long elapsed = now - servoState[i].moveStartMs;

    if (elapsed >= servoState[i].moveDurationMs) {
      servoState[i].posPulse = servoState[i].targetPulse;
      pwm.setPWM(i, 0, servoState[i].posPulse);
      servoState[i].moving = false;
    } else {
      float progress = (float)elapsed / (float)servoState[i].moveDurationMs;
      uint16_t newPos = lerpEased(servoState[i].startPulse, servoState[i].targetPulse, progress);
      if (newPos != servoState[i].posPulse) {
        servoState[i].posPulse = newPos;
        pwm.setPWM(i, 0, newPos);
      }
    }
  }
}

void updateSpeedRamps() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoState[i].speedRamping) continue;

    unsigned long elapsed = now - servoState[i].speedRampStartMs;

    if (elapsed >= servoState[i].speedRampDurationMs) {
      setServoSpeed(i, servoState[i].targetSpeed);
      servoState[i].speedRamping = false;
    } else {
      float t = (float)elapsed / servoState[i].speedRampDurationMs;
      if (t < 0.5f) {
        t = 4.0f * t * t * t;
      } else {
        t = 1.0f - pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
      }

      int8_t speed = servoState[i].startSpeed + (servoState[i].targetSpeed - servoState[i].startSpeed) * t;
      uint16_t pulse = speedToPulse(i, speed);
      servoState[i].posPulse = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}

void updateWave() {
  if (!waveActive) return;

  unsigned long elapsed = millis() - waveStartTime;

  for (uint8_t i = waveStartServo; i <= waveEndServo; i++) {
    uint8_t servoIndex = i - waveStartServo;
    float phase = (float)(elapsed) / (float)(waveSpeed * 360 / 1000);
    phase += (float)(servoIndex * wavePhaseOffset) / 360.0f;

    float sineVal = sin(phase * 2.0f * PI);
    float degrees = waveCenter + (sineVal * (float)waveAmplitude / 2.0f);
    uint16_t pulse = degreesToPulse(i, (uint16_t)constrain(degrees, 0, servoConfig[i].totalDegrees));

    if (pulse != servoState[i].posPulse) {
      servoState[i].posPulse = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}

void updateSequence() {
  if (!sequenceActive || currentSequence == nullptr) return;

  unsigned long elapsed = millis() - sequenceStartTime;
  Keyframe kf;

  for (uint8_t i = lastTriggeredKeyframe; i < currentSequenceLength; i++) {
    memcpy_P(&kf, &currentSequence[i], sizeof(Keyframe));

    if (elapsed < (uint32_t)kf.time * timeMultiplier) break;

    if (kf.servo == SEQUENCE_END_MARKER_SERVO) {
      if (sequenceLoop) {
        sequenceStartTime = millis();
        lastTriggeredKeyframe = 0;
        Serial.println(F("Sequence looping"));
      } else {
        sequenceActive = false;
        Serial.println(F("Sequence complete"));
      }
      return;
    }

    moveServoDegrees(kf.servo, kf.degrees, (uint32_t)kf.duration * timeMultiplier);
    lastTriggeredKeyframe = i + 1;
  }
}

void updateSpeedSequence() {
  if (!speedSeqActive || currentSpeedSeq == nullptr) return;

  unsigned long elapsed = millis() - speedSeqStartTime;
  SpeedFrame sf;

  for (uint8_t i = lastTriggeredSpeedFrame; i < currentSpeedSeqLength; i++) {
    memcpy_P(&sf, &currentSpeedSeq[i], sizeof(SpeedFrame));

    if (elapsed < (uint32_t)sf.time * timeMultiplier) break;

    if (sf.servo == SEQUENCE_END_MARKER_SERVO) {
      if (speedSeqLoop) {
        speedSeqStartTime = millis();
        lastTriggeredSpeedFrame = 0;
        Serial.println(F("Speed sequence looping"));
      } else {
        speedSeqActive = false;
        for (uint8_t j = 0; j < NUM_SERVOS; j++) {
          if (servoConfig[j].continuous) {
            setServoSpeed(j, 0);
          }
        }
        Serial.println(F("Speed sequence complete"));
      }
      return;
    }

    rampServoSpeed(sf.servo, sf.speed, (uint32_t)sf.rampMs * timeMultiplier);
    lastTriggeredSpeedFrame = i + 1;
  }
}
