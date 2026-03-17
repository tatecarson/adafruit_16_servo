#pragma once

#include "servo_runtime.h"

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
    if (servoState[i].stopped || !servoState[i].speedRamping) continue;

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
  float cycleMs = (float)waveSpeed * 360.0f;
  if (cycleMs < 1.0f) cycleMs = 1.0f;

  for (uint8_t i = waveStartServo; i <= waveEndServo; i++) {
    if (servoState[i].stopped || servoConfig[i].continuous) continue;
    uint8_t servoIndex = i - waveStartServo;
    uint16_t minDegrees = servoConfig[i].upDegrees;
    uint16_t maxDegrees = servoConfig[i].downDegrees;
    if (maxDegrees == 0) maxDegrees = servoConfig[i].totalDegrees;

    float phase = (float)elapsed / cycleMs;
    phase += (float)(servoIndex * wavePhaseOffset) / 360.0f;

    float sineVal = sin(phase * 2.0f * PI);
    float centerDegrees = ((float)minDegrees + (float)maxDegrees) / 2.0f;
    float degrees = centerDegrees + (sineVal * (float)waveAmplitude / 2.0f);
    uint16_t pulse = degreesToPulse(i, (uint16_t)constrain(degrees, minDegrees, maxDegrees));

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

    if (!servoState[kf.servo].stopped) {
      moveSequenceDegrees(kf.servo, kf.degrees, (uint32_t)kf.duration * timeMultiplier);
    }
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

    if (!servoState[sf.servo].stopped) {
      rampServoSpeed(sf.servo, sf.speed, (uint32_t)sf.rampMs * timeMultiplier);
    }
    lastTriggeredSpeedFrame = i + 1;
  }
}

bool startNextProgramPositionStep() {
  if (currentProgram == nullptr || currentProgram->positionSteps == nullptr || currentProgram->positionLength == 0) {
    programPositionDone = true;
    return false;
  }

  while (true) {
    if (currentProgramPositionStepIndex >= currentProgram->positionLength) {
      if (!programLoop) {
        programPositionDone = true;
        return false;
      }
      currentProgramPositionStepIndex = 0;
      currentProgramPositionIteration = 0;
      Serial.println(F("Program position track looping"));
    }

    ProgramSequenceStep step;
    memcpy_P(&step, &currentProgram->positionSteps[currentProgramPositionStepIndex], sizeof(ProgramSequenceStep));
    uint16_t repeatCount = step.repeatCount < 1 ? 1 : step.repeatCount;
    if (currentProgramPositionIteration >= repeatCount) {
      currentProgramPositionStepIndex++;
      currentProgramPositionIteration = 0;
      continue;
    }

    if (!startPositionSequence(step.sequenceNumber, false, true)) {
      programPositionDone = true;
      return false;
    }

    currentProgramPositionIteration++;
    programPositionDone = false;
    return true;
  }
}

bool startNextProgramSpeedStep() {
  if (currentProgram == nullptr || currentProgram->speedSteps == nullptr || currentProgram->speedLength == 0) {
    programSpeedDone = true;
    return false;
  }

  while (true) {
    if (currentProgramSpeedStepIndex >= currentProgram->speedLength) {
      if (!programLoop) {
        programSpeedDone = true;
        return false;
      }
      currentProgramSpeedStepIndex = 0;
      currentProgramSpeedIteration = 0;
      Serial.println(F("Program speed track looping"));
    }

    ProgramSequenceStep step;
    memcpy_P(&step, &currentProgram->speedSteps[currentProgramSpeedStepIndex], sizeof(ProgramSequenceStep));
    uint16_t repeatCount = step.repeatCount < 1 ? 1 : step.repeatCount;
    if (currentProgramSpeedIteration >= repeatCount) {
      currentProgramSpeedStepIndex++;
      currentProgramSpeedIteration = 0;
      continue;
    }

    if (!startSpeedSequence(step.sequenceNumber, false, true)) {
      programSpeedDone = true;
      return false;
    }

    currentProgramSpeedIteration++;
    programSpeedDone = false;
    return true;
  }
}

void updateSequenceProgram() {
  if (!programActive) return;
  if (!sequenceActive && !programPositionDone) {
    startNextProgramPositionStep();
  }
  if (!speedSeqActive && !programSpeedDone) {
    startNextProgramSpeedStep();
  }

  if (!sequenceActive && !speedSeqActive && programPositionDone && programSpeedDone) {
    programActive = false;
    Serial.println(F("Program complete"));
  }
}
