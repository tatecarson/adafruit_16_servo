CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wno-unused-variable -Wno-unused-function -Wno-unused-parameter -I mock
TARGET   = run_tests

# UNO R4 WiFi OTA partition cap. Anything over this brick-flashes the board
# when OTA'd (servo-8zb / servo-dz7). Math: (FLASH_LENGTH 0x40000
# - SKETCH_START_ADDRESS 0x4000) / 2, page-aligned to 2 KB pages.
OTA_BUDGET = 122880
FQBN       = arduino:renesas_uno:unor4wifi
BUILD_DIR  = /tmp/adafruit-16-servo-build
SKETCH_DIR = ../adafruit_16_servo

.PHONY: all test clean size ota-budget

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): test_time_multiplier.cpp mock/Arduino.h mock/Wire.h mock/Adafruit_PWMServoDriver.h mock/WiFiS3.h mock/avr/pgmspace.h
	$(CXX) $(CXXFLAGS) -o $@ $<

run_storage_tests: test_storage.cpp mock/Arduino.h mock/EEPROM.h
	$(CXX) $(CXXFLAGS) -o $@ $<

storage: run_storage_tests
	./run_storage_tests

run_motion_tests: test_motion_engine.cpp mock/Arduino.h mock/EEPROM.h mock/Adafruit_PWMServoDriver.h
	$(CXX) $(CXXFLAGS) -o $@ $<

motion: run_motion_tests
	./run_motion_tests

# OTA budget guard — compile the sketch and fail loud if the resulting
# binary exceeds the OTA partition limit. Catches the kind of accidental
# growth that bricked the cluster during servo-8zb.
size: ota-budget
ota-budget:
	@echo "=== OTA Partition Size Check ==="
	@echo "  budget: $(OTA_BUDGET) bytes (UNO R4 WiFi OTA slot)"
	@command -v arduino-cli >/dev/null 2>&1 || { \
	  echo "" ; \
	  echo "ERROR: arduino-cli not on PATH. Install from https://arduino.github.io/arduino-cli/" >&2 ; \
	  exit 2 ; \
	}
	@arduino-cli compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKETCH_DIR) 2>&1 | tee /tmp/ota-budget-build.log
	@SIZE=$$(grep -oE 'Sketch uses [0-9]+ bytes' /tmp/ota-budget-build.log | grep -oE '[0-9]+' | head -1) ; \
	if [ -z "$$SIZE" ]; then \
	  echo "ERROR: could not parse sketch size from arduino-cli output" >&2 ; \
	  exit 1 ; \
	fi ; \
	echo "" ; \
	if [ $$SIZE -le $(OTA_BUDGET) ]; then \
	  HEADROOM=$$(($(OTA_BUDGET) - $$SIZE)) ; \
	  echo "  PASS  $$SIZE / $(OTA_BUDGET) bytes  (+$$HEADROOM headroom)" ; \
	else \
	  OVERAGE=$$(($$SIZE - $(OTA_BUDGET))) ; \
	  echo "  FAIL  $$SIZE / $(OTA_BUDGET) bytes  (-$$OVERAGE overage)" >&2 ; \
	  echo "" >&2 ; \
	  echo "  This binary exceeds the OTA partition cap on UNO R4 WiFi." >&2 ; \
	  echo "  OTA-flashing it would brick the cluster (servo-8zb)." >&2 ; \
	  echo "  Reduce binary size before pushing firmware changes — see" >&2 ; \
	  echo "  servo-dz7 for the patterns that worked last time (sinf/powf,"  >&2 ; \
	  echo "  removing unused commands like WAVE)." >&2 ; \
	  exit 1 ; \
	fi

clean:
	rm -f $(TARGET) run_storage_tests run_motion_tests
