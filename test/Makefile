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

.PHONY: all test clean size ota-budget storage motion sequence setlist gallery sim-verify

all: test

# Browser setlist simulation (servo-6j4) must reproduce the firmware scheduler's
# pick order exactly. This extracts the SETLIST-SIM-CORE block from
# servo_controller.html and asserts it matches firmware-captured oracle picks.
# Requires node. Fails fast if node is missing so the oracle check can't be
# silently skipped; set SKIP_SIM_VERIFY=1 to deliberately bypass it.
sim-verify:
	@if [ -n "$(SKIP_SIM_VERIFY)" ]; then \
	  echo "SKIP sim-verify: SKIP_SIM_VERIFY set"; \
	else \
	  command -v node >/dev/null 2>&1 || { \
	    echo "ERROR: sim-verify needs Node.js (node not on PATH)." >&2; \
	    echo "       Install Node, or run with SKIP_SIM_VERIFY=1 to bypass." >&2; \
	    exit 1; \
	  }; \
	  node verify_sim_matches_firmware.mjs && \
	  node verify_sim_analysis.mjs && \
	  node verify_shape_curves.mjs && \
	  node verify_editor_keyframes.mjs && \
	  node verify_seq_preview.mjs && \
	  node verify_ids.mjs && \
	  node verify_seq_arrangement.mjs && \
	  node verify_motor_test_safety.mjs && \
	  node verify_baked_motion_preroll.mjs; \
	fi

# Aggregate run of every host suite. The legacy time-multiplier suite was
# removed in servo-voc along with the legacy PLAY/SPLAY/RUN-n engine it tested.
test: storage motion sequence setlist gallery sim-verify

run_storage_tests: test_storage.cpp mock/Arduino.h mock/EEPROM.h
	$(CXX) $(CXXFLAGS) -o $@ $<

storage: run_storage_tests
	./run_storage_tests

run_motion_tests: test_motion_engine.cpp mock/Arduino.h mock/EEPROM.h mock/Adafruit_PWMServoDriver.h
	$(CXX) $(CXXFLAGS) -o $@ $<

motion: run_motion_tests
	./run_motion_tests

run_sequence_tests: test_sequence_engine.cpp mock/Arduino.h mock/EEPROM.h mock/Adafruit_PWMServoDriver.h
	$(CXX) $(CXXFLAGS) -o $@ $<

sequence: run_sequence_tests
	./run_sequence_tests

run_setlist_tests: test_setlist_scheduler.cpp $(SKETCH_DIR)/setlist_scheduler.h $(SKETCH_DIR)/sequence_engine.h $(SKETCH_DIR)/servo_runtime.h mock/Arduino.h mock/EEPROM.h mock/Adafruit_PWMServoDriver.h
	$(CXX) $(CXXFLAGS) -o $@ $<

setlist: run_setlist_tests
	./run_setlist_tests

run_gallery_tests: test_gallery_mode.cpp $(SKETCH_DIR)/gallery_mode.h $(SKETCH_DIR)/setlist_scheduler.h $(SKETCH_DIR)/storage.h $(SKETCH_DIR)/servo_runtime.h mock/Arduino.h mock/EEPROM.h mock/Adafruit_PWMServoDriver.h
	$(CXX) $(CXXFLAGS) -o $@ $<

gallery: run_gallery_tests
	./run_gallery_tests

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
	rm -f run_storage_tests run_motion_tests run_sequence_tests run_setlist_tests run_gallery_tests
