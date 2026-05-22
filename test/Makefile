CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wno-unused-variable -Wno-unused-function -Wno-unused-parameter -I mock
TARGET   = run_tests

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): test_time_multiplier.cpp mock/Arduino.h mock/Wire.h mock/Adafruit_PWMServoDriver.h mock/avr/pgmspace.h
	$(CXX) $(CXXFLAGS) -o $@ $<

run_storage_tests: test_storage.cpp mock/Arduino.h mock/EEPROM.h
	$(CXX) $(CXXFLAGS) -o $@ $<

storage: run_storage_tests
	./run_storage_tests

clean:
	rm -f $(TARGET) run_storage_tests
