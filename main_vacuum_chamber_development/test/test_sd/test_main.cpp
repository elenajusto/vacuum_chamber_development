#include <Arduino.h>
#include <unity.h>
#include <SD.h>

void test_sd() {
  // Connect to SD card
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_sd);
  UNITY_END();
}

void loop() {}
