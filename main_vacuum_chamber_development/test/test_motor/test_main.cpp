#include <Arduino.h>
#include <unity.h>

void test_motor() {
  // Turn on motor
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_motor);
  UNITY_END();
}

void loop() {}
