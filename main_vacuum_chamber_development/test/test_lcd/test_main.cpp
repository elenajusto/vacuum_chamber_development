#include <Arduino.h>
#include <unity.h>
#include <LiquidCrystal_I2C.h>

void test_lcd() {
  // Connect to LCD
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_lcd);
  UNITY_END();
}

void loop() {}
