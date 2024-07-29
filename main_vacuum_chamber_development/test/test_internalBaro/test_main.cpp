#include <Arduino.h>
#include <unity.h>
#include <Adafruit_BMP280.h>

void test_barometer() {
  Adafruit_BMP280 barometer = Adafruit_BMP280(&Wire);
  TEST_ASSERT_TRUE(barometer.begin((uint8_t)118U));
  TEST_ASSERT_FLOAT_WITHIN(100, 100304.6, barometer.readPressure());
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_barometer);
  UNITY_END();
}

void loop() {}
