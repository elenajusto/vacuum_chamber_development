#include <Arduino.h>
#include <unity.h>
#include <LiquidCrystal_I2C.h>

void setUp(void) {
  pinMode(7, OUTPUT);     // Red light - Test failed
  pinMode(8, OUTPUT);     // Green light - Test Success

  // Testing start
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  delay(1000);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}

void tearDown(void) {
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}

void testLights(){
  if(Unity.TestFailures == 0U){
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(7, HIGH);
  }
}

void test_lcd() {
  LiquidCrystal_I2C lcd(0x27,20,4);
  lcd.init();
  TEST_ASSERT(lcd.print("Hello, world!"));
}

void setup() {
  UNITY_BEGIN();

  RUN_TEST(test_lcd);

  testLights();

  UNITY_END();
}

void loop() {}
