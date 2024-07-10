
/* Libraries */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>        // LCD driver
#include <Adafruit_BMP280.h>          // Pressure sensor driver

/* Variables*/
Adafruit_BMP280 bmp;                  // Instantiate pressure sensor
LiquidCrystal_I2C lcd(0x27,20,4);     // Instantiate LCD - 0x27 for a 16 chars and 2 line display

int enableA = 2;                      // Enable A pin on L298 H-Bridge
int in1 = 3;                          // Motor A IN1 pin on L298 H-Bridge
int in2 = 5;                          // Motor A IN2 pin on L298 H-Bridge

unsigned long time_now = 0;

int pot = A0;                         // analog ports are named An

/* Function Prototypes */
void setPressureSensor();             // Setup pressure sensor
void getPressureReading();            // Print out pressure reading
void setLCD();                        // Setup LCD
void setMotorA();                     // Setup H-bridge pins to control motor A
int getPot();                         // Returns mapped potentiometer value for analogWrite

/* Setup */
void setup()
{
  Wire.begin();                        // Start I2C bus
  Serial.begin(9600);
  while (!Serial);                     // Wait for serial monitor
  Serial.println("\nSerial Online");

  setLCD();
  setPressureSensor();
  setMotorA();
}

/* Main Loop */
void loop()
{
  time_now = millis();

  getPressureReading();

  Serial.println(getPot());

  analogWrite(in1, getPot());
  analogWrite(in2, LOW);
  
  delay(500);
}

/* Function Definitions */
void setPressureSensor()
{
  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void getPressureReading()
{
  //Serial.print(F("Temperature = "));
  //Serial.print(bmp.readTemperature());
  //Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  //Serial.println();
}

void setLCD()
{
  lcd.init();                      
  lcd.backlight();
}

void setMotorA()
{
  pinMode(enableA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(enableA, HIGH);
}

int getPot() 
{
  int sensorValue = analogRead(pot);
  int mappedValue = map(sensorValue, 0, 1023, 0, 255);
  return mappedValue;
}
