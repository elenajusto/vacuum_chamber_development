
/* Libraries */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>        // LCD driver
#include <Adafruit_BMP280.h>          // Pressure sensor driver

/* Variables*/
Adafruit_BMP280 bmp;                  // Instantiate pressure sensor
LiquidCrystal_I2C lcd(0x27,20,4);     // Instantiate LCD - 0x27 for a 16 chars and 2 line display

int enableA = 1;                      // Enable A pin on L298 H-Bridge
int in1 = 3;                          // Motor A IN1 pin on L298 H-Bridge
int in2 = 5;                          // Motor A IN2 pin on L298 H-Bridge
int pot = A0;                         // Potentiometer pin

unsigned long time_now = 0;           // Tracking time for millis()

int k_p = 2;                          // Proportional Constant (PID Control)
int k_i = 0.1;                        // Integral Constant (PID Control)
int k_d = 0.1;                        // Derivative Constant (PID Control)
int interval = 500;                   // e.g., 1ms

int setPoint = 212;
int error_prev = 0;
int integral = 0;
  
/* Function Prototypes */
void setPressureSensor();                   // Setup pressure sensor
void setLCD();                              // Setup LCD
void setMotorA();                           // Setup H-bridge pins to control motor A

void printTemperatureReading();
void printPressureReading();
void printAltitudeReading();

int getPot();                               // Returns mapped potentiometer value for analogWrite

void analogMotorControl(int analogValue);   // Control vaccum suction based on potentiometer

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
  // Debug Information
  printAltitudeReading();
  Serial.print("     PWM Output = ");
  Serial.println(getPot());

  // Control Chamber Pressure
  analogMotorControl(getPot());

  /* PID Control Loop */
  /*
  int val = bmp.readAltitude(1013.25);          // Get value from sensor (feedback)

  int error = setPoint - val;                   // Calculating PID terms
  integral = integral + (error * interval);
  int derivative = (error - error_prev) / interval;
  int output = (k_p * error) + (k_i * integral) + (k_d * derivative);

  error_prev = error;

  analogWrite(in1, output);
  analogWrite(in2, LOW);

  Serial.print("PID Output: ");
  Serial.print(output);

  delay(interval);
  */
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

void printTemperatureReading()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
}

void printPressureReading()
{
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
}

void printAltitudeReading()
{
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
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

void analogMotorControl(int analogValue)
{
  analogWrite(in1, analogValue);
  analogWrite(in2, LOW);
}
