
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

/* Function Prototypes */
void setPressureSensor();             // Setup pressure sensor
void getPressureReading();            // Print out pressure reading
void setLCD();                        // Setup LCD
void setMotorA();                     // Setup H-bridge pins to control motor A

/* Setup */
void setup()
{
  Wire.begin();                        // Start I2C bus
  Serial.begin(9600);
  while (!Serial);                     // Wait for serial monitor
  Serial.println("\nSerial Online");

  setPressureSensor();
  setMotorA();
}

/* Main Loop */
void loop()
{
  setLCD();
  getPressureReading();

  lcd.setCursor(1,0);
  //lcd.print("Temp: ");
  lcd.print(bmp.readTemperature());
  lcd.print(" *C");
  
  lcd.setCursor(1,1);
  //lcd.print(" Pres: ");
  lcd.print(bmp.readPressure());
  lcd.print(" Pa");
  
  //delay(1000);

  if (bmp.readAltitude(1013.25) < 200){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (bmp.readAltitude(1013.25) > 200) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
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
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
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
