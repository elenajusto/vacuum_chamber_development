
/* Libraries */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>        // LCD driver
#include <Adafruit_BMP280.h>          // Pressure sensor driver
#include <SPI.h>                      // SD card communications
#include <SD.h>                       // SD card communications

/* Variables*/
Adafruit_BMP280 bmp;                  // Instantiate pressure sensor
LiquidCrystal_I2C lcd(0x27,20,4);     // Instantiate LCD - 0x27 for a 16 chars and 2 line display
File myFile;                     // File instance

String txtBuffer;                // String to hold one line of text

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
int getPot();                                               // Returns mapped potentiometer value for analogWrite
void analogMotorControl(int analogValue);                   // Control vaccum suction based on potentiometer
void readFromCard(String filename);                         // Read all data from input file
void getDataSendCommand(String filename, String buffer);    // Get data and call actuator function
void dummyActuator(int value);        

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
  
  /* Control Chamber Pressure */

  // Manual Control
  //analogMotorControl(getPot());

  // PID Control
  // Get value from sensor (feedback)
  int val = bmp.readAltitude(1013.25);         

  // Calculating PID terms
  int error = setPoint - val;                   
  integral = integral + (error * interval);
  int derivative = (error - error_prev) / interval;
  int output = (k_p * error) + (k_i * integral) + (k_d * derivative);

  // Get error
  error_prev = error;

  // Send command to actuator (vacuum pump motor)
  analogMotorControl(output);

  // Debug
  Serial.print("PID Output: ");
  Serial.println(output);

  printAltitudeReading();

  delay(interval);
  
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

void readFromCard(String filename)
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  
  Serial.println("initialization done.");

  // re-open the file for reading:
  myFile = SD.open(filename);
  if (myFile) {

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    
    // close the file:
    myFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void getDataSendCommand(String filename, String buffer)
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  
  Serial.println("initialization done.");

  // re-open the file for reading:
  myFile = SD.open(filename);
  if (myFile) {

    // read from the file until there's nothing else in it:
    while (myFile.available()) {

      // Write one line to buffer
      txtBuffer = myFile.readStringUntil(',');
      
      // Print to serial monitor
      Serial.println(txtBuffer);
      
      // Convert string to integer and position servo
      dummyActuator( txtBuffer.toFloat() );
      
    }
    
    // close the file:
    myFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void dummyActuator(int data)
{
  Serial.print("Sending ");
  Serial.print(data);
  Serial.println(" to actuator...");
  delay(1000);
  Serial.println("Actuation command done!");
  delay(500);
}
