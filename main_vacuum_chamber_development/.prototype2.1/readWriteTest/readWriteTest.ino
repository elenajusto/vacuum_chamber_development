/*
  Modified SD card read/write to return the values stored in a text based
  csv file containing pressure data.
*/

/* Includes */
#include <SPI.h>
#include <SD.h>

/* Variables */
File myFile;                     // File instance

String txtBuffer;                // String to hold one line of text

/* Function Prototypes */
void readFromCard(String filename);                         // Read all data from input file
void getDataSendCommand(String filename, String buffer);    // Get data and call actuator function
void dummyActuator(int value);                              // Simulate control of actuator 

void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Read from Card
  //readFromCard("OUTPUT.TXT");

  // Read data and send commands
  getDataSendCommand("OUTPUT.TXT", txtBuffer);
}

void loop() {
  // nothing happens after setup
}


/* Function Definitions */
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
