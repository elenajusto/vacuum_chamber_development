/*
  Modified SD card read/write to return the values stored in a text based
  csv file containing pressure data.
*/

/* Includes */
#include <SPI.h>
#include <SD.h>

/* Variables */
File myFile;

int numberOfTime;             // Store number of time measurements from csv
int numberOfAltitude;         // Store number of altitude measurements from csv
int numberOfPressure;         // Store number of pressure measurements from csv

/* Function Prototypes */
void readFromCard();
void getTimeNumber();
void getAltitudeNumber();
void getPressureNumber();

/* Main Program */
void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Read from Card
  readFromCard();
}

void loop() {
  // nothing happens after setup
}


/* Function Definitions */
void readFromCard()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // re-open the file for reading:
  myFile = SD.open("OUTPUT.TXT");
  if (myFile) {
    Serial.println("examplecurve.txt:");

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

void getTimeNumber()
{
  
}

void getAltitudeNumber()
{
  
}

void getPressureNumber()
{
  
}
