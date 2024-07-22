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
void readFromCard(String filename);      // Read all data from input file

/* Main Program */
void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Read from Card
  readFromCard("OUTPUT.TXT");
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
