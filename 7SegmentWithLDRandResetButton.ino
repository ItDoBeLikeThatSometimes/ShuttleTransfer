#include "SevSeg.h"
#include <Wire.h> // Include Wire library

SevSeg sevseg; 

int counter = 0; // Declare a counter variable
unsigned long previousMillis = 0; // Variable to store the time of the last counter increment
const long interval = 1000; // Interval between counter increments (in milliseconds)

const int ldrPin = A1; // The analog pin for the LDR sensor
int ldrValue = 0; // Variable to store the LDR sensor reading
int ldrThreshold = 800; // Threshold to detect when LDR value is high enough

bool paused = true; // Start with counter paused

// Pin to signal counter increment to master
const int counterIncrementPin = 3; // Digital pin 3

void setup(){
  byte numDigits = 4;
  byte digitPins[] = {10, 11, 12, 13};
  byte segmentPins[] = {9, 2, 3, 5, 6, 8, 7, 4};

  bool resistorsOnSegments = true; 
  bool updateWithDelaysIn = true;
  byte hardwareConfig = COMMON_CATHODE; 
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevseg.setBrightness(90);
  
  pinMode(ldrPin, INPUT); // Set the LDR pin as input
  
  // Initialize the counter increment pin
  pinMode(counterIncrementPin, OUTPUT);
  digitalWrite(counterIncrementPin, LOW); // Ensure it's LOW at start
  
  Serial.begin(9600); // Initialize the serial monitor
  
  Wire.begin(4); // Join I2C bus as slave with address 4
  Wire.onReceive(receiveEvent); // Register receive event
  Wire.onRequest(requestEvent); // Register request event
}

void loop(){
    // Continuously refresh the display for a smooth appearance
    sevseg.refreshDisplay();

    unsigned long currentMillis = millis();

    // Continuously read the LDR value
    ldrValue = analogRead(ldrPin);
    // Optional: 
    Serial.print("LDR Value: "); Serial.println(ldrValue);

    // Check if it's time to increment the counter based on the LDR and interval
    if (!paused && ldrValue > ldrThreshold && currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Update the last increment time
        counter++; // Increment the counter
        sevseg.setNumber(counter); // Update the counter on the display

        // Signal the master that the counter has incremented
        digitalWrite(counterIncrementPin, HIGH);
        delay(10); // Short pulse to ensure master detects it
        digitalWrite(counterIncrementPin, LOW);
    }
}

void receiveEvent(int howMany) {
    if (howMany < 1) return;
    char cmd = Wire.read();
    switch (cmd) {
        case 'R': // Reset counter
            counter = 0;
            sevseg.setNumber(counter);
            break;
        case 'S': // Set counter value
            if (howMany >= 3) {
                int highByte = Wire.read();
                int lowByte = Wire.read();
                counter = (highByte << 8) | lowByte;
                sevseg.setNumber(counter);
            }
            break;
        case 'T': // Set LDR threshold
            if (howMany >= 3) {
                int highByte = Wire.read();
                int lowByte = Wire.read();
                ldrThreshold = (highByte << 8) | lowByte;
            }
            break;
        case 'G': // Get counter value
            // Handled in requestEvent()
            break;
        case 'P': // Pause counter
            paused = true;
            break;
        case 'C': // Continue counter
            paused = false;
            break;
    }
}

void requestEvent() {
    Wire.write((counter >> 8) & 0xFF); // Send high byte of counter
    Wire.write(counter & 0xFF);        // Send low byte of counter
}
