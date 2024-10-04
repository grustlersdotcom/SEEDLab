#include <Wire.h>
#define MY_ADDR 8

// Declare the variables to store received I2C data
volatile uint8_t quadrant = 0;  // Stores the received quadrant
volatile uint8_t msgLength = 0; // Flag indicating a message has been received

void initializeI2C() {
  Wire.begin(MY_ADDR);         // Initialize I2C with the given address
  Wire.onReceive(receive);     // Set ISR when data is received over I2C
}

// ISR function when the Arduino receives I2C information
void receive() {
  uint8_t offset = Wire.read();  // Read the offset (optional, depending on protocol)
  
  while (Wire.available()) {
    quadrant = Wire.read();  // Read the quadrant value
    msgLength = 1;           // Set the flag to indicate a message has been received
  }
}