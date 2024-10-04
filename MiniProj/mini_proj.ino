#include "DualMC33926MotorShield.h"  // Include the motor shield library
DualMC33926MotorShield motorShield;  // Declare the motor shield object globally

// Declare variables from the receive tab
extern volatile uint8_t quadrant;
extern volatile uint8_t msgLength;
extern float leftWheelDistance, rightWheelDistance;  // Distances from encoders

// Function prototypes
void initializeEncoders();
void updateWheelMetrics();
void checkQuadrantAndMove();
void serialTestQuadrantInput();

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Initialize encoders
  initializeEncoders();

  Serial.println("Initializing motor shield...");
  motorShield.init();
  Serial.println("Motor shield initialized!");

  // Initialize I2C communication
  initializeI2C();
}

void loop() {
  rightEncoderISR();
  leftEncoderISR();
  updateWheelMetrics();

  move_back();
  // Check for serial input to simulate quadrant input
  serialTestQuadrantInput();
  // Check if a quadrant message has been received via I2C
  if (msgLength > 0) {
    Serial.print("Received quadrant: ");
    Serial.println(quadrant);  // Debugging: print the received quadrant
    checkQuadrantAndMove();    // Call function to move wheels based on quadrant
    //msgLength = 0;             // Reset the message flag
  }

  delay(100);  // Adjust delay for control loop timing
  
}