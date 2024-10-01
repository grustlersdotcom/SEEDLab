extern volatile uint8_t quadrant;  // Quadrant value received from I2C
extern volatile uint8_t msgLength; // Flag indicating a message has been received

void serialTestQuadrantInput() {
  if (Serial.available() > 0) {
    // Read the input from the serial monitor
    char input = Serial.read();

    // Only handle valid quadrant inputs (1, 2, 3, 4)
    if (input >= '1' && input <= '4') {
      quadrant = input - '0';  // Convert char ('1', '2', etc.) to corresponding int (1, 2, etc.)
      msgLength = 1;  // Set flag to indicate a message has been received
      
      //Serial.print("Simulated Quadrant ");
      //Serial.println(quadrant);
    } else if (input != '\n' && input != '\r') {
      // If input is not a newline or carriage return, print invalid input message
      Serial.println("Invalid input. Enter 1, 2, 3, or 4 to simulate quadrant.");
    }
  }
}