// Encoder pins
const int encoder_left_A = 2;  // Left wheel encoder A (interrupt pin)
const int encoder_left_B = 5;  // Left wheel encoder B
const int encoder_right_A = 3; // Right wheel encoder A (interrupt pin)
const int encoder_right_B = 6; // Right wheel encoder B

// Variables to track encoder counts
volatile int encoder_left_count = 0;
volatile int encoder_right_count = 0;
volatile int previous_left_A = LOW;
volatile int previous_right_A = LOW;

// Robot constants (update these to accurate values)
const float wheel_radius = 0.1524/2;  // Wheel radius in meters (6 inch diameter = 0.1524 meters)
const float wheel_base = 0.2921;  // Distance between the center of two wheels (11.5 inches = 0.2921 meters)
const int encoder_resolution = 3200/4;  // Encoder ticks per revolution
const float distance_per_encoder_tick = (2.0 * PI * wheel_radius) / encoder_resolution;

// Function to initialize encoders
void initialize_encoders() {
  pinMode(encoder_left_A, INPUT_PULLUP);
  pinMode(encoder_left_B, INPUT_PULLUP);
  pinMode(encoder_right_A, INPUT_PULLUP);
  pinMode(encoder_right_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder_left_A), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_right_A), right_encoder_isr, CHANGE);
}

// Interrupt service routines for encoder counts
void left_encoder_isr() {
  int currentA = digitalRead(encoder_left_A);
  int currentB = digitalRead(encoder_left_B);

  if (previous_left_A == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoder_left_count++;  // Clockwise
    } else {
      encoder_left_count--;  // Counterclockwise
    }
  }
  previous_left_A = currentA;
}

void right_encoder_isr() {
  int currentA = digitalRead(encoder_right_A);
  int currentB = digitalRead(encoder_right_B);

  if (previous_right_A == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoder_right_count--;  // Clockwise
    } else {
      encoder_right_count++;  // Counterclockwise
    }
  }
  previous_right_A = currentA;
}