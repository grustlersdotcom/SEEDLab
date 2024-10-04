// Encoder pins
const int encoderLeftA = 2;  // Left wheel encoder A (interrupt pin)
const int encoderLeftB = 4;  // Left wheel encoder B
const int encoderRightA = 3; // Right wheel encoder A (interrupt pin)
const int encoderRightB = 5; // Right wheel encoder B

// Variables to track encoder counts
volatile int encoderLeftCount = 0;
volatile int encoderRightCount = 0;
volatile int prevLeftA = LOW;
volatile int prevRightA = LOW;

// Robot constants (update these to accurate values)
const float wheelRadius = 0.1524;  // Wheel radius in meters (15.24 cm)
const int encoderResolution = 800;  // Encoder ticks per revolution
const float wheelBase = 0.3048;  // Distance between the two wheels (meters) (30.48 cm)

// Position and orientation variables
float x = 0.0, y = 0.0, theta = 0.0;  // Robot position and orientation
float leftWheelDistance = 0.0, rightWheelDistance = 0.0;

volatile unsigned long lastRightDebounceTime = 0;  // Right encoder debounce timer
const unsigned long debounceDelay = 5;  // Debounce delay in milliseconds

bool moved = false;

// Function to initialize encoders
void initializeEncoders() {
  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), rightEncoderISR, CHANGE);
}

// Function to update the wheel distances and robot position
void updateWheelMetrics() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  lastTime = currentTime;
  
  // Calculate the distance per encoder tick
  float distancePerTick = (2.0 * PI * wheelRadius) / encoderResolution;

  // Calculate the distance traveled by each wheel
  leftWheelDistance = encoderLeftCount * distancePerTick;
  rightWheelDistance = encoderRightCount * distancePerTick;

  // Calculate linear and angular velocity
  float linearVelocity = (leftWheelDistance + rightWheelDistance) / 2.0;
  float angularVelocity = (rightWheelDistance - leftWheelDistance) / wheelBase;

  // Update position and orientation using Euler's method
  x += linearVelocity * timeDelta * cos(theta);
  y += linearVelocity * timeDelta * sin(theta);
  theta += angularVelocity * timeDelta;
}

// Interrupt service routines for encoder counts
void leftEncoderISR() {
  int currentA = digitalRead(encoderLeftA);
  int currentB = digitalRead(encoderLeftB);

  if (prevLeftA == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoderLeftCount++;  // Clockwise
    } else {
      encoderLeftCount--;  // Counterclockwise
    }
  }
  prevLeftA = currentA;
}

void rightEncoderISR() {
  int currentA = digitalRead(encoderRightA);
  int currentB = digitalRead(encoderRightB);

  if (prevRightA == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoderRightCount--;  // Clockwise
    } else {
      encoderRightCount++;  // Counterclockwise
    }
  }
  prevRightA = currentA;
}