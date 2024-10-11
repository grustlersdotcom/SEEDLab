//This is code to test the encoders individually. Change pinA and pinB according to what motor it is.
//Left motor should be pins 2 and 4. Right motor should be pins 3 and 5.
//Forward roll should count up and backward count down.

const int pinA = 2;  // CLK pin on Encoder
const int pinB = 4;  // DT pin on Encoder
volatile int position = 0;  // To track encoder position
volatile int prevA = LOW;

void setup() {
  pinMode(pinA, INPUT_PULLUP);  // Enable pull-up resistor on pinA
  pinMode(pinB, INPUT_PULLUP);  // Enable pull-up resistor on pinB
  Serial.begin(9600);

  // Attach interrupt to pinA, trigger on any change (rising or falling edge)
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
}

void loop() {
  // The main loop can do other tasks or just wait
  // Print the position value if it has changed
  static int lastPosition = 0;
  if (lastPosition != position) {
    Serial.print("Position: ");
    Serial.println(position);
    lastPosition = position;
  }
}

// Interrupt Service Routine (ISR) to handle encoder changes
void updateEncoder() {
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  // Only count on the rising edge of pinA
  if (prevA == LOW && currentA == HIGH) {
    // Determine rotation direction based on the state of pinB
    if (currentB == LOW) {
      position++;  // Clockwise rotation
    } else {
      position--;  // Counterclockwise rotation
    }
  }

  prevA = currentA;  // Update the previous state for pinA
}