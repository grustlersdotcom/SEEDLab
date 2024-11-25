#include "DualMC33926MotorShield.h"  // Include the motor shield library
#include <Wire.h>
#define MY_ADDR 8

extern volatile int8_t command = 0;
extern volatile int8_t junk = 0;
extern volatile uint8_t receivedFlag = 0;
extern volatile int8_t updatedCommand = 0;
extern volatile int8_t lastCommand = 0;
extern const float voltage_max;
extern float left_target_pos, right_target_pos;


extern bool move_mode = true;

DualMC33926MotorShield motorShield;  // Declare the motor shield object globally

// change the below two variables to suit your needs
const bool enableDistance = true;
const bool enableAngle = true;

const float targetDistance = 6 + 0.08; // ft + 0.08
volatile float targetAngle = 20; // degrees, initial search angle

const float meters_per_foot = 0.3048; // they give feet, system works in meters
const float degrees_per_radian = 180.0 / 3.14159; // they give radians, system works in degrees
const uint8_t SPEED = 150;

// state tracking variables
// state of 0 = Idle/Searching
// state of 1 = Turning
// state of 2 = Moving Straight
// state of 3 = Completed Movement / Stopped
int state = 0;
int stat_print_count = 0;
unsigned long previousMillis = 0;
const long interval = 1000; // Interval for control loop timing

unsigned long calculateServoDelay(int desiredAngle) {
  // Time it takes for 90 degrees rotation
  const unsigned long fullRotationTime = 1375; // ms
  
  // Calculate fraction of full rotation
  float angleFraction = abs(desiredAngle) / 90.0;
  
  // Calculate delay time
  unsigned long delayTime = angleFraction * fullRotationTime;
  
  return delayTime;
}


void setup() {
  initialize_encoders();
  initializeI2C();
  Serial.begin(9600);  // Initialize serial communication

  Serial.println("Initializing motor shield...");
  motorShield.init();
  Serial.println("Motor shield initialized!");

  init_statistics();
  //attachInterrupt(digitalPinToInterrupt(1), stopMovement, FALLING); // Use FALLING to avoid noise issues

  move_mode = false; // Explicitly initialize move_mode
}

void loop() {


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    switch (state) {
      case 0:
        if (enableAngle) {
          configure_move_angle(targetAngle); // Rotate 20 degrees continuously
          move_both_motors();
          Serial.println("State 0");
          state = 1;
        }
        state = 0;
        break;
      case 1:
        if (enableAngle) {
          configure_move_angle(targetAngle); // Rotate updated degrees
          Serial.println("State 1");
          move_both_motors();
        }
        //move_both_motors();
        if (has_reached_target()) {
          state = 2;
        }
        break;
      case 2:
        motorShield.setM1Speed(SPEED);
        motorShield.setM2Speed(SPEED);
        while (state == 2) {
        // Simplified PID to correct the direction based on the angle received
          int error = command;  // Command represents the deviation angle from the target (in degrees)
          
          // Proportional term
          int Kp = 10;  // Proportional gain, adjust to tune responsiveness
          int correction = Kp * error;

          // Limit the correction to a reasonable range to avoid excessive oscillation
          correction = constrain(correction, -20, 20);

          // Adjust motor PWM based on correction
          if (error < 0) {
            // If the angle is negative, reduce speed of M1 to correct left
            motorShield.setM1Speed(0.95*(SPEED + correction));
            motorShield.setM2Speed(SPEED);
          } else if (error > 0) {
            // If the angle is positive, reduce speed of M2 to correct right
            motorShield.setM1Speed(0.95*SPEED);
            motorShield.setM2Speed(SPEED - correction);
          } else {
            // If no error, keep motors at base PWM
            motorShield.setM1Speed(0.95*SPEED);
            motorShield.setM2Speed(SPEED);
          }
          // Add a small delay to stabilize the loop
          delay(5);
      }
        break;
      case 3:
        Serial.println("Completed movement.");
        update_statistics(10 * voltage_max, 10 * voltage_max);
        motorShield.setM1Speed(0);
        motorShield.setM2Speed(0);
        Serial.println("State 3");
        move_mode = false;
        state = 4;
        command = 0;
        break;
      case 4:
      Serial.println("Case 4");
      Serial.println(command);
          if(command>0){
            motorShield.setM1Speed(125);
            motorShield.setM2Speed(-125);
            state = 5;
          } if (command<0){
            motorShield.setM1Speed(-125);
            motorShield.setM2Speed(125);
            state = 5;
          }
        delay(1375);
        /*
        Serial.println("Sending ACK");
        Wire.beginTransmission(MY_ADDR);
        Wire.write("D");
        Wire.endTransmission();
        */
        break;
      case 5:
      Serial.println("Case 5");
          Serial.println("Target reached");
          motorShield.setM1Speed(0);
          motorShield.setM2Speed(0);
        break;
  }

  delay(100);  // Adjust delay for control loop timing
}
}

void handleI2CInput() {
  junk = Wire.read();
  while(Wire.available()){
    command =-Wire.read();  // Read the command byte
    Serial.println(command);
    receivedFlag = 1;       // Set the flag to indicate a message has been received
    updatedCommand = command;

  }
  if ((receivedFlag > 0) && (updatedCommand != lastCommand)) {
    switch (command) {
      case 91:
        targetAngle = 90;  // Turn 90 degrees right
        state = 1;
        break;
      case -91:
        targetAngle = -90; // Turn 90 degrees left
        state = 1;
        break;
      case -26:
        //Add something about transitioning to state 3
        stopMovement();
        state = 3;
        break;
      case -100:
        state = 2;
        break;
      case -99:
        state = 5;
        break;
      default:
        if ((command != 127) && (command <= 25) && (command >= -25)) { // Marker detected, stop rotation and turn to specified angle
          stopMovement();
          //delay(100); // Use a shorter delay to avoid blocking
          targetAngle = command; // Turn to the marker angle received
          if (lastCommand > (command + 7) || lastCommand < (command - 7)){
            state = 1;
          }
          //Serial.println(command);
        }
        break;
    }
    receivedFlag = 0; // Reset message length after processing
    lastCommand = updatedCommand;
  }
}

void stopMovement() {
  apply_simultaneous_voltage(0, 0);
}

void initializeI2C() {
  Wire.begin(MY_ADDR);         // Initialize I2C with the given address
  Wire.onReceive(handleI2CInput);     // Set ISR when data is received over I2C
}
