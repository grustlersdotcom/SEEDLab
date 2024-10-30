#include "DualMC33926MotorShield.h"  // Include the motor shield library

extern volatile uint8_t quadrant;
extern volatile uint8_t msgLength;
extern const float voltage_max;
extern float left_target_pos, right_target_pos;

DualMC33926MotorShield motorShield;  // Declare the motor shield object globally

// change the below two variables to suit your needs
const bool enableDistance = true;
const bool enableAngle = false;

const float targetDistance = 7 + 0.08; // ft + 0.08  //180+0.09
const float targetAngle = 0; // radians (90 deg. == 1.57079 radians)//if 45 -5,  -135 +8, 180 - 9

const bool angleIsInDegrees = true; // change to false if they give you radians
const bool distanceIsInMeters = false; // change to true if they give you meters

const float meters_per_foot = 0.3048; // they give feet, system works in meters
const float degrees_per_radian = 180.0 / 3.14159; // they give radians, system works in degrees

// state tracking variables
// state of 0 = just started (need to configure angle)
// state of 1 = configured angle (need to move and check for completion)
// state of 2 = completed angle, configured straight (need to move and check for completion)
// state of 3 = completed straight
int state = 0;
int stat_print_count = 0;

void setup() {
  initialize_encoders();
  initializeI2C();
  Serial.begin(9600);  // Initialize serial communication

  Serial.println("Initializing motor shield...");
  motorShield.init();
  Serial.println("Motor shield initialized!");

  init_statistics();
}

void loop() {

  if (millis() > 1000) {
    if (state == 0) {
      if (enableAngle) {
        if (angleIsInDegrees) {
          configure_move_angle(targetAngle);
        } else {
          configure_move_angle(targetAngle * degrees_per_radian);
        }
      }
      state = 1;
    } else if (state == 1) {
      if (has_reached_target()) {
        ///delay(1000);
        if (enableDistance) {
          if (distanceIsInMeters) {
            configure_move_straight(targetDistance);
          } else {
            configure_move_straight(targetDistance * meters_per_foot);
          }
        }
        state = 2;
      }
      move_both_motors();
    } else if (state == 2) {
      if (has_reached_target()) {
        configure_move_straight(0);
        state = 3;
      }
      move_both_motors();
    } else if (state = 3) {
      Serial.println("Completed movement.");
      update_statistics(10 * voltage_max, 10 * voltage_max);
      apply_simultaneous_voltage(0, 0);
    }
    //               left, right, position, position error, voltage, velocity
    //print_statistics(true, true,  false,    true,           true,    false);

  }

  delay(100);  // Adjust delay for control loop timing
}