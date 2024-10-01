extern DualMC33926MotorShield motorShield;
extern volatile uint8_t quadrant;  // Quadrant value received from I2C
extern float leftWheelDistance, rightWheelDistance;  // Distances from encoders
extern volatile uint8_t msgLength;

// PID gains for position control
float Kp_pos = 5;  // Proportional gain
float Ki_pos = 20; // Integral gain

// Desired positions for both wheels (left and right)
int desired_pos[2] = {0, 0};
float integral_error[2] = {0, 0}; // Integral error for both wheels

// Function to check quadrant and move wheels accordingly
void checkQuadrantAndMove() {
  msgLength = 0;
  // Set desired positions based on the received quadrant
  switch (quadrant) {
    case 1:  // NE
      desired_pos[0] = 0;  // Left wheel target position
      desired_pos[1] = 0;  // Right wheel target position
      break;
    case 2:  // NW
      desired_pos[0] = 0;  // Left wheel target position
      desired_pos[1] = 1;  // Right wheel target position
      break;
    case 3:  // SW
      desired_pos[0] = 1;  // Left wheel target position
      desired_pos[1] = 1;  // Right wheel target position
      break;
    case 4:  // SE
      desired_pos[0] = 1;  // Left wheel target position
      desired_pos[1] = 0;  // Right wheel target position
      break;
    default:
      Serial.println("Invalid quadrant received");
      return;
  }

  // Call the function to move the wheels to the desired positions using the PI controller
  moveWheelsToPosition();
}


void moveWheelsToPosition() {
  for (int i = 0; i < 2; i++) {
    if (desired_pos[i] == 0) {
      Serial.print("Wheel ");
      Serial.print(i);
      Serial.println(" moved to 0");
      if (i == 0) {
        while ((leftWheelDistance) != (0.00 || -0.00)) {
          updateWheelMetrics();
          if (leftWheelDistance > 0.00) {
            motorShield.setM2Speed(-75);
          }
          if(leftWheelDistance < 0.00) {
            motorShield.setM2Speed(75);
          }
          if (leftWheelDistance == 0.00) {
            motorShield.setM2Speed(0);
            break;
          }
        }
      }
      if (i == 1) {
        while ((rightWheelDistance) != (0.00 || -0.00)) {
          updateWheelMetrics();
          if (rightWheelDistance > 0.00) {
            motorShield.setM1Speed(-75);
          }
          if(rightWheelDistance < 0.00) {
            motorShield.setM1Speed(75);
          }
          if (rightWheelDistance == 0.00) {
            motorShield.setM1Speed(0);
            break;
          }
        }
      }
    }
  
    if(desired_pos[i] == 1) {
      Serial.print("Wheel ");
      Serial.print(i);
      Serial.println(" moved to 1");
      if (i == 0) {
        while ((leftWheelDistance) != (0.50 || -0.50)) {
          updateWheelMetrics();
          if (leftWheelDistance > 0.50) {
            motorShield.setM2Speed(-75);
          }
          if(leftWheelDistance < 0.49) {
            motorShield.setM2Speed(75);
          }
          if (leftWheelDistance > 0.49) {
            motorShield.setM2Speed(0);
            break;
          }
        }
      }
      if (i == 1) {
        while ((rightWheelDistance) != (0.50 || -0.50)) {
          updateWheelMetrics();
          if (rightWheelDistance > 0.50) {
            motorShield.setM1Speed(-75);
          }
          if(rightWheelDistance < 0.49) {
            motorShield.setM1Speed(75);
          }
          if (rightWheelDistance > 0.49) {
            motorShield.setM1Speed(0);
            break;
          }
        }
      }
    }
  }
}

/*
void moveWheelsToPosition() {
  float actual_pos[2] = {leftWheelDistance, rightWheelDistance};

  for (int i = 0; i < 2; i++) {
    // Calculate the position error
    float pos_error = desired_pos[i] - actual_pos[i];

    // Update integral error
    integral_error[i] += pos_error;

    // PI control for desired speed
    float desired_speed = Kp_pos * pos_error + Ki_pos * integral_error[i];

    // Clamp the desired speed to the motor shield's range (-400 to 400)
    desired_speed = constrain(desired_speed, -255, 255);

    // Debugging output to see the speed values
    Serial.print("Wheel ");
    Serial.print(i);
    Serial.print(" - Actual position: ");
    Serial.print(actual_pos[i]);
    Serial.print(" - Desired position: ");
    Serial.print(desired_pos[i]);
    Serial.print(" - Desired speed (clamped): ");
    Serial.println(desired_speed);

    // Control motor speed and direction using motor shield
    control_motor(i, desired_speed);
  }
}

void control_motor(int motor_index, float speed) {
  int motorSpeed = constrain(speed, -255, 255);  // Constrain speed to motor's max speed range (-255 to 255)

  if (motor_index == 0) {
    Serial.print("Left motor speed: ");
    Serial.println(motorSpeed);
    motorShield.setM1Speed(motorSpeed);  // Set speed for Motor 1 (left wheel)
    if (leftWheelDistance > 0.5) {
      Serial.println("made it");
      motorShield.setM2Speed(-255);
      return;
    }

  } else {
    Serial.print("Right motor speed: ");
    Serial.println(motorSpeed);
    motorShield.setM2Speed(motorSpeed);  // Set speed for Motor 2 (right wheel)
  }
}
*/