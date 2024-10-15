#include <Encoder.h>
#include <DualMC33926MotorShield.h>
//For Arduino Uno with ATmega328 microcontroller

DualMC33926MotorShield motorShield;

//PID  constants
float Kp_pos = 1.0;
float Ki_pos = 0.5;
float Kd_pos = 0.1;
float maxVoltage = 255.0;

//Arrays for PID
float pos_error[2] = {0};
float integral_error[2] = {0};
float prev_pos_error[2] = {0};
float desired_speed[2] = {0};
float actual_speed[2] = {0};
float Voltage[2] = {0};

//Encoder pins for left and right wheels
Encoder enc1(2, 5);  //Left encoder
Encoder enc2(3, 6);  //Right encoder

//Robot constants
const float wheelRadius = 0.1524;  //Wheel radius in meters
const float wheelBase = 0.3048;    //Distance between wheels
const float wheelCircumference = 2 * PI * wheelRadius;  //Circumference of the wheel
const int CPR = 800;  //Encoder counts per revolution

//Position tracking variables
float x = 0.0, y = 0.0, theta = 0.0;  //Robot position and orientation
float leftWheelDistance = 0.0, rightWheelDistance = 0.0;

//Movement variables
float distanceInInches = 36;  //Desired distance in inches
float turnAngle = 0;         //Desired turn angle in degrees

void resetEncoders();
void moveStraight(float distanceInInches);
void turnDegrees(float degrees);
void updateWheelMetrics();
float calculateDistance(long encoderCount);
float calculateAngle(long leftCount, long rightCount);
void computePID(int index, float desired_pos[], float actual_pos[], float deltaTime);
void computeTurnPID(float desiredAngle, float actualAngle, float deltaTime);

void setup() {
  motorShield.init();
  Serial.begin(9600);
}

void loop() {
  static unsigned long lastTime = millis();
  const float desiredLoopTime = 50;  //Set loop to run every 50 ms

  if (millis() - lastTime >= desiredLoopTime) {
    lastTime += desiredLoopTime;  //Ensure exactly 50 ms between loops

    if (turnAngle != 0) {
      turnDegrees(turnAngle);  //Perform turn if angle is non-zero
    }
    moveStraight(distanceInInches);  //Then move straight
  }
}

void resetEncoders() {
  enc1.write(0);
  enc2.write(0);
}

void moveStraight(float distanceInInches) {
  float distanceInMeters = distanceInInches * 0.0254;
  float tolerance = 0.01;  //Precision tolerance in meters
  float decelerationDistance = 0.1;  //Decelerate over 10 cm

  resetEncoders();
  unsigned long lastTime = millis();

  while (true) {
    updateWheelMetrics();  //Update wheel distances

    //Stopping condition based on tolerance
    if (fabs(leftWheelDistance - distanceInMeters) < tolerance && fabs(rightWheelDistance - distanceInMeters) < tolerance) {
      motorShield.setM1Speed(0);
      motorShield.setM2Speed(0);
      break;
    } else {
      //PID control for each motor
      float desired_pos[] = {distanceInMeters, distanceInMeters};  //Target positions for both wheels
      float actual_pos[] = {leftWheelDistance, rightWheelDistance};

      unsigned long currentTime = millis();
      float deltaTime = (currentTime - lastTime) / 1000.0;

      //Deceleration logic using non-linear scaling
      if (distanceInMeters - leftWheelDistance < decelerationDistance) {
        float decelerationFactorLeft = pow(((distanceInMeters - leftWheelDistance) / decelerationDistance), 2);  //Quadratic scaling
        Voltage[0] *= decelerationFactorLeft;
      }
      if (distanceInMeters - rightWheelDistance < decelerationDistance) {
        float decelerationFactorRight = pow(((distanceInMeters - rightWheelDistance) / decelerationDistance), 2);  //Quadratic scaling
        Voltage[1] *= decelerationFactorRight;
      }

      //Compute PID for both motors
      computePID(0, desired_pos, actual_pos, deltaTime);  //Left motor
      computePID(1, desired_pos, actual_pos, deltaTime);  //Right motor

      //Set motor speeds after constraining voltage values
      Voltage[0] = constrain(Voltage[0], -maxVoltage, maxVoltage);
      Voltage[1] = constrain(Voltage[1], -maxVoltage, maxVoltage);
      motorShield.setM1Speed(Voltage[0]);  //Left motor
      motorShield.setM2Speed(Voltage[1]);  //Right motor

      lastTime = currentTime;
    }
  }
}

void turnDegrees(float degrees) {
  float radians = degrees * PI / 180.0;
  float tolerance = 0.01;  //Precision tolerance in radians

  resetEncoders();
  unsigned long lastTime = millis();

  while (true) {
    updateWheelMetrics();  //Update wheel distances
    float currentAngle = calculateAngle(enc1.read(), enc2.read());

    //Stopping condition based on tolerance
    if (fabs(currentAngle - radians) < tolerance) {
      motorShield.setM1Speed(0);
      motorShield.setM2Speed(0);
      break;
    } else {
      unsigned long currentTime = millis();
      float deltaTime = (currentTime - lastTime) / 1000.0;

      //Apply PID control for accurate turn
      computeTurnPID(radians, theta, deltaTime);

      if (degrees > 0) {
        //Positive turn: Reverse left motor, forward right motor
        motorShield.setM1Speed(constrain(-Voltage[0], -maxVoltage, maxVoltage));  //Left motor reverse
        motorShield.setM2Speed(constrain(Voltage[1], -maxVoltage, maxVoltage));   //Right motor forward
      } else {
        //Negative turn: Forward left motor, reverse right motor
        motorShield.setM1Speed(constrain(Voltage[0], -maxVoltage, maxVoltage));   //Left motor forward
        motorShield.setM2Speed(constrain(-Voltage[1], -maxVoltage, maxVoltage));  //Right motor reverse
      }

      lastTime = currentTime;  //Update after PID computation
    }
  }
}

void updateWheelMetrics() {
  long leftCount = enc1.read();
  long rightCount = enc2.read();

  float distancePerTick = (2.0 * PI * wheelRadius) / CPR;
  leftWheelDistance = leftCount * distancePerTick;
  rightWheelDistance = rightCount * distancePerTick;

  //Update position
  float linearVelocity = (leftWheelDistance + rightWheelDistance) / 2.0;
  float angularVelocity = (rightWheelDistance - leftWheelDistance) / wheelBase;

  //Use micros() for higher precision time measurement
  unsigned long currentTime = micros();
  static unsigned long lastTime = 0;
  float timeDelta = (currentTime - lastTime) / 1000000.0;  //Convert microseconds to seconds
  lastTime = currentTime;

  //Update global position and orientation
  x += linearVelocity * timeDelta * cos(theta);
  y += linearVelocity * timeDelta * sin(theta);
  theta += angularVelocity * timeDelta;

  //Calculate actual speed for PID control
  actual_speed[0] = leftWheelDistance / timeDelta;  //Left wheel speed
  actual_speed[1] = rightWheelDistance / timeDelta;  //Right wheel speed
}

float calculateDistance(long encoderCount) {
  return (encoderCount / CPR) * wheelCircumference;  //Convert encoder counts to meters traveled
}

float calculateAngle(long leftCount, long rightCount) {
  float leftDistance = calculateDistance(leftCount);
  float rightDistance = calculateDistance(rightCount);
  return (rightDistance - leftDistance) / wheelBase;  //Calculate turning angle in radians
}

void computePID(int index, float desired_pos[], float actual_pos[], float deltaTime) {
  //Calculate position error
  pos_error[index] = desired_pos[index] - actual_pos[index];
  
  //Anti-windup: Only accumulate integral error when the voltage is not saturated
  if (Voltage[index] < maxVoltage && Voltage[index] > -maxVoltage) {
    integral_error[index] += pos_error[index] * deltaTime;
  }

  //Derivative term
  float derivative = (pos_error[index] - prev_pos_error[index]) / deltaTime;
  
  //PID control: Calculate the desired speed
  desired_speed[index] = Kp_pos * pos_error[index] + Ki_pos * integral_error[index] + Kd_pos * derivative;

  //Adjust voltage based on speed error and constrain to allowable range
  Voltage[index] = Kp_pos * (desired_speed[index] - actual_speed[index]);
  Voltage[index] = constrain(Voltage[index], -maxVoltage, maxVoltage);  //Clamp to allowable range

  //Store the previous error for the next cycle
  prev_pos_error[index] = pos_error[index];
}

void computeTurnPID(float desiredAngle, float actualAngle, float deltaTime) {
  float error = desiredAngle - actualAngle;
  static float integralLeft = 0, integralRight = 0;

  //Left motor PID with anti-windup
  if (Voltage[0] < maxVoltage && Voltage[0] > -maxVoltage) {
    integralLeft += error * deltaTime;
  }
  float derivativeLeft = (error - prev_pos_error[0]) / deltaTime;
  Voltage[0] = Kp_pos * error + Ki_pos * integralLeft + Kd_pos * derivativeLeft;  //Left motor voltage

  //Right motor PID with anti-windup
  if (Voltage[1] < maxVoltage && Voltage[1] > -maxVoltage) {
    integralRight += error * deltaTime;
  }
  float derivativeRight = (error - prev_pos_error[1]) / deltaTime;
  Voltage[1] = Kp_pos * error + Ki_pos * integralRight + Kd_pos * derivativeRight;  //Right motor voltage

  prev_pos_error[0] = error;
  prev_pos_error[1] = error;
}
