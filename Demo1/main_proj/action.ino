#include "structs.h"

extern float initial_left_pos;
extern float initial_right_pos;
extern float left_target_pos;
extern float right_target_pos;

const float voltage_max = 4.0;
const float min_effective_voltage = 1.7;
const float min_effective_turning_voltage = 2.5;
const float straightness_correction_factor = 0.99; // limit ahead wheel's voltage to this factor of other wheel target voltage
const float allowed_delta_threshold = 0.01; // multiplied by circumference to get threshold distance delta
const float acceleration_ramp_length = 2.0;
const float wheel_circumference = 0.47877872; // meters
const float target_reached_threshold = 0.02; // 2% == good enough to declare done

// move mode true = straight, false = angle
bool move_mode = true;

float apply_voltage(int motor_index, float target_voltage) {
  float target_units = target_voltage * (255.0 / voltage_max);
  if (motor_index == 0) {
    motorShield.setM2Speed(target_units);
    update_statistics(target_voltage, 10 * voltage_max);
  } else {
    motorShield.setM1Speed(target_units);
    update_statistics(10 * voltage_max, target_voltage);
  }
}

void apply_simultaneous_voltage(float left_target_voltage, float right_target_voltage) {
  motorShield.setM2Speed(left_target_voltage * (255.0 / voltage_max));
  motorShield.setM1Speed(right_target_voltage * (255.0 / voltage_max));
  update_statistics(left_target_voltage, right_target_voltage);
}

void move_one_motor(int motor_index) {
  float target_voltage = get_target_voltage(motor_index);
  apply_voltage(motor_index, target_voltage);
}

void move_both_motors() {
  if (move_mode) {
    move_straight();
  } else {
    move_angle();
  }
}

void move_straight() {
  // get raw voltage targets
  float left_target_voltage = get_target_voltage(0);
  float right_target_voltage = get_target_voltage(1);

  // apply limiters depending on straightness of path
  float left_current_position = get_motor_position(0);
  float right_current_position = get_motor_position(1);
  float left_target_replacement = straightness_correction_factor * right_target_voltage;
  float right_target_replacement = straightness_correction_factor * left_target_voltage;
  float left_travelled = left_current_position - initial_left_pos;
  float right_travelled = right_current_position - initial_right_pos;
  if (left_travelled > (right_travelled + (allowed_delta_threshold * wheel_circumference)) && left_target_voltage > left_target_replacement) {
    left_target_voltage = left_target_replacement;
  } else if (right_travelled > (left_travelled + (allowed_delta_threshold * wheel_circumference)) && right_target_voltage > right_target_replacement) {
    right_target_voltage = right_target_replacement;
  }

  // apply
  apply_simultaneous_voltage(left_target_voltage, right_target_voltage);
}

void move_angle() {
  float left_target_voltage = get_target_voltage(0);
  float right_target_voltage = get_target_voltage(1);
  apply_simultaneous_voltage(left_target_voltage, right_target_voltage);
}

void configure_move_straight(float meters) {
  initial_left_pos = get_motor_position(0);
  initial_right_pos = get_motor_position(1);
  left_target_pos = initial_left_pos + meters;
  right_target_pos = initial_right_pos + meters;
  move_mode = true;
}

void configure_move_angle(float angle) {
  // correct for negatives
  while (angle < 0) {
    angle = 360 + angle;
  }

  initial_left_pos = get_motor_position(0);
  initial_right_pos = get_motor_position(1);

  // choose direction
  // moving 90 deg. is about 1/2 in opposite direction on each wheel
  if (angle < 180) {
    // turn counterclockwise
    float half_turn_portion = angle / 180.0;
    left_target_pos = initial_left_pos - (half_turn_portion * wheel_circumference);
    right_target_pos = initial_right_pos + (half_turn_portion * wheel_circumference);
  } else {
    // turn clockwise
    float half_turn_portion = (360.0 - angle) / 180.0;
    left_target_pos = initial_left_pos + (half_turn_portion * wheel_circumference);
    right_target_pos = initial_right_pos - (half_turn_portion * wheel_circumference);
  }

  move_mode = false;
}

bool has_reached_target() {
  float left_travelled = abs(get_motor_position(0) - initial_left_pos);
  float right_travelled = abs(get_motor_position(1) - initial_right_pos);
  float left_should_travel_total = abs(left_target_pos - initial_left_pos);
  float right_should_travel_total = abs(right_target_pos - initial_right_pos);
  float left_delta = abs(left_travelled - left_should_travel_total);
  float right_delta = abs(right_travelled - right_should_travel_total);

  float threshold;
  if (move_mode) {
    threshold = 0.005525; // 0.375 in per wheel tolerance going straight
  } else {
    threshold = 0.02; // 0.1 in per wheel tolerance while turning
  }
  return (left_delta < threshold) && (right_delta < threshold);

  // 2%
  // return (left_delta <= (target_reached_threshold * left_should_travel_total)) && (right_delta <= (target_reached_threshold * right_should_travel_total));

  // 0.05m
  // return (abs(get_motor_position(0) - left_target_pos) < 0.05) && (abs(get_motor_position(1) - right_target_pos) < 0.05);
}

float get_target_voltage(int motor_index) {
  float target_velocity = pid_control(motor_index);
  float current_velocity = get_current_velocity(motor_index);
  float target_voltage = feedback_gain_cell(motor_index, target_velocity - current_velocity);

  // map target_voltage to apply various effects like ramping on, etc
  float original_target_voltage = target_voltage;
  float min_effective = get_min_effective_voltage();
  // accidentally tuned with acceleration ramp on for angle, so leave it on always
  //if (move_mode) {
    target_voltage = acceleration_ramp(motor_index, target_voltage);
  //}
  if (abs(target_voltage) < min_effective && abs(original_target_voltage) >= min_effective) {
    if (original_target_voltage < 0) {
      target_voltage = -1 * min_effective;
    } else {
      target_voltage = min_effective;
    }
  }

  return constrain(target_voltage, -1.0 * voltage_max, voltage_max);
}

float acceleration_ramp(int motor_index, float target_voltage) {
  float position_delta = abs(get_motor_position(motor_index) - get_motor_initial_position(motor_index));
  if (position_delta < acceleration_ramp_length) {
    float adjustment_factor = pow((position_delta / acceleration_ramp_length), 2);;
    return adjustment_factor * target_voltage;
  }
  return target_voltage;
}

float feedback_gain_cell(int motor_index, float velocity_error) {
  PIDTunables tunables;
  get_pid_tunables(motor_index, &tunables);
  return velocity_error * tunables.p;
}
