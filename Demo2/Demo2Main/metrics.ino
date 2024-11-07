#include "structs.h"

extern volatile int encoder_left_count;
extern volatile int encoder_right_count;
extern const float distance_per_encoder_tick;
extern const float min_effective_voltage;
extern const float min_effective_turning_voltage;

float left_target_pos, right_target_pos;
float initial_left_pos, initial_right_pos;

StatBuffer timestamp,
  left_position, right_position,
  left_voltage, right_voltage,
  left_position_error, right_position_error;

void init_single_buffer(struct stat_buffer *buf) {
  buf->size = 30;
  buf->used = 1;
  buf->buf[0] = 0;
}

void shift_buffer(struct stat_buffer *buf, int shift_count) {
  for (int i = shift_count, n = buf->used; i < n; i++) {
    buf->buf[i - shift_count] = buf->buf[i];
  }
  buf->used -= shift_count;
}

void add_to_buffer(struct stat_buffer *buf, float value) {
  if (buf->used == buf->size) {
    shift_buffer(buf, 10);
  }
  buf->buf[buf->used] = value;
  buf->used += 1;
}

void init_statistics() {
  left_target_pos = 0.0;
  right_target_pos = 0.0;
  init_single_buffer(&timestamp);
  init_single_buffer(&left_position);
  init_single_buffer(&right_position);
  init_single_buffer(&left_voltage);
  init_single_buffer(&right_voltage);
  init_single_buffer(&left_position_error);
  init_single_buffer(&right_position_error);
}

void update_statistics(float left_voltage_measurement, float right_voltage_measurement) {
  add_to_buffer(&timestamp, millis());
  add_to_buffer(&left_position, get_motor_position(0));
  add_to_buffer(&right_position, get_motor_position(1));

  // simply copy previous voltage values if 10*max provided
  if (left_voltage_measurement == 10*voltage_max) {
    add_to_buffer(&left_voltage, left_voltage.buf[left_voltage.used - 1]);
  } else {
    add_to_buffer(&left_voltage, left_voltage_measurement);
  }

  if (right_voltage_measurement == 10*voltage_max) {
    add_to_buffer(&right_voltage, right_voltage.buf[right_voltage.used - 1]);
  } else {
    add_to_buffer(&right_voltage, right_voltage_measurement);
  }

  // calculate position errors
  add_to_buffer(&left_position_error, left_target_pos - (left_position.buf[left_position.used - 1]));
  add_to_buffer(&right_position_error, right_target_pos - (right_position.buf[right_position.used - 1]));
}

void print_statistics(bool left, bool right, bool position, bool position_error, bool voltage, bool velocity) {
  if ((left || right) && (position || position_error || voltage || velocity)) {
    Serial.print("time:");
    Serial.print(timestamp.buf[timestamp.used-1]);
    Serial.print(", ");
  }
  if (left) {
    if (position) {
      Serial.print("leftPos:");
      Serial.print(left_position.buf[left_position.used-1]);
      Serial.print(", ");
    }
    if (position_error) {
      Serial.print("leftPosErr:");
      Serial.print(left_position_error.buf[left_position_error.used-1]);
      Serial.print(", ");
    }
    if (voltage) {
      Serial.print("leftVolt:");
      Serial.print(left_voltage.buf[left_voltage.used-1]);
      Serial.print(", ");
    }
    if (velocity) {
      Serial.print("leftVelocity:");
      Serial.print(get_current_velocity(0));
      Serial.print(", ");
    }
  }

  if (right) {
    if (position) {
      Serial.print("rightPos:");
      Serial.print(right_position.buf[right_position.used-1]);
      Serial.print(", ");
    }
    if (position_error) {
      Serial.print("rightPosErr:");
      Serial.print(right_position_error.buf[right_position_error.used-1]);
      Serial.print(", ");
    }
    if (voltage) {
      Serial.print("rightVolt:");
      Serial.print(right_voltage.buf[right_voltage.used-1]);
      Serial.print(", ");
    }
    if (velocity) {
      Serial.print("rightVelocity:");
      Serial.print(get_current_velocity(1));
      Serial.print(", ");
    }
  }
  if ((left || right) && (position || position_error || voltage || velocity)) {
    Serial.println("");
  }
}

float get_motor_position(int motor_index) {
  if (motor_index == 0) {
    return encoder_left_count * distance_per_encoder_tick;
  } else {
    return encoder_right_count * distance_per_encoder_tick;
  }
}

float get_target_motor_position(int motor_index) {
  if (motor_index == 0) {
    return left_target_pos;
  } else {
    return right_target_pos;
  }
}

float get_motor_initial_position(int motor_index) {
  if (motor_index == 0) {
    return initial_left_pos;
  } else {
    return initial_right_pos;
  }
}

void get_pid_tunables(int motor_index, struct pid_tunables *tunables) {
  if (move_mode) {
    if(motor_index == 0){
      // Ziegler-Nichols method:
      // Determined Kc to be 10.47, Pc to be 811ms
      // Control Scheme: P,      I,       D
      // P Controller:   5.235,  0,       0
      // PI Controller:  4.7115, 675.833, 0
      // PID Controller: 6.282,  405.5,   101.375
      tunables->p = 6.282;
      tunables->i = 405.5;
      tunables->d = 101.375;
    } else {
      // Ziegler-Nichols method:
      // Determined Kc to be 10.90, Pc to be 815ms
      // Control Scheme: P,    I,       D
      // P Controller:   5.45,  0,       0
      // PI Controller:  4.905, 679.167, 0
      // PID Controller: 6.54,  407.5,   101.875
      tunables->p = 6.54;
      tunables->i = 407.5;
      tunables->d = 101.875;
    }
  } else {
    // turning requires different tuning due to low distances
    if(motor_index == 0){
      // Ziegler-Nichols method:
      // Determined Kc to be 24.75, Pc to be 723ms
      // Control Scheme: P,       I,       D
      // P Controller:   12.375,  0,       0
      // PI Controller:  11.1375, 602.5,   0
      // PID Controller: 14.85,   361.5,   90.375
      tunables->p = 14.85;
      tunables->i = 361.5;
      tunables->d = 90.375;
    } else {
      // Ziegler-Nichols method:
      // Determined Kc to be 23.87, Pc to be 851ms
      // Control Scheme: P,       I,       D
      // P Controller:   11.935,  0,       0
      // PI Controller:  10.7415, 709.167, 0
      // PID Controller: 14.322,  425.5,   106.375
      tunables->p = 14.322;
      tunables->i = 425.5;
      tunables->d = 106.375;
    }
  }
}


float straightness_pid(float s_error){
  static float integral_error = 0.0;
  static float prevois_error = 0.0;
  float kp = 1.55;
  float ki = 0.4;//.6
  float kd = 2.25;

  float distance_to_target = abs(left_target_pos - get_motor_position(0));
  if (distance_to_target < 1.5) {
    ki *= distance_to_target / 1.5; // Smoothly scale down ki as you approach the target
    kd *= 1.0 + (1.5 - distance_to_target) * 0.2; // Gradually increase kd closer to the target
  } else if (distance_to_target > 6.0) {
    integral_error = 0.0;  // Reset integral error at the beginning to avoid any initial buildup
  }

  Serial.println(ki);
  Serial.println(kd);
  Serial.println(left_target_pos);
  Serial.println(get_motor_position(0));
  if (abs(s_error) < 0.01) { // Ignore very small errors
    s_error = 0.0;
  }

  integral_error += s_error;
  //integral_error = constrain(integral_error, -3, 3);
  float derivative_error= s_error - prevois_error;
  prevois_error = s_error;
  float pid_out = (kp * s_error) + (ki * integral_error) + (kd*derivative_error);
  return constrain(pid_out, -1, 1);

}


float get_current_velocity(int motor_index) {
  // get penultimate time and position
  float penultimate_time = 0;
  float penultimate_position = 0;

  if (timestamp.used > 1) {
    penultimate_time = timestamp.buf[timestamp.used - 2];
  }

  if (motor_index == 0 && left_position.used > 1) {
    penultimate_position = left_position.buf[left_position.used - 2];
  } else if (motor_index == 1 && right_position.used > 1) {
    penultimate_position = right_position.buf[right_position.used - 2];
  }

  // get last time and position
  float ultimate_time = timestamp.buf[timestamp.used - 1];
  float ultimate_position;
  if (motor_index == 0) {
    ultimate_position = left_position.buf[left_position.used - 1];
  } else {
    ultimate_position = right_position.buf[right_position.used - 1];
  }

  // perform calculations
  float delta_time = ultimate_time - penultimate_time;
  float delta_position = ultimate_position - penultimate_position;
  if (delta_time == 0.0) {
    // TODO is this safe?
    return 0.0;
  }
  return delta_position / delta_time;
}

float get_current_voltage(int motor_index) {
  if (motor_index == 0) {
    return left_voltage.buf[left_voltage.used - 1];
  } else {
    return right_voltage.buf[right_voltage.used - 1];
  }
}

float get_position_error(int motor_index) {
  if (motor_index == 0) {
    return left_position_error.buf[left_position_error.used - 1];
  } else {
    return right_position_error.buf[right_position_error.used - 1];
  }
}

float get_min_effective_voltage() {
  if (move_mode) {
    return min_effective_voltage;
  } else {
    return min_effective_turning_voltage;
  }
}