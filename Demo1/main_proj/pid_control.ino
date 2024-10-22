#include "structs.h"

extern StatBuffer timestamp, left_position_error, right_position_error;

float calculate_integral_error_for_buf(struct stat_buffer *buf) {
  float error_sum = 0.0;
  for (int i = 0, n = buf->used; (i+1) < n; i++) {
    float left_val = buf->buf[i];
    float right_val = buf->buf[i+1];
    float time_diff = (timestamp.buf[i+1]) - (timestamp.buf[i]);
    if (left_val >= 0 && right_val >= 0) {
      // both points above the horizontal
      if (left_val > right_val) {
        error_sum += (time_diff * right_val) + (0.5 * time_diff * (left_val - right_val));
      } else if (right_val > left_val) {
        error_sum += (time_diff * left_val) + (0.5 * time_diff * (right_val - left_val));
      } else {
        error_sum += (time_diff * left_val);
      }
    } else if (left_val < 0 && right_val < 0) {
      // both points below the horizontal
      if (left_val < right_val) {
        error_sum += (time_diff * right_val) + (0.5 * time_diff * (left_val - right_val));
      } else if (right_val < left_val) {
        error_sum += (time_diff * left_val) + (0.5 * time_diff * (right_val - left_val));
      } else {
        error_sum += (time_diff * left_val);
      }
    } else {
      // one point above, one point below, more complex... ignore for now?
    }
  }
}

float get_integral_error(int motor_index) {
  if (motor_index == 0) {
    return calculate_integral_error_for_buf(&left_position_error);
  } else {
    return calculate_integral_error_for_buf(&right_position_error);
  }
}

float get_derivative_error(int motor_index) {
  float position_error = get_position_error(motor_index);

  float previous_position_error, delta_time;
  if (motor_index == 0 && left_position_error.used > 1) {
    previous_position_error = left_position_error.buf[left_position_error.used - 2];
    delta_time = timestamp.buf[timestamp.used - 1] - timestamp.buf[timestamp.used - 2];
  } else if (motor_index == 1 && right_position_error.used > 1) {
    previous_position_error = right_position_error.buf[right_position_error.used - 2];
    delta_time = timestamp.buf[timestamp.used - 1] - timestamp.buf[timestamp.used - 2];
  } else {
    previous_position_error = 0.0;
    delta_time = 0.0;
  }

  if (delta_time == 0.0) {
    return 0.0;
  }
  return (previous_position_error - position_error) / delta_time;
}

float pid_control(int motor_index) {
  PIDTunables tunables;
  get_pid_tunables(motor_index, &tunables);

  float pid_result = 0.0;
  if (tunables.p != 0.0) {
    pid_result += tunables.p * get_position_error(motor_index);
  }
  if (tunables.i != 0.0) {
    pid_result += tunables.i * get_integral_error(motor_index);
  }
  if (tunables.d > 0.0) {
    pid_result += tunables.d * get_derivative_error(motor_index);
  }

  return pid_result;
};
