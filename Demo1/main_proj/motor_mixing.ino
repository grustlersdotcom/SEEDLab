
// see https://cookierobotics.com/066/ for a simple intro to motor mixing theory
float motor_mix(int motor_index, float desired_thrust, float desired_torque) {
    if (motor_index) {
        return desired_thrust + desired_torque;
    } else {
        return desired_thrust - desired_torque;
    }
}

float linear_motor_mix(int motor_index, float desired_thrust) {

}

float rotational_motor_mix(int motor_index, float desired_torque) {

}
