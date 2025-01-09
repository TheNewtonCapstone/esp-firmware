#include "velocity_controller.h"
#include <algorithm>

namespace newton {

VelocityController::VelocityController() = default;

void VelocityController::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
    filtered_velocity = 0.0f;
}

void VelocityController::set_target(float target) {
    setpoint = target;
}

void VelocityController::set_gains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

float VelocityController::filter_velocity(float raw_velocity) {
    filtered_velocity = alpha * raw_velocity + (1.0f - alpha) * filtered_velocity;
    return filtered_velocity;
}

float VelocityController::update(float current_velocity) {
    float filtered = filter_velocity(current_velocity);
    
    float error = setpoint - filtered;
    
    float p_term = kp * error;
    
    integral += ki * error * 0.001f; // 0.001 because dt = 1ms
    integral = std::min(std::max(integral, min_integral), max_integral);
    float i_term = integral;
    
    float derivative = (error - prev_error) * 1000.0f; // *1000 because dt = 1ms
    float d_term = kd * derivative;
    
    prev_error = error;
    
    float output = p_term + i_term + d_term;
    return std::min(std::max(output, min_output), max_output);
}

} 