#pragma once

namespace newton {

  class VelocityController {
  public:
    VelocityController();

    void set_gains(float p, float i, float d);
    void set_target(float target);
    void reset();

    float filter_velocity(float raw_velocity);
    float update(float current_velocity);

    float get_filtered_velocity() const { return filtered_velocity; }
    float get_setpoint() const { return setpoint; }

  private:
    void* control_task(void* arg);
  private:
      // PID gains
    float kp = 0.08f;
    float ki = 0.4f;
    float kd = 0.004f;

    // Low-pass filter coefficient (100Hz cutoff at 1kHz sampling)
    const float alpha = 0.3799f;

    // State variables
    float integral = 0.0f;
    float prev_error = 0.0f;
    float filtered_velocity = 0.0f;
    float setpoint = 0.0f;

    // Limits
    const float max_integral = 1.0f;
    const float min_integral = -1.0f;
    const float max_output = 1.0f;
    const float min_output = -1.0f;
  };

}