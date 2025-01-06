#ifndef STEP_H
#define STEP_H

const int MOTOR_CONTROL_FREQ = 500; // Hz
const int MIN_SAMPLING_FREQ = MOTOR_CONTROL_FREQ * 2; // Nyquist criterion
const int MAX_SAMPLES = 2000; // 4 seconds of data at 500 Hz

struct StepResponse{
    unsigned long timestamp;
    unsigned long speed;
    float step;
    int pwm;
};

StepResponse stepResponseData[MAX_SAMPLES]; 

#endif// STEP_H
