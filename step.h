#ifndef STEP_H
#define STEP_H

const int MOTOR_CONTROL_FREQ = 700; // Hz
const int MIN_SAMPLING_FREQ = MOTOR_CONTROL_FREQ * 2; // Nyquist criterion
const int MAX_SAMPLES = 4000; // 4 seconds of data at 500 Hz

struct StepResponse{
    unsigned long timestamp;
    unsigned long count;
    float position;
    float velocity;
    float step;
    int pwm;
};

StepResponse stepResponseData[MAX_SAMPLES]; 

#endif// STEP_H
