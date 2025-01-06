// #include <SimpleFOC.h>
#include <ESP32Encoder.h>

#include <ESP32Servo.h>

#include "step.h"

#define motorPin 12
#define A 22
#define B 23

const float VEL_PERIOD = 0.0005f;

const int MAX_INTEGRAL = 1000;
const int LOG_INTERVAL = 100;

const float MIN_SPEED_PWM = 0;
const float MAX_SPEED_PWM = 400;

Servo servo;
ESP32Encoder encoder;

// Encoder encoder(A, B, 2048);

// void doA() { encoder.handleA(); }
// void doB() { encoder.handleB(); }

// position
float prev_pos = 0;
float curr_pos = 0;


// Velocity control variables
float tar_vel = 0.f;
float curr_vel = 0.f;
float prev_vel_error = 0.f;
float vel_integral = 0;
unsigned long prev_time = 0;




// Much lower gains and added derivative
struct PID {
  float kp;
  float ki;
  float kd;
};

PID vel_pid = { 10.0f, 0.01, 5 };


String str_pos = "0";

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(115200);


  encoder.attachHalfQuad(A, B);
  encoder.setCount(0);
  // encoder.quadrature = Quadrature::ON;
  // encoder.pullup = Pullup::USE_EXTERN;
  // encoder.init();
  // encoder.enableInterrupts(doA, doB);

  servo.attach(motorPin, 1100, 1900);
  servo.writeMicroseconds(1500);
  delay(2000);
  Serial.printf("step, speed, pwm\n");
  prev_time = micros();
  // prev_pos = encoder.getAngle();
}

void loop() {
  //     if(Serial.available() > 0) {
  //         str_pos = Serial.readStringUntil('\r\n');
  //         tar_vel = str_pos.toFloat();
  //         Serial.printf("New target velocity: %.2f\n", tar_vel);
  //         vel_integral = 0; // Reset integral when target changes
  //     }

  //     encoder.update();
  //     unsigned long curr_time = micros();
  //     float dt = (curr_time - prev_time) / 1000000.0f;
  //     float alpha = 0.9; // Smoothing factor
  // curr_vel = alpha * curr_vel + (1 - alpha) * encoder.getVelocity();


  //     // // Calculate PID output
  //     float error = tar_vel - curr_vel;

  //     // Only integrate if we're close to target
  //     if(abs(error) < 100) {
  //         vel_integral += error * dt;
  //         vel_integral = constrain(vel_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  //     } else {
  //         vel_integral = 0; // Reset integral when error is large
  //     }

  //     float derivative = (error - prev_vel_error) / dt;

  //     float pid_output = vel_pid.kp * error +
  //                       vel_pid.ki * vel_integral +
  //                       vel_pid.kd * derivative;

  //     // // Smooth mapping with larger dead zone
  //     int pwm_value = 1520;

  //     if(abs(pid_output) > 20) { // Larger dead zone
  //         if(pid_output > 0) {
  //             // Exponential mapping for smoother control
  //             float normalized = constrain(pid_output / 800.0f, 0, 1);
  //             pwm_value = 1500 + (int)(75.0f * normalized * normalized);
  //             // pwm_value = 1500 + pid_output;
  //         } else {
  //             float normalized = constrain(-pid_output / 800.0f, 0, 1);
  //             pwm_value = 1500 - (int)(75.0f * normalized * normalized);
  //             // pwm_value = 1500 - pid_output;
  //         }
  //     }

  //     servo.writeMicroseconds(pwm_value);

  //     prev_vel_error = error;
  //     // prev_pos = new_pos;
  //     prev_time = curr_time;

  // setSpeed(0.2);
  // static unsigned long lastLog = 0;
  // unsigned long curr_time = millis();
  // unsigned long dt = curr_time - lastLog;
  // float speed = getVelocity();
  // if(millis() - lastLog >= LOG_INTERVAL) {
  //     // Serial.printf("Pos: %.2f, Vel: %.2f RPM, Target: %.2f RPM, Error: %.2f, PID: %.2f, PWM: %d\n",
  //     //              new_pos * 180/PI,
  //     //              curr_vel,
  //     //              tar_vel,
  //     //              error,
  //     //              pid_output,
  //     //              pwm_value);
  //     // Serial.printf("Dt: %f, Velocity : %f, Target vel: %f, pid_output %f, PWM : %d\n", dt, curr_vel,tar_vel, pid_output, pwm_value);
  // // Serial.printf("Dt : %lu, %lu %f\n ", dt,lastLog,speed);
  //     // lastLog = millis();
  //     lastLog = curr_time;
  // }
  // step response
  for(float step = 0.1f; step <= .9f; step += 0.1f){
    Serial.printf("step :%f\n", step);
    int sampleCount = measureStepResponse(step);
    Serial.printf("Step: %.2f, Samples: %d\n\n", step, sampleCount);
    delay(2000);
  }
  // encoder.update();
  // encoder.getVelocity();
  // setSpeed(0);
  // delay(1000);
  // Serial.println("starting");
  // int sampleCount = measureStepResponse(0.4);
  // setSpeed(0);
  // Serial.printf("Step: %.2f, Samples: %d\n\n", 0.4, sampleCount);
  vTaskDelay(1);
}

// float updateVelocity(float dt){
//     curr_vel = getPosition();
//     // if(dt <= VEL_PERIOD)
//     //   return curr_vel;
//     // Calculate raw velocity first
//     float vel = (curr_vel - prev_pos) / dt;
//     // Stronger filtering
//     // curr_vel = vel;
//     curr_vel = 0.9 * curr_vel + 0.1 * vel;

//     prev_pos = new_pos;
//     return curr_vel;
// }

float getPosition() {
  long counts = encoder.getCount() / 2;
  double dposition = counts / 5.12;
  return (float)dposition * (3.14159/180.0);

    // long counts = encoder.getCount();
    // // 8192 counts = 360 degrees
    // // So divide by (8192/360) = 22.755... to get degrees
    // float degrees = counts / 22.755555555556;
    // // Convert degrees to radians
    // return degrees;
    // return (float)degrees * (3.14159/180.0);
  
  // return encoder.getAngle() * 180.f / 3.1415f;
}

// float getVelocity(){
//   const float alpha = 0.8f; 
//   static unsigned long prev_time = 0;
//   unsigned long curr_time = millis();
//   static float prev_vel = 0;
//   static float filtered_vel = 0;
//   float dt = (float) (curr_time - prev_time) / 1000.f;
  
//   if(dt <= VEL_PERIOD)
//     return filtered_vel;  


//   static float prev_pos = 0;
//   float curr_pos = getPosition();
//   float raw_vel =  ( curr_pos - prev_pos) / dt;

//   // low pass filter ishh
// filtered_vel = alpha * filtered_vel + (1.0f - alpha) * raw_vel;


//   prev_pos = curr_pos;
//   prev_time = curr_time;
// Serial.printf("Time: %lu, dt: %.6f, pos: %.2f, vel: %.2f\n", 
//               curr_time, dt, curr_pos, filtered_vel);
//   // Serial.printf("Dt : %f, %f %f\n ", dt,prev_pos, getPosition());
//   return filtered_vel ;
// }

float getVelocity() {
    static unsigned long prev_time = micros();
    static float prev_pos = getPosition();
    static float filtered_vel = 0;
    const float alpha = 0.9f;
    const float MIN_DT = 0.005f;  // 5ms minimum window
    
    unsigned long curr_time = micros();
    float dt = (float)(curr_time - prev_time) / 1000000.0f;
    
    // Only update if minimum time window has elapsed
    if (dt >= MIN_DT) {
        float curr_pos = getPosition();
        float raw_vel = (curr_pos - prev_pos) / dt;
        
        // Stronger filtering for more stable output
        filtered_vel = alpha * filtered_vel + (1.0f - alpha) * raw_vel;
        
        prev_pos = curr_pos;
        prev_time = curr_time;
    }
    
    return filtered_vel;
}

int setSpeed(float input) {
  int reference_point = 1500;
  input = constrain(input, -1.0, 1.0);
  int output = reference_point + (int)(input * 400);

  servo.writeMicroseconds(output);
  return output;
}

int measureStepResponse(float step) {
  unsigned long startTime = millis();
  unsigned long sampleTime = 1000 / MOTOR_CONTROL_FREQ;
  unsigned long nextSampleTime = startTime + sampleTime;
  int sampleCount = 0;
  int pwm = setSpeed(step);
  while (sampleCount < MAX_SAMPLES) {
    if (millis() >= nextSampleTime) {
      stepResponseData[sampleCount].timestamp = micros();
      stepResponseData[sampleCount].step = step;
      stepResponseData[sampleCount].speed = encoder.getCount();
      stepResponseData[sampleCount].pwm = pwm;
      nextSampleTime += sampleTime;
      sampleCount++;
      // Serial.println(sampleCount);
    }
  }
  
  setSpeed(0);
  for (int i = 0; i < sampleCount; i++) {
    Serial.printf("%d,%lu,%.2f,%.2f,%d\n", i,
                  stepResponseData[i].timestamp - startTime,
                  stepResponseData[i].step, stepResponseData[i].speed,
                  stepResponseData[i].pwm);
  }
  return sampleCount;
}

// int measureStepResponse(float step){
//     unsigned long startTime = millis();
//     unsigned long sampleTime = 1000 / MOTOR_CONTROL_FREQ;
//     unsigned long nextSampleTime = startTime + sampleTime;
//     int sampleCount = 0;

//     // Add debug prints
//     Serial.printf("Starting measurement for step: %.2f\n", step);
//     int pwm = setSpeed(step);
//     Serial.printf("PWM set to: %d\n", pwm);

//     while(sampleCount < MAX_SAMPLES){
//         unsigned long currentTime = millis();
//         encoder.update();
//         if(currentTime >= nextSampleTime){
//             // Add timestamp to debug
//             stepResponseData[sampleCount].step = step;
//             stepResponseData[sampleCount].speed = encoder.getVelocity();
//             stepResponseData[sampleCount].pwm = pwm;
//             Serial.printf("Sample %d at %lu ms: %.2f,%.2f,%d\n",
//                          sampleCount, currentTime - startTime,
//                          stepResponseData[sampleCount].step,
//                          stepResponseData[sampleCount].speed,
//                          stepResponseData[sampleCount].pwm);
//             nextSampleTime += sampleTime;
//             sampleCount++;
//             if(sampleCount % 10 == 0) {
//             vTaskDelay(1);
//         }
//         }

//     }
//     return sampleCount;
// }


