// #include <SimpleFOC.h>
#include <ESP32Encoder.h>

#include <ESP32Servo.h>

#include "step.h"

#define motorPin 12
#define A 22
#define B 23

const float VEL_PERIOD = 0.0005f;
static constexpr float PPR = 512;
static constexpr float PPR_TO_RAD = PI / PPR * -1.0f; 

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
    encoder.setCount(0);

}

void loop() {
  //     if(Serial.available() > 0) {
  //         str_pos = Serial.readStringUntil('\r\n');
  //         tar_vel = str_pos.toFloat();
  //         Serial.printf("New target velocity: %.2f\n", tar_vel);
  //         vel_integral = 0; // Reset integral when target changes
  //     }

  //     encoder.update();
  //     unsigned long curr_tijkk:w:w
  // me = micros();
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
  delay(2000);
  // step response
  for(float step = 0.1f; step < .9f; step += 0.1f){
    Serial.printf("step :%f\n", step);
    Serial.printf("i,%lu,%lu,%.5f,%.5f,%f,%d");
    int sampleCount = measureStepResponse(step);
    Serial.printf("Step: %.2f, Samples: %d\n\n", step, sampleCount);
    delay(2000);
  }

  // Serial.printf("Position :  %f\n", getPosition());
  vTaskDelay(1);
}


float getPosition() {
  return  encoder.getCount() * PPR_TO_RAD ;
}

float getVelocity() {
    static unsigned long prev_time = micros();
    static float prev_pos = 0;
    const float alpha = 0.9f;
    const float MIN_DT = 0.005f;  // 5ms minimum window
    // static float filtered_vel = 0;
    
    unsigned long current_time = micros();
    float current_pos = getPosition();
    
    float dt = (float)(current_time - prev_time) / 1000000.0f;
    float raw_velocity = (current_pos - prev_pos) / dt;

    prev_pos = current_pos;
    prev_time = current_time;

    return raw_velocity;
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
  encoder.setCount(0);
  int pwm = setSpeed(step);
  while (sampleCount < MAX_SAMPLES) {
    if (millis() >= nextSampleTime) {
      stepResponseData[sampleCount].timestamp = micros();
      stepResponseData[sampleCount].count = encoder.getCount();
      stepResponseData[sampleCount].position = getPosition();
      stepResponseData[sampleCount].velocity = getVelocity();
      stepResponseData[sampleCount].step = step;
      stepResponseData[sampleCount].pwm = pwm;
      nextSampleTime += sampleTime;
      sampleCount++;
      // Serial.println(sampleCount);
    }
  }
  
  setSpeed(0); 
  for (int i = 0; i < sampleCount; i++) {
    Serial.printf("%d,%lu,%lu,%.4f,%.4f,%f,%d\n", 
                  i,
                  stepResponseData[i].timestamp - startTime,
                  stepResponseData[i].count,
                  stepResponseData[i].position,
                  stepResponseData[i].velocity,
                  stepResponseData[i].step,
                  stepResponseData[i].pwm);
  }
  return sampleCount;
}

