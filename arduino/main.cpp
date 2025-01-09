#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdio.h>
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CONSTANTS
#define INTERRUPT_PIN 2
#define INTERNAL_LED_PIN 13
#define RX_PIN 16
#define TX_PIN 17
#define TX_BUFFER_SIZE 32
#define RX_BUFFER_SIZE 255

// MOTOR CONSTANTS
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 8;

struct ActuatorState {
    unsigned long jetson_send_time; // timestamp in microseconds
    unsigned long jetson_receive_time;
    unsigned long esp_receive_time;
    unsigned long esp_measure_time;

    float commanded_torque; // normalized used by issac sim
    int pulse_count; // info from encoder
    float velocity;
    float pwm;
    bool direction;
};

// Global variables
ActuatorState motor_data;
volatile int pulse_count = 0;
unsigned long prev_time = 0;
int prev_count = 0;

struct Motor {
    int encoder;
    int pwm;
    int dir;
};

const Motor motor = { 18, 5, 19 };  // Adjust pins as needed

void setup() {
    pinMode(INTERNAL_LED_PIN, OUTPUT);
    digitalWrite(INTERNAL_LED_PIN, HIGH);

    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, RX_PIN, TX_PIN);

    setup_motor();
    xTaskCreatePinnedToCore(task_motor, "task_motor", 4096, NULL, 5, NULL, 0);
}

void loop() {
    // Empty - using RTOS task
}

void setup_motor() {
    pinMode(motor.encoder, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(motor.encoder), ISR_ENCODER_CB, RISING);

    ledcAttach(motor.pwm, PWM_FREQ, PWM_RESOLUTION);
    pinMode(motor.dir, OUTPUT);

    // Quick test of motor
    ledcWrite(motor.pwm, 255);
    delay(100);
    ledcWrite(motor.pwm, 0);
}

void task_motor(void *pvParameters) {
    char rx_buf[RX_BUFFER_SIZE];
    char tx_buf[TX_BUFFER_SIZE];
    float motor_cmd;

    prev_time = micros();

    while (1) {
        // Clear receive buffer
        memset(rx_buf, 0, RX_BUFFER_SIZE);

        // Record when we get command
        unsigned long command_time = micros();
        int rx_bytes = Serial2.readBytes(rx_buf, RX_BUFFER_SIZE);

        if (rx_bytes > 0 && rx_buf[0] == 's' && rx_buf[15] == 'e') {
            memcpy(&motor_cmd, rx_buf + 3, sizeof(float));

            // Record command reception time
            motor_data.command_received_us = command_time;
            motor_data.commanded_torque = motor_cmd;

            // Apply motor command
            set_torque(motor_cmd);
        } else {
            set_torque(0);
        }

        // Take measurements
        unsigned long current_time = micros();
        float dt = (current_time - prev_time) * 1e-6f; // Convert to seconds

        motor_data.measurement_time_us = current_time;
        motor_data.pulse_count = pulse_count;
        motor_data.velocity = (pulse_count - prev_count) / dt;

        // Pack and send data
        tx_buf[0] = 's';
        tx_buf[1] = 'm';
        memcpy(tx_buf + 3, &motor_data, sizeof(ActuatorState));
        tx_buf[31] = 'e';
        Serial2.write(tx_buf, TX_BUFFER_SIZE);

        // Update previous values
        prev_count = pulse_count;
        prev_time = current_time;

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update rate
    }
}

void set_torque(float torque) {
    // Save motor command data
    motor_data.direction = (torque > 0);
    int pwm = (int)(((abs(torque) * 0.625f) + 0.375f) * 255.f);
    motor_data.pwm_value = pwm;

    // Apply to motor
    digitalWrite(motor.dir, motor_data.direction ? HIGH : LOW);
    ledcWrite(motor.pwm, pwm);
}

void ISR_ENCODER_CB() {
    pulse_count++;
}