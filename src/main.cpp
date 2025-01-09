#include <stdio.h>
#include "newton/actuator.h"
#include "newton/encoder.h"
#include "newton/stopwatch.h"
#include "velocity_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_err.h>
#include "esp_task_wdt.h"

const char* TAG = "main";

// Shared variables for inter-task communication
static float g_current_velocity = 0.0f;
static SemaphoreHandle_t velocity_mutex = NULL;
static newton::VelocityController g_pid_controller;

void task_encoder(void*) {
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  const uint32_t PIN_A = 22;
  const uint32_t PIN_B = 23;
  const int16_t PPR = 512;
  newton::Encoder encoder(PIN_A, PIN_B, PPR);

  while (true) {
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    encoder.update();


    if (xSemaphoreTake(velocity_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_current_velocity = encoder.get_velocity();
      xSemaphoreGive(velocity_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // 1kHz sampling
  }
}

void task_pid_control(void*) {
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  const uint32_t PIN_PWM = 14;
  const uint16_t MIN_PULSE_WIDTH = 1100;
  const uint16_t MAX_PULSE_WIDTH = 1900;
  newton::Actuator actuator(PIN_PWM, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  g_pid_controller.set_target(100.0f);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    ESP_ERROR_CHECK(esp_task_wdt_reset());


    float current_velocity = 0.0f;
    if (xSemaphoreTake(velocity_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      current_velocity = g_current_velocity;
      xSemaphoreGive(velocity_mutex);
    }

    // Update PID controller
    float control_output = g_pid_controller.update(current_velocity);

    // Apply control output to actuator
    actuator.set_ranged(control_output);

    // Ensure precise 1kHz timing
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main() {
    // Create mutex for velocity sharing
  velocity_mutex = xSemaphoreCreateMutex();

  // Create tasks with appropriate priorities
  xTaskCreate(task_pid_control, "task_pid", 8192, NULL, 11, NULL);
  xTaskCreate(task_encoder, "task_encoder", 4096, NULL, 10, NULL);
}