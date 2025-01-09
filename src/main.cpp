#include <stdio.h>

#include "newton/actuator.h"
#include "newton/encoder.h"
#include "newton/stopwatch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_err.h>
#include "esp_task_wdt.h"

const char *TAG = "main";

void task_encoder(void *)
{
  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  const uint32_t PIN_A = 22;
  const uint32_t PIN_B = 23;
  const int16_t PPR = 512;

  newton::Encoder encoder(PIN_A, PIN_B, PPR);

  while (true)
  {
    ESP_ERROR_CHECK(esp_task_wdt_reset());

    encoder.update();

    float position = encoder.get_position();
    float velocity = encoder.get_velocity();

    // printf("%f, %f\n", position, velocity);
  }
}

void task_actuator(void *)
{
  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  const uint32_t PIN_PWM = 14;
  const uint16_t MIN_PULSE_WIDTH = 1100;
  const uint16_t MAX_PULSE_WIDTH = 1900;

  newton::Actuator actuator(PIN_PWM, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  newton::Stopwatch stopwatch("Actuator");

  stopwatch.start();
  actuator.set_pulse_width(1500);

  stopwatch.lap();
  vTaskDelay(pdMS_TO_TICKS(1200));
  stopwatch.stop();

  ESP_LOGI(TAG, "Start time: %lld", stopwatch.get_start_time().get_value());
  ESP_LOGI(TAG, "Pulse width time: %lld", stopwatch.get_lap_duration(0).get_value());
  ESP_LOGI(TAG, "Task delay time: %lld", stopwatch.get_lap_duration(1).get_value());
  ESP_LOGI(TAG, "Elapsed time: %lld", stopwatch.get_elapsed_time().get_value());
  ESP_LOGI(TAG, "End time: %lld", stopwatch.get_end_time().get_value());

  ESP_LOGI(TAG, "Starting actuator loop");

  while (true)
  {
    ESP_ERROR_CHECK(esp_task_wdt_reset());

    actuator.set_ranged(0.5f);
    vTaskDelay(pdMS_TO_TICKS(500));

    actuator.set_ranged(-0.5f);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

extern "C" void app_main()
{
  xTaskCreate(task_actuator, "task_actuator", 8192, NULL, 10, NULL);
  xTaskCreate(task_encoder, "task_encoder", 4096, NULL, 10, NULL);
}