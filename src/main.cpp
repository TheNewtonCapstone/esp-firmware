#include <stdio.h>

#include "newton/encoder.h"
#include "newton/actuator.h"
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

  Encoder encoder(PIN_A, PIN_B, PPR);

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

  const uint32_t PIN_PWM = 12;
  const uint16_t MIN_PULSE_WIDTH = 1100;
  const uint16_t MAX_PULSE_WIDTH = 1900;

  newton::Actuator actuator(PIN_PWM, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  actuator.set_pulse_width(1500);
  vTaskDelay(pdMS_TO_TICKS(1200));

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