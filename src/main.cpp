#include <stdio.h>

#include "newton/encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_err.h>
#include "esp_task_wdt.h"

const uint32_t PIN_A = 22;
const uint32_t PIN_B = 23;
const int16_t PPR = 512;

const char *TAG = "main";

void task_encoder(void *)
{
  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  Encoder encoder(PIN_A, PIN_B, PPR);

  while (true)
  {
    ESP_ERROR_CHECK(esp_task_wdt_reset());

    encoder.update();

    float position = encoder.get_position();
    float velocity = encoder.get_velocity();

    printf("%f, %f\n", position, velocity);
  }
}

extern "C" void app_main()
{
  xTaskCreatePinnedToCore(task_encoder, "task_encoder", 2048, NULL, 10, NULL, 0);
}