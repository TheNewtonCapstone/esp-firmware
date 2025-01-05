#include <stdio.h>

#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const uint32_t PIN_A = 22;
const uint32_t PIN_B = 23;
const int16_t CPR = 512;

extern "C" void app_main() {
  Encoder encoder(PIN_A, PIN_B, CPR);

  while (true) {
    encoder.update();
    printf("Position: %d\n", (int)encoder.get_position());
    printf("Velocity: %f\n", encoder.get_velocity());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}