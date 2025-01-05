#include <stdio.h>

#include "encoder.h"

const uint32_t PIN_A = 22;
const uint32_t PIN_B = 23;
const int16_t CPR = 512;

void app_main() {
  Encoder encoder(PIN_A, PIN_B, CPR);

  while (true) {
    encoder.update();
    printf("Position: %d\n", encoder.get_position());
    printf("Velocity: %f\n", encoder.get_velocity());
  }
}