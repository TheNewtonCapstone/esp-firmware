#include "logger.h"
void setup() {
  Logger& logger = Logger::get_instance(115200);
  if (logger.begin() != 1) {
    Serial.println("Logger init failed");
    return;
  }

  LOG_INFO("System starting up...");
  LOG_INFO("Running on core %d", xPortGetCoreID());
  LOG_INFO("Free heap: %lu bytes", ESP.getFreeHeap());
}

void loop() {
  // Example periodic logging
  static uint32_t lastLog = 0;
  const uint32_t LOG_INTERVAL = 1000;  // Log every 1 second

  uint32_t now = millis();
  if (now - lastLog >= LOG_INTERVAL) {
    // Log system status periodically
    LOG_INFO("Uptime: %lu ms, Free heap: %lu bytes",
             now,
             ESP.getFreeHeap());

    lastLog = now;
  }

  // Example logging different data types
  static int counter = 0;
  if (counter % 100 == 0) {
    float temperature = 25.5f;
    int humidity = 60;

    LOG_INFO("Temperature: %.1f°C, Humidity: %d%%",
             temperature,
             humidity);
  }
  counter++;

  // Example error condition logging
  if (ESP.getFreeHeap() < 50000) {
    LOG_INFO("WARNING: Low memory condition detected!");
  }

  // Don't forget to include some delay to prevent watchdog triggers
  vTaskDelay(pdMS_TO_TICKS(10));
}