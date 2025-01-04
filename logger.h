#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <Arduino.h>
#include "freertos/queue.h"
#include "freertos/task.h"


#define LOG_INFO(format, ...) \
    do { \
        Logger::get_instance().log(format); \
    } while (0)



struct LogMessage {
  uint32_t timestamp;
  char message[256];
};

class Logger {
public:
  static Logger& get_instance(uint32_t baudrate = 115200) {
    static Logger instance(baudrate);
    return instance;
  }
  ~Logger() {
    if (log_queue != NULL) {
      vQueueDelete(log_queue);
    }
    if (task_handle != NULL) {
      // the handle is deleted by the task itself
      running = false;
      // wake up
      xTaskNotifyGive(task_handle);
      for (int i = 0; i < 100; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (eTaskGetState(task_handle) == eDeleted) {
          break;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
    }
  }

  /* 
    * 
    A static queue and task are used for logging to ensure deterministic behavior, 
    with all memory allocated at compile time. 
    This eliminates heap fragmentation and guarantees memory availability, 
    making it suitable for real-time systems. 
    Trade-offs include fixed memory usage, slightly more complex code, and compile-time stack size determination. 
    Given the logger’s role as a critical system service, these trade-offs are acceptable.
    */
  int begin() {
    if (running) {
      return -1;
    }
    mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    if (mutex == NULL) {
            return -1;  // Mutex creation failed
    }
    static uint8_t storage_area[LOG_QUEUE_SIZE * sizeof(LogMessage)];
    static StaticQueue_t xStaticQueue;


    log_queue = xQueueCreateStatic(LOG_QUEUE_SIZE,
                                   sizeof(LogMessage),
                                   storage_area,
                                   &xStaticQueue);
    if (log_queue == NULL) {
      return -1;
    }

    Serial.begin(baudrate);
    running = true;
    task_handle = xTaskCreateStaticPinnedToCore(
      logger_task,
      "logger",
      LOG_TASK_STACK_SIZE,
      this,
      LOG_TASK_PRIORITY,
      logger_task_stack,
      &logger_task_buffer,
      LOG_TASK_CORE);
      
    return (task_handle != NULL ) ? -1 : 0; 
  }
  void log(const char* format, ...) {
    if(xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE){
        return; // mutex is not available
    }
    // explain the guard pattern
    struct MutexGuard {
      MutexGuard(SemaphoreHandle_t _mutex) : mutex(_mutex) {}
      ~MutexGuard() {
        xSemaphoreGive(mutex);
      }
      SemaphoreHandle_t mutex;
    } guard(mutex);

    if (!running) {
        return;
    }
    // important to check for NULL because vsnprintf relies on
    if (format == NULL) {
      return;
    }
    // xqueueSendToBack crashes if the queue is not created
    if (log_queue == NULL) {
      return;
    }

    LogMessage msg;

    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // size checks to avoid buffer overflow
    size_t len = strlen(buffer);
    if (len >= sizeof(msg.message)) {
      Serial.println("Log message too long");
      return;
    }
    //
    msg.timestamp = millis();
    strncpy(msg.message, buffer, sizeof(msg.message) - 1);
    msg.message[sizeof(msg.message) - 1] = '\0';
    xQueueSendToBack(log_queue, &msg, pdMS_TO_TICKS(10));
  }

    private : Logger(uint32_t _baudrate)
    : baudrate(_baudrate), running(false), task_handle(NULL) {}

  static void logger_task(void* pvParameters) {
    Logger* logger = static_cast<Logger*>(pvParameters);
    LogMessage msg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(true){

        if(xSemaphoreTake(logger->mutex, pdMS_TO_TICKS(10)) == pdTRUE){
            if(!logger->running){
                xSemaphoreGive(logger->mutex);
                break;
            }
            xSemaphoreGive(logger->mutex);
        }

      // HACK PORTMAX_DELAY should be specific
      if (xQueueReceive(logger->log_queue, &msg, portMAX_DELAY) == pdTRUE) {
        if (Serial) {
          Serial.printf("%lu,%s\n", msg.timestamp, msg.message);
          Serial.flush();
        }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);

  };

private:
  uint32_t baudrate;
  bool running{ false };

  QueueHandle_t log_queue;
  TaskHandle_t task_handle;

  SemaphoreHandle_t mutex;
  StaticSemaphore_t mutex_buffer;

  static constexpr uint8_t LOG_QUEUE_SIZE = 100;
  static constexpr uint8_t LOG_TASK_PRIORITY = 1;
  static constexpr uint8_t LOG_TASK_CORE = 0;
  static constexpr unsigned long LOG_TASK_STACK_SIZE = 4096;


  // static allocation bufferss
  static StaticTask_t logger_task_buffer;
  static StackType_t logger_task_stack[LOG_TASK_STACK_SIZE];
};

StaticTask_t Logger::logger_task_buffer;
StackType_t Logger::logger_task_stack[LOG_TASK_STACK_SIZE];

// add utility functions

#endif  // _LOGGER_H_