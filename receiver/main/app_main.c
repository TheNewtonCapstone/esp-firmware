#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#define RCV_HOST    HSPI_HOST
#define GPIO_MOSI   23
#define GPIO_MISO   19
#define GPIO_SCLK   18
#define GPIO_CS     5

static const char* TAG = "spi_slave";

// Task to monitor GPIO states
void gpio_monitor_task(void* pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Pin States - CS:%d, SCLK:%d, MOSI:%d",
            gpio_get_level(GPIO_CS),
            gpio_get_level(GPIO_SCLK),
            gpio_get_level(GPIO_MOSI));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    esp_err_t ret;

    // Configure GPIOs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    // Configure MOSI, SCLK, CS as inputs
    io_conf.pin_bit_mask = (1ULL << GPIO_MOSI) | (1ULL << GPIO_SCLK) | (1ULL << GPIO_CS);
    gpio_config(&io_conf);

    // Configure MISO as output
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_MISO);
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIOs configured");

    // Start GPIO monitoring task
    xTaskCreate(gpio_monitor_task, "gpio_monitor", 2048, NULL, 5, NULL);

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
        .flags = SPICOMMON_BUSFLAG_SLAVE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        // Try inverting CS polarity
        // .flags = SPI_SLAVE_POSITIVE_CS  // Add this to try positive CS
    };

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);  // Disabled DMA
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI slave initialization failed: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "SPI slave initialized successfully");

    WORD_ALIGNED_ATTR uint8_t recvbuf[32];
    spi_slave_transaction_t t;

    while (1) {
        memset(recvbuf, 0xA5, sizeof(recvbuf));
        memset(&t, 0, sizeof(t));

        t.length = 32 * 8;
        t.rx_buffer = recvbuf;

        ESP_LOGI(TAG, "Waiting for transaction...");

        // Print CS state right before transaction
        ESP_LOGI(TAG, "CS State before transaction: %d", gpio_get_level(GPIO_CS));

        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Transaction complete! Length trans: %d", t.trans_len / 8);
            printf("Received: ");
            for (int i = 0; i < 16; i++) {
                printf("0x%02X ", recvbuf[i]);
                if ((i + 1) % 8 == 0) printf("\n");
            }
            printf("\n");
        }
        else {
            ESP_LOGE(TAG, "Transaction failed: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}