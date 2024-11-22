#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// SPI pins configuration as per your setup
#define GPIO_MOSI   23
#define GPIO_MISO   19 
#define GPIO_SCLK   18
#define GPIO_CS     5

#define RCV_HOST    HSPI_HOST

// Buffer size for SPI transactions
#define BUFFER_SIZE 128

static const char* TAG = "spi_slave";

void app_main(void) {
    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    //Initialize SPI slave interface
    esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    //Buffer for receiving data
    uint8_t* recvbuf = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    //Buffer for sending data
    uint8_t* sendbuf = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);

    //Initialize buffers
    memset(recvbuf, 0, BUFFER_SIZE);
    memset(sendbuf, 0, BUFFER_SIZE);

    //Transaction structure
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while (1) {
        //Clear receive buffer
        memset(recvbuf, 0, BUFFER_SIZE);

        //Set up SPI slave transaction
        t.length = BUFFER_SIZE * 8;
        t.rx_buffer = recvbuf;
        t.tx_buffer = sendbuf;

        //Enable SPI slave
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret == ESP_OK) {
            //Print received data
            ESP_LOGI(TAG, "Received %d bytes:", t.trans_len / 8);
            for (int i = 0; i < t.trans_len / 8; i++) {
                printf("%02x ", recvbuf[i]);
            }
            printf("\n");

            //Echo received data back
            memcpy(sendbuf, recvbuf, BUFFER_SIZE);
        }
        else {
            ESP_LOGE(TAG, "SPI slave transmission failed");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay between transactions
    }

    //Clean up (this part never reaches in this example)
    free(recvbuf);
    free(sendbuf);
    spi_slave_free(RCV_HOST);
}