#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#define TAG "SLAVE"

/**
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD 33 // (33) // 32
#define ECHO_TEST_RXD 39 // (39)  // 36
#define ECHO_TEST_RTS (-1)
#define ECHO_TEST_CTS (-1)

#define BUF_SIZE (1024)
#define READ_FROM_TERMINAL

static void slave_read_write_task(void * arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    gpio_set_direction(ECHO_TEST_TXD, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_TEST_RXD, GPIO_MODE_INPUT);

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS,
                 ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t * data_from_slave = (uint8_t *) malloc(BUF_SIZE);
    uint8_t * data            = (uint8_t *) malloc(BUF_SIZE);
    char      null_string     = '\0';
    strncpy((char *) &data[1], &null_string, 1);

    char string[BUF_SIZE] = "Hello world from slave!";
    memcpy(data_from_slave, string, BUF_SIZE);
    size_t available_bytes = 0;
#ifdef READ_FROM_TERMINAL
    while (1)
    {
        // Write data back to the UART
        uart_write_bytes(UART_NUM_2, (const char *) data_from_slave,
                         strlen(string));

        uart_get_buffered_data_len(UART_NUM_2, &available_bytes);
        // uart_flush(UART_NUM_2);

        // Read data from the UART
        if (available_bytes > 1)
        {
            ESP_LOGI(TAG, "Available number of bytes: %d", available_bytes);
            uart_read_bytes(UART_NUM_2, data, BUF_SIZE,
                            200 / portTICK_PERIOD_MS);
            data[available_bytes] = '\0';
        }
        if (data[0] != '\0')
        {
            ESP_LOGI(TAG, "Received from MASTER: %s", data);
        }

        strncpy((char *) &data[0], &null_string, 1);
        vTaskDelay(600 / portTICK_PERIOD_MS);
    }
#else
    while (1)
    {
        // Write data back to the UART
        uart_write_bytes(UART_NUM_2, (const char *) data_from_slave,
                         strlen((char *) data_from_slave));

        uart_get_buffered_data_len(UART_NUM_2, &available_bytes);
        ESP_LOGI(TAG, "Available number of bytes: %d", available_bytes);

        // Read data from the UART
        memset(data, 0, BUF_SIZE);
        uart_read_bytes(UART_NUM_2, data, available_bytes,
                        200 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Received from MASTER: %s", data);

        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
#endif // READ_FROM_TERMINAL
}

void app_main(void)
{
    xTaskCreate(slave_read_write_task, "UART_SLAVE_TASK", 5 * 1024, NULL, 10,
                NULL);
}
