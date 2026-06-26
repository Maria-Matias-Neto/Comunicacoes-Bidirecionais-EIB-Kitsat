#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT UART_NUM_1

// Ligações ao KitSat
#define TX_PIN 19
#define RX_PIN 21

#define BUF_SIZE 128

// ================= UART INIT =================
static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);

    uart_param_config(UART_PORT, &cfg);

    uart_set_pin(
        UART_PORT,
        TX_PIN,
        RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
}

// ================= PROCESS COMMAND =================
static void process_command(char *cmd)
{
    printf("Recebido: %s\n", cmd);

    // comandos aceites
    if (
        strcmp(cmd, "PING") == 0 ||
        strcmp(cmd, "ping") == 0 ||
        strcmp(cmd, "str ping") == 0
    )
    {
        printf("Enviar ACK\n");

        uart_write_bytes(UART_PORT, "ACK", 3);
    }
}

// ================= UART TASK =================
static void uart_task(void *arg)
{
    uint8_t data[BUF_SIZE];
    char buffer[BUF_SIZE];

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            data,
            BUF_SIZE - 1,
            100 / portTICK_PERIOD_MS
        );

        if (len > 0)
        {
            data[len] = '\0';

            printf("Recebidos %d bytes: ", len);

            for (int i = 0; i < len; i++)
            {
                printf("%02X ", data[i]);
            }

            printf("\n");

            strcpy(buffer, (char *)data);

            process_command(buffer);
        }
    }
}

// ================= MAIN =================
void app_main(void)
{
    printf("Inicializar UART...\n");

    uart_init();

    printf("UART pronta.\n");

    xTaskCreate(
        uart_task,
        "uart_task",
        4096,
        NULL,
        5,
        NULL
    );

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
