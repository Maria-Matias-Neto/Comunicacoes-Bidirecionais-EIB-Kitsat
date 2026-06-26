#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT UART_NUM_1

// UART ligada ao KitSat
#define TX_PIN 19
#define RX_PIN 21

#define BUF_SIZE 256

// ======================================================
// FNV CHECKSUM
// ======================================================

uint32_t ufnv(char *bytes, int str_len)
{
    uint32_t hval = 0x811c9dc5;
    uint32_t fnv_32_prime = 0x01000193;
    uint64_t uint32_max = 4294967296;

    for(int i = 0; i < str_len; i++)
    {
        hval = hval ^ bytes[i];
        hval = (hval * fnv_32_prime) % uint32_max;
    }

    return hval;
}

// ======================================================
// CREATE KITSAT COMMAND
// ======================================================

uint8_t createKitsatCommand(
    char *buf,
    uint8_t subsystem_id,
    uint8_t command_id,
    char *parameters,
    uint8_t parameters_len)
{
    buf[0] = subsystem_id;
    buf[1] = command_id;
    buf[2] = parameters_len;

    if(parameters_len > 0)
    {
        memcpy(buf + 3, parameters, parameters_len);
    }

    uint32_t fnv = ufnv(buf, parameters_len + 3);

    memcpy(buf + 3 + parameters_len, &fnv, 4);

    return parameters_len + 7;
}

// ======================================================
// TELEMETRY STRUCT
// ======================================================

struct telemetry
{
    uint8_t target_id;
    uint8_t command_id;
    uint32_t timestamp;
    uint8_t data_length;
    char *data;
    uint8_t valid_telemetry;
};

// ======================================================
// PARSE TELEMETRY
// ======================================================

struct telemetry parse_telemetry(char *buf, size_t len)
{
    struct telemetry tm;

    tm.valid_telemetry = 0;

    if(len < 11)
    {
        return tm;
    }

    tm.target_id = buf[0];
    tm.command_id = buf[1];

    tm.timestamp =
        ((uint32_t)buf[2] << 24) |
        ((uint32_t)buf[3] << 16) |
        ((uint32_t)buf[4] << 8)  |
        ((uint32_t)buf[5]);

    tm.data_length = buf[6];

    if(len < (7 + tm.data_length + 4))
    {
        return tm;
    }

    tm.data = malloc(tm.data_length + 1);

    if(tm.data == NULL)
    {
        return tm;
    }

    memcpy(tm.data, &buf[7], tm.data_length);

    tm.data[tm.data_length] = '\0';

    uint32_t received_checksum =
        ((uint32_t)buf[tm.data_length + 10] << 24) |
        ((uint32_t)buf[tm.data_length + 9] << 16) |
        ((uint32_t)buf[tm.data_length + 8] << 8) |
        ((uint32_t)buf[tm.data_length + 7]);

    uint32_t calculated_checksum =
        ufnv(buf, tm.data_length + 7);

    if(received_checksum == calculated_checksum)
    {
        tm.valid_telemetry = 1;
    }

    return tm;
}

// ======================================================
// UART INIT
// ======================================================

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_PORT, &cfg);

    uart_set_pin(
        UART_PORT,
        TX_PIN,
        RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    printf("UART pronta.\n");
}

// ======================================================
// MAIN
// ======================================================

void app_main(void)
{
    uart_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    // ==========================================
    // IMU_GET_ACC
    //
    // target_id = 5
    // command_id = 9
    // ==========================================

    char tx_msg[64];

    uint8_t tx_len =
        createKitsatCommand(
            tx_msg,
            5,
            5,
            NULL,
            0
        );

    printf("Enviar comando IMU_GET_ACC...\n");

    uart_write_bytes(UART_PORT, tx_msg, tx_len);

    // ==========================================
    // RECEBER RESPOSTA
    // ==========================================

    char rx_buf[256];

    while(1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            (uint8_t*)rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(1500)
        );

        if(len > 0)
        {
            //printf("\nRecebidos %d bytes\n", len);

            //for(int i = 0; i < len; i++)
            //{
            //    printf("%02X ", (uint8_t)rx_buf[i]);
            //}

            //printf("\n");

            struct telemetry tm =
                parse_telemetry(rx_buf, len);

            if(tm.valid_telemetry)
            {
                //printf("\n=== TELEMETRIA VALIDA ===\n");

                //printf("Target ID: %d\n", tm.target_id);
                //printf("Command ID: %d\n", tm.command_id);
                //printf("Timestamp: %lu\n", tm.timestamp);
                //printf("Data length: %d\n", tm.data_length);

                //printf("Data:\n");

                //for(int i = 0; i < tm.data_length; i++)
                //{
                //    printf("%02X ", (uint8_t)tm.data[i]);
                //}

                //printf("\n");

                //printf("ASCII:\n");
                printf("%.*s\n", tm.data_length, tm.data);

                free(tm.data);
            }
            else
            {
                printf("Checksum invalido ou pacote incompleto.\n");
            }
        }
        else
        {
            printf("Sem resposta...\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        printf("\nEnviar novo pedido IMU_GET_ACC...\n");

        uart_write_bytes(UART_PORT, tx_msg, tx_len);
    }
}
