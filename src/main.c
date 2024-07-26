#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_MISO_HSPI 12
#define PIN_NUM_MOSI_HSPI 13
#define PIN_NUM_CLK_HSPI  14
#define PIN_NUM_CS_HSPI   15

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

static const char *TAG = "SPI_EXAMPLE";

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi_vspi;
    spi_device_handle_t spi_hspi;

    init_uart();  // UART初期化

    // VSPI設定
    spi_bus_config_t buscfg_vspi = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    // HSPI設定
    spi_bus_config_t buscfg_hspi = {
        .miso_io_num = PIN_NUM_MISO_HSPI,
        .mosi_io_num = PIN_NUM_MOSI_HSPI,
        .sclk_io_num = PIN_NUM_CLK_HSPI,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    // デバイス設定
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = -1,         // CS pin managed by software
        .queue_size = 7,
    };

    // VSPIバスを初期化
    ret = spi_bus_initialize(SPI2_HOST, &buscfg_vspi, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // HSPIバスを初期化
    ret = spi_bus_initialize(SPI3_HOST, &buscfg_hspi, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // VSPIデバイスを追加
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_vspi);
    ESP_ERROR_CHECK(ret);

    // HSPIデバイスを追加
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_hspi);
    ESP_ERROR_CHECK(ret);

    printf("SPI initialization complete\n");

    uint8_t counter = 0;
    spi_transaction_t t;

    while(1) {
        memset(&t, 0, sizeof(t));
        t.length = 8;  // 8ビット
        t.tx_buffer = &counter;
        t.rx_buffer = NULL;

        // VSPIでデータ送信
        ret = spi_device_transmit(spi_vspi, &t);
        if (ret == ESP_OK) {
            printf("VSPI sent: 0x%02X\n", counter);
        } else {
            printf("VSPI transmission failed\n");
        }

        // HSPIでデータ送信
        ret = spi_device_transmit(spi_hspi, &t);
        if (ret == ESP_OK) {
            printf("HSPI sent: 0x%02X\n", counter);
        } else {
            printf("HSPI transmission failed\n");
        }

        counter++;  // カウンターをインクリメント

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1秒待機
    }
}