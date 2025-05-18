#include <stdio.h>
//#include <string.h>
#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "E32_Lora_Lib.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LORA_LIB";

// Default-Pins
static e32_pins_t e32_pins = {
    .gpio_m0 = 10,
    .gpio_m1 = 11,
    .gpio_aux = 14,
    .gpio_txd = 12,
    .gpio_rxd = 13,
    .uart_port = UART_NUM_1
};

void e32_set_pins(const e32_pins_t *pins)
{
    if (pins != NULL) {
        e32_pins = *pins;
    }
}

void func(void)
{
    ESP_LOGI(TAG, "LoRAESPIDFLib V0.1");
    gpio_get_level(e32_pins.gpio_aux);

}

void wait_for_aux()
{
    // AUX is HIGH, when module is ready
    ESP_LOGI(TAG, "Wait for AUX to be HIGH");
    while (!gpio_get_level(e32_pins.gpio_aux))
    {
      vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB));
    }
} 
void set_mode(enum MODE mode)
{
    gpio_set_level(e32_pins.gpio_m0, (mode & 0b01));
    gpio_set_level(e32_pins.gpio_m1, ((mode & 0b10) >> 1));
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB));
    wait_for_aux();
}

// send data to E32 module
esp_err_t e32_send_data(const uint8_t *data, size_t len)
{
    int bytes_written = uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    ESP_LOGI(TAG, "%d Bytes send", len);
    return (bytes_written == len) ? ESP_OK : ESP_FAIL;
}

void sendConfiguration(e32_config_t *e32_config)
{
    ESP_LOGI(TAG, "Send configuration to E32 module");

    set_mode(MODE_SLEEP_PROG);                 // Set to programming mode (M0=1, M1=1)
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed

    ESP_LOGI(TAG, "Send configuration command to E32 module");

    ESP_ERROR_CHECK(e32_send_data((uint8_t *)e32_config, sizeof(e32_config_t)));
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed
    wait_for_aux();                // Wait for AUX to be HIGH
    set_mode(MODE_NORMAL);                // Set back to normal mode (M0=0, M1=0)
    ESP_LOGI(TAG, "Configuration command sent to E32 module");
}