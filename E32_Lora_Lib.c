#include <stdio.h>
//#include <string.h>
//#include "freertos/FreeRTOS.h"
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
      // vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB));
    }
} 