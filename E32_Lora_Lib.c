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

esp_err_t e32_receive_data(uint8_t *buffer, size_t buffer_len, size_t *received_len)
{
    if (buffer == NULL || received_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    wait_for_aux(); // Wait for AUX to be HIGH

    int len = uart_read_bytes(E32_UART_PORT, buffer, buffer_len, pdMS_TO_TICKS(100));

    if (len > 0) {
        *received_len = (size_t)len;

        // Optional: null-terminate if there's space
        if (*received_len < buffer_len) {
            buffer[*received_len] = '\0';
        }

        return ESP_OK;
    } else {
        *received_len = 0;
        return ESP_ERR_TIMEOUT;  // no data received within timeout
    }
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

void get_config()
{
    // Read configuration from E32 module and print it in hex format
    uint8_t e32_read_cmd[] = {0xC1, 0xC1, 0xC1}; // Command to read configuration (0xC1: Read module's configuration)
    ESP_LOGI(TAG, "Set programming mode");
    set_mode(MODE_SLEEP_PROG);
    ESP_LOGI(TAG, "Send configuration read command");
    ESP_ERROR_CHECK(e32_send_data(e32_read_cmd, sizeof(e32_read_cmd)));
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed
    wait_for_aux();
    uint8_t e32_rx_buffer[BUF_SIZE];
    int e32_rx_len = uart_read_bytes(E32_UART_PORT, e32_rx_buffer, BUF_SIZE, pdMS_TO_TICKS(200));
    if (e32_rx_len > 0)
    {
        ESP_LOGI(TAG, "Configuration received (%d bytes):", e32_rx_len);
        decode_config(e32_rx_buffer, e32_rx_len); // Decode and print config in plain text
    }
    else
    {
        ESP_LOGW(TAG, "No response from E32 module");
    }
    set_mode(MODE_NORMAL); // Set back to normal mode (M0=0, M1=0)
}

void decode_config(uint8_t *e32_data, int e32_data_len)
{
    if (e32_data_len < 6)
    {
        printf("Invalid configuration data length: %d bytes\n", e32_data_len);
        return;
    }
    for (int i = 0; i < e32_data_len; i++)
        printf("%02X ", e32_data[i]);
    printf("\n");
    // Extract fields based on E32 response format
    uint8_t e32_header = e32_data[0];
    uint16_t e32_address = (e32_data[1] << 8) | e32_data[2];
    uint8_t e32_sped = e32_data[3];
    uint8_t e32_channel = e32_data[4];
    uint8_t e32_option = e32_data[5];
    e32_channel = e32_channel & 0x1F; // Bit 0-4 are channel number, bit 5-7 is reserved (0x1F: Mask to extract lower 5 bits)
    // Parse SPED byte
    const char *e32_uart_parity_bit[] = {
        "8N1", "8O1", "8E1", "8N1(11)"};
    const char *e32_uart_baudrates[] = {
        "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"};
    const char *e32_air_rates[] = {
        "0.3 kbps", "1.2 kbps", "2.4 kbps", "4.8 kbps", "9.6 kbps", "19.2 kbps", "Invalid", "Invalid"};
    uint8_t e32_uart_baud = (e32_sped & 0x38) >> 3; // Mask to get the UART baud rate bits (0x38: Extract bits 3-5)
    uint8_t e32_air_rate = (e32_sped & 0x7);
    uint8_t e32_uart_parity = (e32_sped & 0xC0) >> 6; // Mask to get the UART parity bits (0xC0: Extract bits 6-7)
    // Parse OPTION byte
    const char *e32_transmission_mode_str[] = {
        "Transparent", "Fixed", "Reserved", "Reserved"};
    const char *e32_io_mode_str[] = {
        "TXD, AUX OpenColOut, RXD OpenColIn", "TXD, AUX PushPullOut, RXD PullUpIn"};
    const char *e32_tx_power_str[] = {
        "30 dBm", "27 dBm", "24 dBm", "21 dBm"};
    uint8_t e32_transmission_mode = (e32_option & 0x80) >> 7; // Mask to get the transmission mode bits (0x80: Extract highest bit)
    uint8_t e32_io_mode = (e32_option & 0x40) >> 6;           // Mask to get the I/O mode bits (0x40: Extract bit 6)
    uint8_t e32_wakeup_time = (e32_option & 0x38) >> 3;       // Mask to get the wakeup time bits (0x38: Extract bits 3-5)
    uint8_t e32_fec_enabled = (e32_option & 0x4) >> 2;        // Mask to get the FEC enabled bits (0x4: Extract bit 2)
    uint8_t e32_tx_power = (e32_option & 0x3);                // Mask to get the TX power bits (0x3: Extract bits 0-1)
    printf("E32 Module Configuration:\n");
    printf("Header: 0x%02X\n", e32_header);
    printf("Address: 0x%04X\n", e32_address);
    printf("UART Parity: %s \n", e32_uart_parity < 4 ? e32_uart_parity_bit[e32_uart_parity] : "Unknown");
    printf("UART Baud Rate: %s bps\n", e32_uart_baud < 8 ? e32_uart_baudrates[e32_uart_baud] : "Unknown");
    printf("Air Data Rate: %s\n", e32_air_rate < 6 ? e32_air_rates[e32_air_rate] : "Unknown");
    printf("Channel: %d (%.1f MHz)\n", e32_channel, 862.0 + e32_channel);
    printf("Transmission Mode: %s\n", e32_transmission_mode_str[e32_transmission_mode]);
    printf("I/O Mode: %s\n", e32_io_mode_str[e32_io_mode]);
    printf("Wakeup Time: %d ms\n", (e32_wakeup_time + 1) * 250); // Wakeup time in ms
    printf("FEC Enabled: %s\n", e32_fec_enabled ? "Yes" : "No");
    printf("TX Power: %s\n", e32_tx_power_str[e32_tx_power]);
}