/**************************************************************************
 E32-900T30D LoRa Lib


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Try sending a message to remote LoRa

  Project settings:
  ESP-IDF config editor:
  -> LORA debug settings: y for extended output

  History: master if not shown otherwise
  20250518: V0.1: initial version
  20250720: V0.2: filter with magic bytes

  */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "E32_Lora_Lib.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LORA_LIB";
// Version string for the E32_Lora_Lib
#define E32_LORA_LIB_VERSION "E32_Lora_Lib V0.2 (20250518)"

// Default-Pins

// Pin configuration
/* #define E32_M0_GPIO 10
#define E32_M1_GPIO 11
#define E32_AUX_GPIO 14
#define E32_TXD_GPIO 12 // TXD Pin on ESP32
#define E32_RXD_GPIO 13 // RXD Pin on ESP32 */

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
    //ESP_LOGI(TAG, "Wait for AUX to be HIGH");
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
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "UART write error: %d", bytes_written);
        return ESP_FAIL;
    }
    if ((size_t)bytes_written != len) {
        ESP_LOGW(TAG, "UART write incomplete: %d/%d bytes", bytes_written, (int)len);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "%d Bytes send", len);
    return ESP_OK;
}

#define FILTER_MAGIC_BYTES_ENABLED 0 // Set to 1 to enable magic byte filtering
#define FILTER_LEGACY_REMOVE_LEADING_ENABLED 0 // Set to 1 to enable legacy filter
#if FILTER_MAGIC_BYTES_ENABLED
static const uint8_t MAGIC_BYTES[3] = {0xAA, 0xBB, 0xCC}; // Example magic bytes, change as needed
#endif

// Filter function: returns new length, or 0 if not found
static int filter_message_magic_bytes(uint8_t *buffer, int len) {
#if FILTER_MAGIC_BYTES_ENABLED
    for (int i = 0; i <= len - 3; ++i) {
        if (buffer[i] == MAGIC_BYTES[0] && buffer[i+1] == MAGIC_BYTES[1] && buffer[i+2] == MAGIC_BYTES[2]) {
            // Found magic bytes, shift message to front
            int new_len = len - i - 3;
            if (new_len > 0) {
                for (int j = 0; j < new_len; ++j) {
                    buffer[j] = buffer[i + 3 + j];
                }
                return new_len;
            } else {
                return 0; // Only magic bytes, no message
            }
        }
    }
    return 0; // Magic bytes not found
#elif FILTER_LEGACY_REMOVE_LEADING_ENABLED
    // Legacy filter: remove leading non-ASCII bytes
    int start = 0;
    while (start < len && (buffer[start] < 0x20 || buffer[start] > 0x7E)) {
        start++;
    }
    if (start < len) {
        int new_len = len - start;
        for (int i = 0; i < new_len; ++i) {
            buffer[i] = buffer[start + i];
        }
        return new_len;
    } else {
        return 0;
    }
#else
    // No filtering
    return len;
#endif
}

esp_err_t e32_receive_data(uint8_t *buffer, size_t buffer_len, size_t *received_len)
{
    if (buffer == NULL || received_len == NULL) {
        ESP_LOGE(TAG, "Null pointer argument in e32_receive_data");
        return ESP_ERR_INVALID_ARG;
    }
    wait_for_aux(); // Wait for AUX to be HIGH
    int len = uart_read_bytes(E32_UART_PORT, buffer, buffer_len, pdMS_TO_TICKS(100));
    if (len < 0) {
        ESP_LOGE(TAG, "UART read error: %d", len);
        *received_len = 0;
        return ESP_FAIL;
    }
    if (len == 0) {
        *received_len = 0;
        // ESP_LOGW(TAG, "No data received (timeout)"); // Silenced to avoid log spam during polling
        return ESP_ERR_TIMEOUT;
    }
    // Filter using magic bytes or legacy filter
    int filtered_len = filter_message_magic_bytes(buffer, len);
    if (filtered_len == 0) {
        *received_len = 0;
        return ESP_ERR_TIMEOUT;
    }
    *received_len = (size_t)filtered_len;
    // Optional: null-terminate if there's space
    if (*received_len < buffer_len) {
        buffer[*received_len] = '\0';
    }
    ESP_LOGI(TAG, "Received %d bytes", filtered_len);
    return ESP_OK;
}

bool e32_data_available() {
    size_t length = 0;
    esp_err_t err = uart_get_buffered_data_len(E32_UART_PORT, &length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_get_buffered_data_len error: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGD(TAG, "UART buffered data length: %d", (int)length);
    return length > 0;
}

void e32_init_config(e32_config_t *config)
{
    config->HEAD = 0xC0; // This is the command to save parameters to non-volatile memory.
    config->ADDH = 0x00;
    config->ADDL = 0x00;
    config->SPED.uartParity = E32_UART_PARITY_8N1;
    config->SPED.uartBaudRate = E32_UART_BAUD_RATE_9600;
    config->SPED.airDataRate = AIR_DATA_RATE_2400;
    config->CHAN = 0x06; // Kanal 7 (902.875MHz)
    // Add explanation for 0x06: This corresponds to channel 7 in the frequency range.
    config->OPTION.fixedTransmission = TRANSMISSION_TRANSPARENT; // Transparent mode
    config->OPTION.ioDriveMode = IO_DRIVE_MODE_PUSH_PULL;
    config->OPTION.wirelessWakeupTime = WIRELESS_WAKEUP_TIME_250MS;
    config->OPTION.fec = FEC_ENABLE;
    config->OPTION.transmissionPower = TRANSMISSION_POWER_30dBm; // 30dBm
}


void init_io()
{
    ESP_LOGI(TAG, "Initialisiere IO-Pins");
    // configure command pins M0 and M1
    gpio_config_t mode_conf = {
        .intr_type = GPIO_INTR_DISABLE,                                // no interrupt
        .mode = GPIO_MODE_OUTPUT,                                      // set as output mode
        .pin_bit_mask = (1ULL << e32_pins.gpio_m0) | (1ULL << e32_pins.gpio_m1), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                         // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE                              // disable pull-up mode
    };
    gpio_config(&mode_conf);
    // configure AUX pin
    gpio_config_t aux_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // no interrupt
        .mode = GPIO_MODE_INPUT,                // set as input mode
        .pin_bit_mask = (1ULL << e32_pins.gpio_aux), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE       // disable pull-up mode
    };
    gpio_config(&aux_conf);

    // configure UART with the given settings
    uart_config_t uart_config = {
        .baud_rate = 9600,                     // baud rate
        .data_bits = UART_DATA_8_BITS,         // data bits
        .parity = UART_PARITY_DISABLE,         // no parity
        .stop_bits = UART_STOP_BITS_1,         // stop bits
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // no flow control
    };
    uart_driver_install(E32_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);                                 // install UART driver
    uart_param_config(E32_UART_PORT, &uart_config);                                                  // configure UART parameters
    uart_set_pin(E32_UART_PORT, e32_pins.gpio_txd, e32_pins.gpio_rxd, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // set UART pins

    // Activate internal pull-up on RX pin to avoid floating input
    gpio_set_pull_mode(e32_pins.gpio_rxd, GPIO_PULLUP_ONLY);
/* #if CONFIG_DEBUG_LORA
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) | (1ULL << 14));
    ESP_LOGI(TAG, "GPIO M0: %d, M1: %d, TXD: %d, RXD: %d, AUX: %d", E32_M0_GPIO, E32_M1_GPIO, E32_TXD_GPIO, E32_RXD_GPIO, E32_AUX_GPIO);
#endif */
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

    // Flush UART RX buffer after config to avoid leftover config/status bytes
    uart_flush_input(E32_UART_PORT);
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



const char* e32_lora_lib_get_version(void) {
    return E32_LORA_LIB_VERSION;
}

/*
Recommendation for hardware stability:
- For ESP32 UART RX pin, enable the internal pull-up resistor to avoid floating input when the line is idle.
  Example (replace RX_PIN with your actual RX GPIO number):
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);
- Avoid using pull-down on RX unless your hardware specifically requires it.
- Ensure both devices share a common ground.
- Keep UART lines as short as possible and avoid running them parallel to high-current traces.
*/