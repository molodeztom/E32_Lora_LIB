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
   20250802: V0.3: execute init_io internally
   20250802: V0.4: add defines for magic bytes and legacy filter
   20250802: V0.5: add receive message with terminator function
   20250803: V0.6: add automatic version number handling
   20250803: V0.7: optimize debug output, remove unnecessary debug output
   20250804: V0.8: Test receive a struct with checksum, not using terminator because we have a fixed size message 

   */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "E32_Lora_Lib.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LORA_LIB";

// E32 pin definitions
#define E32_PIN_M0 10               // M0 pin for mode control
#define E32_PIN_M1 11               // M1 pin for mode control
#define E32_PIN_AUX 14              // AUX pin for status indication
#define E32_PIN_TXD 12              // TXD pin (ESP32 TX to E32 RX)
#define E32_PIN_RXD 13              // RXD pin (ESP32 RX from E32 TX)


// UART configuration
#define E32_UART_BAUD_RATE 9600     // Default UART baud rate

// E32 configuration constants
#define E32_CONFIG_SAVE_CMD 0xC0       // Command to save parameters to non-volatile memory
#define E32_CONFIG_READ_CMD 0xC1       // Command to read configuration
#define E32_DEFAULT_ADDR_H 0x00        // Default high address byte
#define E32_DEFAULT_ADDR_L 0x00        // Default low address byte
#define E32_DEFAULT_CHANNEL 0x06       // Default channel (902.875MHz)
#define E32_UART_TIMEOUT_MS 100        // UART read timeout in milliseconds
#define E32_CONFIG_READ_TIMEOUT_MS 200 // Timeout for reading configuration

// E32 bit masks for configuration parsing
#define E32_CHANNEL_MASK 0x1F          // Mask to extract channel number (bits 0-4)
#define E32_UART_BAUD_MASK 0x38        // Mask to extract UART baud rate (bits 3-5)
#define E32_UART_BAUD_SHIFT 3          // Shift for UART baud rate bits
#define E32_UART_PARITY_MASK 0xC0      // Mask to extract UART parity (bits 6-7)
#define E32_UART_PARITY_SHIFT 6        // Shift for UART parity bits
#define E32_AIR_RATE_MASK 0x07         // Mask to extract air data rate (bits 0-2)
#define E32_TRANS_MODE_MASK 0x80       // Mask to extract transmission mode (bit 7)
#define E32_TRANS_MODE_SHIFT 7         // Shift for transmission mode bit
#define E32_IO_MODE_MASK 0x40          // Mask to extract I/O mode (bit 6)
#define E32_IO_MODE_SHIFT 6            // Shift for I/O mode bit
#define E32_WAKEUP_TIME_MASK 0x38      // Mask to extract wakeup time (bits 3-5)
#define E32_WAKEUP_TIME_SHIFT 3        // Shift for wakeup time bits
#define E32_FEC_MASK 0x04              // Mask to extract FEC enabled (bit 2)
#define E32_FEC_SHIFT 2                // Shift for FEC bit
#define E32_TX_POWER_MASK 0x03         // Mask to extract TX power (bits 0-1)

// E32 frequency calculation
#define E32_BASE_FREQUENCY 862.0       // Base frequency in MHz
#define E32_WAKEUP_TIME_MULTIPLIER 250 // Multiplier for wakeup time in ms


static e32_pins_t e32_pins = {
    .gpio_m0 = E32_PIN_M0,
    .gpio_m1 = E32_PIN_M1,
    .gpio_aux = E32_PIN_AUX,
    .gpio_txd = E32_PIN_TXD,
    .gpio_rxd = E32_PIN_RXD,
    .uart_port = E32_UART_PORT
};

// Forward declaration of internal functions
static void init_io(void);

void e32_set_pins(const e32_pins_t *pins)
{
    if (pins != NULL) {
        e32_pins = *pins;
    }
}

void initLibrary()
{
#ifdef E32_APP_VERSION_NUMBER
    ESP_LOGI(TAG, "LoRAESPIDFLib %s", E32_APP_VERSION_NUMBER);
#else
    ESP_LOGI(TAG, "LoRAESPIDFLib V0.7");
#endif
    init_io();
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
    ESP_LOGD(TAG, "%d Bytes send", len);
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
    int len = uart_read_bytes(E32_UART_PORT, buffer, buffer_len, pdMS_TO_TICKS(E32_UART_TIMEOUT_MS));
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
    ESP_LOGD(TAG, "Received %d bytes", filtered_len);
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
    config->HEAD = E32_CONFIG_SAVE_CMD; // Command to save parameters to non-volatile memory
    config->ADDH = E32_DEFAULT_ADDR_H;
    config->ADDL = E32_DEFAULT_ADDR_L;
    config->SPED.uartParity = E32_UART_PARITY_8N1;
    config->SPED.uartBaudRate = E32_UART_BAUD_RATE_9600;
    config->SPED.airDataRate = AIR_DATA_RATE_2400;
    config->CHAN = E32_DEFAULT_CHANNEL; // Channel 7 (902.875MHz)
    // Add explanation for 0x06: This corresponds to channel 7 in the frequency range.
    config->OPTION.fixedTransmission = TRANSMISSION_TRANSPARENT; // Transparent mode
    config->OPTION.ioDriveMode = IO_DRIVE_MODE_PUSH_PULL;
    config->OPTION.wirelessWakeupTime = WIRELESS_WAKEUP_TIME_250MS;
    config->OPTION.fec = FEC_ENABLE;
    config->OPTION.transmissionPower = TRANSMISSION_POWER_30dBm; // 30dBm
}

// Private function to initialize IO pins
static void init_io(void)
{
    ESP_LOGD(TAG, "Initialize IO-Pins");
    // configure command pins M0 and M1
    gpio_config_t mode_conf = {
        .intr_type = GPIO_INTR_DISABLE,                                // no interrupt
        .mode = GPIO_MODE_OUTPUT,                                      // set as output mode
        .pin_bit_mask = (1ULL << e32_pins.gpio_m0) | (1ULL << e32_pins.gpio_m1), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                         // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE                              // disable pull-up mode
    };
    gpio_config(&mode_conf);
    ESP_LOGD(TAG, "Initialize AUX Pin");
    // configure AUX pin
    gpio_config_t aux_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // no interrupt
        .mode = GPIO_MODE_INPUT,                // set as input mode
        .pin_bit_mask = (1ULL << e32_pins.gpio_aux), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE       // disable pull-up mode
    };
    gpio_config(&aux_conf);
  ESP_LOGD(TAG, "Initialize Uart Pin");
    // configure UART with the given settings
    uart_config_t uart_config = {
        .baud_rate = E32_UART_BAUD_RATE,       // baud rate
        .data_bits = UART_DATA_8_BITS,         // data bits
        .parity = UART_PARITY_DISABLE,         // no parity
        .stop_bits = UART_STOP_BITS_1,         // stop bits
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // no flow control
    };
     ESP_LOGD(TAG, "Initialize UART");
    uart_driver_install(E32_UART_PORT, E32_UART_BUF_SIZE * 2, 0, 0, NULL, 0);                        // install UART driver
    uart_param_config(E32_UART_PORT, &uart_config);                                                  // configure UART parameters
    uart_set_pin(E32_UART_PORT, e32_pins.gpio_txd, e32_pins.gpio_rxd, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // set UART pins

    // Activate internal pull-up on RX pin to avoid floating input
    gpio_set_pull_mode(e32_pins.gpio_rxd, GPIO_PULLUP_ONLY);

}

void sendConfiguration(e32_config_t *e32_config)
{
    ESP_LOGD(TAG, "Send configuration to E32 module");

    set_mode(MODE_SLEEP_PROG);                 // Set to programming mode (M0=1, M1=1)
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed

    ESP_LOGD(TAG, "Send configuration command to E32 module");

    ESP_ERROR_CHECK(e32_send_data((uint8_t *)e32_config, sizeof(e32_config_t)));
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed
    wait_for_aux();                // Wait for AUX to be HIGH
    set_mode(MODE_NORMAL);                // Set back to normal mode (M0=0, M1=0)
    ESP_LOGD(TAG, "Configuration command sent to E32 module");

    // Flush UART RX buffer after config to avoid leftover config/status bytes
    uart_flush_input(E32_UART_PORT);
}

void get_config()
{
    // Read configuration from E32 module and print it in hex format
    uint8_t e32_read_cmd[] = {E32_CONFIG_READ_CMD, E32_CONFIG_READ_CMD, E32_CONFIG_READ_CMD}; // Command to read configuration
    ESP_LOGD(TAG, "Set programming mode");
    set_mode(MODE_SLEEP_PROG);
    ESP_LOGD(TAG, "Send configuration read command");
    ESP_ERROR_CHECK(e32_send_data(e32_read_cmd, sizeof(e32_read_cmd)));
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // Wait for command to be processed
    wait_for_aux();
    uint8_t e32_rx_buffer[E32_UART_BUF_SIZE];
    int e32_rx_len = uart_read_bytes(E32_UART_PORT, e32_rx_buffer, E32_UART_BUF_SIZE, pdMS_TO_TICKS(E32_CONFIG_READ_TIMEOUT_MS));
    if (e32_rx_len > 0)
    {
        ESP_LOGD(TAG, "Configuration received (%d bytes):", e32_rx_len);
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
        ESP_LOGE(TAG, "Invalid configuration data length: %d bytes", e32_data_len);
        return;
    }
    
    // Print hex representation of configuration data
    char hex_str[e32_data_len * 3 + 1]; // Each byte becomes 2 hex chars + 1 space + null terminator
    for (int i = 0; i < e32_data_len; i++) {
        sprintf(hex_str + i * 3, "%02X ", e32_data[i]);
    }
    ESP_LOGD(TAG, "Config data (hex): %s", hex_str);
    // Extract fields based on E32 response format
    uint8_t e32_header = e32_data[0];
    uint16_t e32_address = (e32_data[1] << 8) | e32_data[2];
    uint8_t e32_sped = e32_data[3];
    uint8_t e32_channel = e32_data[4];
    uint8_t e32_option = e32_data[5];
    e32_channel = e32_channel & E32_CHANNEL_MASK; // Bit 0-4 are channel number, bit 5-7 is reserved
    // Parse SPED byte
    const char *e32_uart_parity_bit[] = {
        "8N1", "8O1", "8E1", "8N1(11)"};
    const char *e32_uart_baudrates[] = {
        "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"};
    const char *e32_air_rates[] = {
        "0.3 kbps", "1.2 kbps", "2.4 kbps", "4.8 kbps", "9.6 kbps", "19.2 kbps", "Invalid", "Invalid"};
    uint8_t e32_uart_baud = (e32_sped & E32_UART_BAUD_MASK) >> E32_UART_BAUD_SHIFT; // Extract UART baud rate bits
    uint8_t e32_air_rate = (e32_sped & E32_AIR_RATE_MASK);                          // Extract air data rate bits
    uint8_t e32_uart_parity = (e32_sped & E32_UART_PARITY_MASK) >> E32_UART_PARITY_SHIFT; // Extract UART parity bits
    // Parse OPTION byte
    const char *e32_transmission_mode_str[] = {
        "Transparent", "Fixed", "Reserved", "Reserved"};
    const char *e32_io_mode_str[] = {
        "TXD, AUX OpenColOut, RXD OpenColIn", "TXD, AUX PushPullOut, RXD PullUpIn"};
    const char *e32_tx_power_str[] = {
        "30 dBm", "27 dBm", "24 dBm", "21 dBm"};
    uint8_t e32_transmission_mode = (e32_option & E32_TRANS_MODE_MASK) >> E32_TRANS_MODE_SHIFT; // Extract transmission mode bit
    uint8_t e32_io_mode = (e32_option & E32_IO_MODE_MASK) >> E32_IO_MODE_SHIFT;                 // Extract I/O mode bit
    uint8_t e32_wakeup_time = (e32_option & E32_WAKEUP_TIME_MASK) >> E32_WAKEUP_TIME_SHIFT;     // Extract wakeup time bits
    uint8_t e32_fec_enabled = (e32_option & E32_FEC_MASK) >> E32_FEC_SHIFT;                     // Extract FEC enabled bit
    uint8_t e32_tx_power = (e32_option & E32_TX_POWER_MASK);                                    // Extract TX power bits
    ESP_LOGI(TAG, "E32 Module Configuration:");
    ESP_LOGI(TAG, "Header: 0x%02X", e32_header);
    ESP_LOGI(TAG, "Address: 0x%04X", e32_address);
    ESP_LOGI(TAG, "UART Parity: %s", e32_uart_parity < 4 ? e32_uart_parity_bit[e32_uart_parity] : "Unknown");
    ESP_LOGI(TAG, "UART Baud Rate: %s bps", e32_uart_baud < 8 ? e32_uart_baudrates[e32_uart_baud] : "Unknown");
    ESP_LOGI(TAG, "Air Data Rate: %s", e32_air_rate < 6 ? e32_air_rates[e32_air_rate] : "Unknown");
    ESP_LOGI(TAG, "Channel: %d (%.1f MHz)", e32_channel, E32_BASE_FREQUENCY + e32_channel);
    ESP_LOGI(TAG, "Transmission Mode: %s", e32_transmission_mode_str[e32_transmission_mode]);
    ESP_LOGI(TAG, "I/O Mode: %s", e32_io_mode_str[e32_io_mode]);
    ESP_LOGI(TAG, "Wakeup Time: %d ms", (e32_wakeup_time + 1) * E32_WAKEUP_TIME_MULTIPLIER); // Wakeup time in ms
    ESP_LOGI(TAG, "FEC Enabled: %s", e32_fec_enabled ? "Yes" : "No");
    ESP_LOGI(TAG, "TX Power: %s", e32_tx_power_str[e32_tx_power]);
}

const char* e32_lora_lib_get_version(void) {
#ifdef E32_APP_VERSION
    return E32_APP_VERSION;
#else
    return "unknown";
#endif
}

esp_err_t e32_receive_message(
    uint8_t *buffer,
    size_t buffer_size,
    size_t *message_len,
    uint32_t timeout_ms,
    uint32_t poll_interval_ms,
    e32_delay_callback_t delay_callback)
{
    if (buffer == NULL || message_len == NULL || delay_callback == NULL) {
        ESP_LOGE(TAG, "Null pointer argument in e32_receive_message");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_size == 0) {
        ESP_LOGE(TAG, "Buffer size cannot be zero");
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t total_received = 0;
    uint32_t waited_ms = 0;
    esp_err_t err = ESP_OK;
    
    // Initial delay before starting to receive
    delay_callback(poll_interval_ms);
    
    ESP_LOGD(TAG, "Waiting for reply up to %" PRIu32 " ms...", timeout_ms);
    
    while (waited_ms < timeout_ms && total_received < buffer_size) {
        size_t received = 0;
        err = e32_receive_data(buffer + total_received, buffer_size - total_received, &received);
        
        if (err == ESP_OK && received > 0) {
            total_received += received;
        }
        
        if (total_received < buffer_size) {
            delay_callback(poll_interval_ms);
            waited_ms += poll_interval_ms;
        }
    }
    
    *message_len = total_received;
    
    if (total_received > 0) {
        ESP_LOGD(TAG, "Received message (%" PRIu32 " bytes)", (uint32_t)total_received);
        return ESP_OK;
    } else {
        ESP_LOGD(TAG, "No reply received within timeout");
        return ESP_ERR_TIMEOUT;
    }
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