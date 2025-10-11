/**************************************************************************
 E32-900T30D LoRa Library


  Hardware:
  
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Project settings:
  ESP-IDF config editor:
  -> LORA debug settings: y for extended output

  History: master if not shown otherwise
  20250518:V0.1: initial version adapted from Lora_E32.h




  */

  #pragma once

  
// Time constants
#define WAIT_FOR_PROCESSING_LIB 200 // ms This is the time required for the module to process commands.

// UART configuration
#define E32_UART_PORT UART_NUM_1
#define E32_UART_BUF_SIZE 1024      // UART buffer size

// Command and response constants
#define CONFIG_CMD_LEN 6            // Length of configuration command
#define RESPONSE_LEN 6              // Length of response

// Message delimiter constants TODO remove E32_MSG_DELIMITER and add to outside Project
// Message delimiter constants
#define E32_MSG_DELIMITER 0x0C    // First byte of message delimiter
//#define E32_MSG_DELIMITER_2 0x0C    // Second byte of message delimiter
#define E32_MIN_ADAPTIVE_POLL_MS 10 // Minimum polling interval in ms


  // e32 working modes
  enum MODE
{
    MODE_NORMAL = 0b00,
    MODE_WAKEUP = 0b01,
    MODE_POWERSAVE = 0b10,
    MODE_SLEEP_PROG = 0b11
};

  typedef struct {
    int gpio_m0;
    int gpio_m1;
    int gpio_aux;
    int gpio_txd;
    int gpio_rxd;
    int uart_port;
} e32_pins_t;


#pragma pack(push, 1) // no padding between struct members
typedef struct
{
    uint8_t airDataRate : 3;  // bit 0-2
    uint8_t uartBaudRate : 3; // bit 3-5
    uint8_t uartParity : 2;   // bit 6-7
} e32_speed_t;

typedef struct
{
    uint8_t transmissionPower : 2;  // bit 0-1
    uint8_t fec : 1;                // bit 2
    uint8_t wirelessWakeupTime : 3; // bit 3-5
    uint8_t ioDriveMode : 1;        // bit 6
    uint8_t fixedTransmission : 1;  // bit 7
} e32_option_t;

typedef struct 
{
    uint8_t HEAD;
    uint8_t ADDH;
    uint8_t ADDL;
    e32_speed_t SPED;
    uint8_t CHAN;
    e32_option_t OPTION;
} e32_config_t;
#pragma pack(pop)


enum E32_UART_PARITY
{
    E32_UART_PARITY_8N1 = 0b00,
    E32_UART_PARITY_8O1 = 0b01,
    E32_UART_PARITY_8E1 = 0b10
};

enum E32_UART_BAUD_RATE
{
    E32_UART_BAUD_RATE_1200 = 0b000,
    E32_UART_BAUD_RATE_2400 = 0b001,
    E32_UART_BAUD_RATE_4800 = 0b010,
    E32_UART_BAUD_RATE_9600 = 0b011,
    E32_UART_BAUD_RATE_19200 = 0b100,
    E32_UART_BAUD_RATE_38400 = 0b101,
    E32_UART_BAUD_RATE_57600 = 0b110,
    E32_UART_BAUD_RATE_115200 = 0b111
};

enum AIR_DATA_RATE
{
    AIR_DATA_RATE_300 = 0b000,
    AIR_DATA_RATE_1200 = 0b001,
    AIR_DATA_RATE_2400 = 0b010,
    AIR_DATA_RATE_4800 = 0b011,
    AIR_DATA_RATE_9600 = 0b100,
    AIR_DATA_RATE_19200 = 0b101
};

enum TRANSMISSION
{
    TRANSMISSION_TRANSPARENT = 0b0,
    TRANSMISSION_FIXED = 0b1
};

enum IO_DRIVE_MODE
{
    IO_DRIVE_MODE_PUSH_PULL = 0b0,
    IO_DRIVE_MODE_OPEN_DRAIN = 0b1
};

enum WIRELESS_WAKEUP_TIME
{
    WIRELESS_WAKEUP_TIME_250MS = 0b000,
    WIRELESS_WAKEUP_TIME_500MS = 0b001,
    WIRELESS_WAKEUP_TIME_750MS = 0b010,
    WIRELESS_WAKEUP_TIME_1000MS = 0b011,
    WIRELESS_WAKEUP_TIME_1250MS = 0b100,
    WIRELESS_WAKEUP_TIME_1500MS = 0b101,
    WIRELESS_WAKEUP_TIME_1750MS = 0b110,
    WIRELESS_WAKEUP_TIME_2000MS = 0b111
};

enum FEC
{
    FEC_DISABLE = 0b0,
    FEC_ENABLE = 0b1
};

enum TRANSMISSION_POWER
{
    TRANSMISSION_POWER_30dBm = 0b00,
    TRANSMISSION_POWER_27dBm = 0b01,
    TRANSMISSION_POWER_24dBm = 0b10,
    TRANSMISSION_POWER_21dBm = 0b11
};





// Callback type for delay function
typedef void (*e32_delay_callback_t)(uint32_t ms);

/**
 * @brief Receive a message with terminator character
 *
 * This function implements a polling-based receiver for messages with the following features:
 * 1. Waits for a reply for up to timeout_ms milliseconds
 * 2. Polls the LoRa module periodically using the provided delay callback
 * 3. Accumulates received data until one of these conditions is met:
 *    - A terminator character is received
 *    - The buffer size limit is reached
 *    - The timeout period expires
 *
 * @param buffer Buffer to store the received message
 * @param buffer_size Size of the buffer
 * @param message_len Pointer to store the length of the received message
 * @param terminator Character that marks the end of a message
 * @param timeout_ms Maximum time to wait for a complete message
 * @param poll_interval_ms Time between polling attempts
 * @param delay_callback Function to call for implementing delays
 * @return esp_err_t ESP_OK if successful, ESP_ERR_TIMEOUT if no complete message received
 */
esp_err_t e32_receive_message(
    uint8_t *buffer,
    size_t buffer_size,
    size_t *message_len,
    uint32_t timeout_ms,
    uint32_t poll_interval_ms,
    e32_delay_callback_t delay_callback
);

/**
 * @brief Send a message with end delimiter
 *
 * This function sends data with the end delimiter appended.
 * It handles memory allocation for the combined buffer.
 *
 * @param data Pointer to the data to send
 * @param len Length of the data
 * @return esp_err_t ESP_OK if successful, error code otherwise
 */
esp_err_t e32_send_message_with_delimiter(const uint8_t *data, size_t len);

// Set pins before calling init
void e32_init_config(e32_config_t *config);
void e32_set_pins(const e32_pins_t *pins);
void initLibrary(void);
void wait_for_aux();
void set_mode(enum MODE mode);
esp_err_t e32_send_data(const uint8_t *data, size_t len);
void sendConfiguration(e32_config_t *config);
void get_config(void);
void decode_config(uint8_t *e32_data, int e32_data_len);
esp_err_t e32_receive_data(uint8_t *buffer, size_t buffer_len, size_t *received_len);
bool e32_data_available();

// Version info for E32_Lora_Lib
// Use the version number extracted from Git tags if available
#ifdef E32_APP_VERSION_NUMBER
#define E32_LORA_LIB_VERSION "V" E32_APP_VERSION_NUMBER
#else
#define E32_LORA_LIB_VERSION "V0.6"
#endif

// Use Git version if available, otherwise fallback to a default
// Note: This needs to be a string literal, not just the macro name
#ifdef E32_APP_VERSION
#define E32_LORA_LIB_GIT_VERSION E32_APP_VERSION
#else
#define E32_LORA_LIB_GIT_VERSION "unknown-git"
#endif

// Returns the E32 library version string
const char *e32_lora_lib_get_version(void);




