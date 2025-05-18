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

  
  #define WAIT_FOR_PROCESSING_LIB 200 // ms This is the time required for the module to process commands.
// UART configuration
#define E32_UART_PORT UART_NUM_1
#define BUF_SIZE 1024

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

// Set pins before calling init
void init_io(void);
void e32_set_pins(const e32_pins_t *pins);
void func(void);
void wait_for_aux();
void set_mode(enum MODE mode);
esp_err_t e32_send_data(const uint8_t *data, size_t len);
void sendConfiguration(e32_config_t *config);
void get_config(void);
void decode_config(uint8_t *e32_data, int e32_data_len);
esp_err_t e32_receive_data(uint8_t *buffer, size_t buffer_len, size_t *received_len);
void get_config(void);
void decode_config(uint8_t *data, int len);
bool e32_data_available();




