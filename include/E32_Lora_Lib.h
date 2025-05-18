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

// Set pins before calling init
void e32_set_pins(const e32_pins_t *pins);

void func(void);
void wait_for_aux();
void set_mode(enum MODE mode);
esp_err_t e32_send_data(const uint8_t *data, size_t len);

