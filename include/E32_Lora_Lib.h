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

