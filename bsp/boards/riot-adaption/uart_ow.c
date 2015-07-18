#include "board.h"
#include "uart.h"
#include "leds.h"
#include "periph/uart.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   uart_tx_cbt txCb;
   uart_rx_cbt rxCb;
   uint8_t     startOrend;
   uint8_t     flagByte;
} uart_vars_t;

volatile uart_vars_t uart_vars;

//=========================== prototypes ======================================
void uart_rx_cb_wrapper(void *arg, char data);
int uart_tx_cb_wrapper(void *arg);

//=========================== public ==========================================

void uart_init_ow(void)
{
  // reset local variables
  memset((void*)&uart_vars,0,sizeof(uart_vars_t));

  //when this value is 0, we are send the first data
  uart_vars.startOrend = 0;
  //flag byte for start byte and end byte
  uart_vars.flagByte = 0x7E;
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb)
{
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;

    extern void uart_set_callbacks(uart_t uart, uart_tx_cb_t tx_cb, uart_rx_cb_t rx_cb, void *arg);
    uart_set_callbacks(STDIO, uart_tx_cb_wrapper, uart_rx_cb_wrapper, NULL);
}

extern void uart_rx_begin(uart_t uart);
extern void uart_rx_end(uart_t uart);
extern void uart_tx_end(uart_t uart);

void uart_enableInterrupts(void)
{
    uart_tx_begin(STDIO);
    uart_rx_begin(STDIO);
}

void uart_disableInterrupts(void)
{
    uart_tx_end(STDIO);
    uart_rx_end(STDIO);
}

void uart_clearRxInterrupts(void)
{
    extern void uart_clear_rx_interrupts(uart_t uart);
    uart_clear_rx_interrupts(STDIO);
}

void uart_clearTxInterrupts(void)
{
    extern void uart_clear_tx_interrupts(uart_t uart);
    uart_clear_tx_interrupts(STDIO);
}

void uart_writeByte(uint8_t byteToWrite)
{
    uart_write_blocking(STDIO, (char) byteToWrite);

   //start or end byte?
   if(byteToWrite == uart_vars.flagByte) {
     uart_vars.startOrend = (uart_vars.startOrend == 0)?1:0;
     //start byte
     if(uart_vars.startOrend == 1) {
        uart_tx_begin(STDIO);
     } else {
        uart_tx_end(STDIO);
     }
   }
}

uint8_t uart_readByte(void)
{
  extern int uart_read(uart_t uart, char *data);
  char temp = 0;
  uart_read(STDIO, &temp);
  return (uint8_t)temp;
}

void uart_rx_cb_wrapper(void *arg, char data)
{
  uart_vars.rxCb();
}

int uart_tx_cb_wrapper(void *arg)
{
  uart_vars.txCb();
  return 0;
}
