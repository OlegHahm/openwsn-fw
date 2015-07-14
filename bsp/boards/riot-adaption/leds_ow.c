#include "board.h"
#include "leds.h"

void _delay(void);

void leds_init_ow(void)
{
}

/* red */
void leds_error_on(void)
{
  LED_RED_ON;
}

void leds_error_off(void)
{
  LED_RED_OFF;
}
void leds_error_toggle(void)
{
  LED_RED_TOGGLE;
}
uint8_t leds_error_isOn(void)
{
  return true;
}
void leds_error_blink(void) {}

/* green */
void leds_sync_on(void)
{
  LED_GREEN_ON;
}
void leds_sync_off(void)
{
  LED_GREEN_OFF;
}
void leds_sync_toggle(void)
{
  LED_GREEN_TOGGLE;
}
uint8_t leds_sync_isOn(void)
{
  return true;
}

/* orange */
void leds_radio_on(void)
{
  LED_ORANGE_ON;
}
void leds_radio_off(void)
{
  LED_ORANGE_OFF;
}
void leds_radio_toggle(void)
{
  LED_ORANGE_TOGGLE;
}
uint8_t leds_radio_isOn(void)
{
  return true;
}

/* yellow */
void leds_debug_on(void) {}
void leds_debug_off(void) {}
void leds_debug_toggle(void) {}
uint8_t leds_debug_isOn(void) {
    return 0;
}

void leds_all_on(void)
{
  leds_error_on();
  leds_sync_on();
  leds_radio_on();
}
void leds_all_off(void)
{
  leds_error_off();
  leds_sync_off();
  leds_radio_off();
}
void leds_all_toggle(void)
{
  leds_error_toggle();
  leds_sync_toggle();
  leds_radio_toggle();
}

void leds_circular_shift(void)
{
  leds_error_toggle();
  _delay();
  leds_sync_toggle();
  _delay();
  leds_radio_toggle();
  _delay();
}

void leds_increment(void) {}

//=========================== private =========================================

void _delay(void)
{
  uint32_t i;
  for(i=0; i<0xfffff; i++);
}
