/* 
 * ps2convergent.c
 *
 * PS2 Keyboard on a Convergent NGEN.
 *
 * (c) Scott Baker, https://www.smbaker.com/
 */

// Some of the PS2 code originally based on keyboard.c
//   for NerdKits with ATmega168
//   hevans@nerdkits.com


//#define F_CPU 14745600

#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/delay.h>

#include "convergent_scancode_map.h"

// NOTE: Swap PB0 and PB2 !!!
// NOTE: Swap PB1 and PB3 !!!

//PIN configuration
//PB4          -  NGEN Reset
//PB3          -  PS2 Keyboard DATA
//PB2 (PCINT2) -  PS2 Keyboard CLK
//PB1          -  NGEN KBD_To_Comp
//PB0          -  NGEN Comp_to_kbd

#define CONVERGENT_RESET (1<<PB4)
#define CONVERGENT_TX (1<<PB1)
#define CONVERGENT_RX (1<<PB0)

#define PS2_INT PCINT2
#define PS2_CLK (1<<PB2)
#define PS2_DATA (1<<PB3)

#define PS2_CAPSLOCK_SCANCODE 0x58
#define PS2_NUMLOCK_SCANCODE 0x77
#define PS2_PAUSE_SCANCODE 0x77
#define PS2_BACKSLASH_SCANCODE 0x5D
#define PS2_LSHIFT_SCANCODE 0x12
#define PS2_RSHIFT_SCANCODE 0x59
#define PS2_BACKQUOTE_SCANCODE 0x0E

#define CAPSLOCK_LEDBIT 0x04
#define NUMLOCK_LEDBIT 0x02

#define LOW 0
#define HIGH 1

volatile uint8_t kbd_data;
volatile uint8_t kbd_event;
volatile uint8_t ack_event, nak_event;
volatile uint8_t next_extended, e1flag;
volatile uint8_t key_state[256];
volatile uint8_t disable_isr;
volatile uint8_t shift_down;
volatile uint8_t last_ps2_clk, last_convergent_rx;
uint8_t convergent_code;
uint8_t started;
uint8_t bit_count;
uint8_t shift;
uint8_t caps_lock;
uint8_t extended;
uint8_t release;
uint8_t capslock;
uint8_t numlock;
uint8_t leds;
uint8_t ps2_argument;
uint8_t ps2_have_argument;

uint8_t convergent_map_code(uint8_t data, uint8_t extended);

void serial_rx_init();
void serial_rx_isr();
void ps2_clk_isr();

// There's only one ISR, so share it between the PS2 RX and
// the serial RX.
ISR(PCINT0_vect)
{
    uint8_t t;

    t = PINB & PS2_CLK;
    if (t!=last_ps2_clk) {
        ps2_clk_isr();
        last_ps2_clk = t;
    }

    t = PINB & CONVERGENT_RX;
    if (t!=last_convergent_rx) {
        serial_rx_isr();
        last_convergent_rx = t;
    }
}

void ps2_clk_isr() {

  //make sure clock line is low, if not ignore this transition
  if(PINB & PS2_CLK) {
    return;
  }

  if (disable_isr) {
    return;
  }

  //if we have not started, check for start bit on DATA line
  if(!started){
    if ( (PINB & PS2_DATA == 0 ) {
      started = 1;
      bit_count = 0;
      kbd_data = 0;
      //printf_P(PSTR("%d"),started);
      return;
    }
  } else if(bit_count < 8) { //we started, read in the new bit
    //put a 1 in the right place of kdb_data if PC2 is high, leave
    //a 0 otherwise
    if (PINB & PS2_DATA) {
      kbd_data |= (1<<bit_count);
    }
    bit_count++;
    return;
  } else if(bit_count == 8){ //pairty bit
    //not implemented
    bit_count++;
    return;
  } else {  //stop bit
    //should check to make sure DATA line is high, what to do if not?
    started = 0;
    bit_count = 0;
  }

  if(kbd_data == 0xF0){ //release code
    release = 1;
    kbd_data = 0;
    return;
  } else if (kbd_data == 0xFA) {
    ack_event = 1;
    return;
  } else if (kbd_data == 0xFE) {
    nak_event = 1;
    return;
  } else if (kbd_data == 0xE0) { //extended code
    next_extended = 1;
    return;
  } else if (kbd_data == 0xE1) { //the other extended code (pause/break)
    e1flag = 1;
    return;
  } else { //not a special character
    if ((e1flag) && (kbd_data!=0x77) && (kbd_data!=0x14)) {
      // E1,14,77,E1,F0,14,F0,77 is the pause sequence.
      // If we see something else, reset the flag.
      e1flag = 0;
    }
    if (release) { //we were in release mode - exit release mode
      if ((kbd_data == PS2_LSHIFT_SCANCODE) || (kbd_data == PS2_RSHIFT_SCANCODE)) {
          shift_down = 0;
      }
      release = 0;
      convergent_code = convergent_map_code(kbd_data, next_extended);
      next_extended = 0;
      if (convergent_code != 0) {
          key_state[convergent_code] = 0;
          kbd_event = 1;
      }
    } else {
      convergent_code = convergent_map_code(kbd_data, next_extended);
      next_extended = 0;
      if (convergent_code != 0) {
          key_state[convergent_code] = 0;
          kbd_event = 1;
      }
    }
  }
}

void led_init()
{
  // make pb5 an output
  DDRB |= (1<<PB5);
}

void led_on()
{
  PORTB |= (1<<PB5);
}

void led_off()
{
  PORTB &= ~(1<<PB5);
}

void init_keyboard()
{
  uint8_t i;

  for (i=0; i<255; i++) {
    key_state[i] = 0;
  }

  started = 0;
  kbd_data = 0;
  bit_count = 0;
  next_extended = 0;
  e1flag = 0;
  kbd_event = 0;
  disable_isr = 0;
  shift_down = 0;
  ps2_have_argument = 0;
  ack_event = 0;
  nak_event = 0;

  //make PS2_CLK input pin
  //DDRB &= ~PS2_CLK;
  //turn on pullup resistor
  PORTB |= PS2_CLK;

  //Set the mask on Pin change interrupt 1 so that ISR for PS2_CLK fires
  //the interrupt.
  PCMSK |= (1<<PS2_INT);
}

void init_isr()
{
  last_ps2_clk = PINB & PS2_CLK;
  last_convergent_rx = PINB & CONVERGENT_RX;

  // Clear pin change interrupt flag.
  GIFR = 1<<PCIF;

  //Enable PIN Change Interrupt 1 - This enables interrupts on pins
  GIMSK |= (1<<PCIE);
}

uint8_t wait_for_bit(uint8_t bit, uint8_t highlow, uint16_t ms)
{
    if (highlow != LOW) {
      highlow = bit;
    }

    uint16_t i;
    for (i=0; i<ms*1000; i++) {
      if ((PINB & bit) == highlow) {
        return 1;
      }
      _delay_us(1);
    }
    // timeout
    return 0;
}

uint8_t compute_parity(uint16_t n) 
{ 
    uint8_t parity = 0; 
    while (n) 
    { 
        parity = !parity; 
        n      = n & (n - 1); 
    }         
    return parity; 
} 

void ps2_clk_low()
{
  PORTB &= ~PS2_CLK;
  DDRB |= PS2_CLK;
}

void ps2_clk_release()
{
  // open-collector output; to set high we just release
  DDRB &= ~PS2_CLK;
}

void ps2_data_low()
{
  PORTB &= ~PS2_DATA;
  DDRB |= PS2_DATA;
}

void ps2_data_release()
{
  // open-collector output; to set high we just release
  DDRB &= ~PS2_DATA;
}

uint8_t ps2_send_byte(uint16_t b)
{
    int i;

    wait_for_bit(PS2_CLK, HIGH, 2);

    // bring clock low
    ps2_clk_low();
    // wait 100us
    _delay_us(300);
    // bring data low
    ps2_data_low();
    _delay_us(1);
    // return clock to input state
    ps2_clk_release();

    // wait for keyboard to bring clock low, up to 15ms
    if (!wait_for_bit(PS2_CLK, LOW, 15)) {
      ps2_data_release();
      return 0;
    }

    if (!compute_parity(b)) {
      b |= 0x100;
    }

    // 8 data bits + one parity bit, assume the parity is in bit position 8
    for (i=0; i<9; i++) {
      // set the data bit
      if ((b&1)==1) {
        ps2_data_release();
      } else {
        ps2_data_low();
      }
      // wait for clock to go high
      if (!wait_for_bit(PS2_CLK, HIGH, 2)) {
        ps2_data_release();
        return 0;
      }
      if (!wait_for_bit(PS2_CLK, LOW, 2)) {
        ps2_data_release();
        return 0;
      }
      b = b >> 1;
    }
    
    // return data to inptu state
    ps2_data_release();

    // wait for the ack
    if (!wait_for_bit(PS2_DATA, LOW, 2)) {
      return 0;
    }
    if (!wait_for_bit(PS2_CLK, LOW, 2)) {
      return 0;
    }

    return 1;
}

void ps2_send_command_argument(uint8_t command, uint8_t argument)
{
  disable_isr = 1;
  if (ps2_send_byte(command)) {
      ps2_argument = argument;
      ps2_have_argument = 1;
  }
  disable_isr = 0;
}

void ps2_send_argument(uint8_t arg)
{
  disable_isr = 1;
  ps2_send_byte(ps2_argument);
  disable_isr = 0;
}

void convergent_init()
{
  // make TX output
  DDRB |= CONVERGENT_TX;
}

uint8_t convergent_map_code(uint8_t data, uint8_t extended)
{
    uint8_t scancode;

    if (extended!=0) {
        scancode = pgm_read_byte(&(extcode_map[data]));
    } else {
        scancode = pgm_read_byte(&(scancode_map[data]));
    }

    return scancode;
}

void convergent_send_packet()
{
  uint16_t i;

  for (i=0; i<256; i++) {
  }
}

int main() {
  init_isr();

  init_keyboard();

  convergent_init();

  serial_rx_init();

  led_init();

  // Tweak clock speed for 5V, comment out if running ATtiny at 3V
  OSCCAL += 3;

  //enable interrupts
  sei();

  leds = 0;

  while(1) {
    if (kbd_event != 0) {
        convergent_send_packet();
    }
    if (ack_event != 0) {
      if (ps2_have_argument) {
        ps2_send_argument(ps2_argument);
        ps2_have_argument = 0;
      }
      ack_event = 0;
    }
    if (nak_event != 0) {
        ps2_have_argument = 0;
        nak_event = 0;
    }
  }

  return 0;
}
