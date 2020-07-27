/* 
 * convergent-kbd-tiny85.ino
 *
 * PS2 Keyboard on a Convergent NGEN.
 *
 * (c) Scott Baker, https://www.smbaker.com/
 */

// Some of the PS2 code originally based on keyboard.c
//   for NerdKits with ATmega168
//   hevans@nerdkits.com


#include <Arduino.h>

#include "convergent_scancode_map.h"
#include "keypad.h"


//PIN configuration
//PD2           -  NGEN Reset
//PD1           -  NGEN KBD_To_Comp
//PD0 (PCINT16) -  NGEN Comp_to_kbd
//PC2 (PCINT10) -  PS2 Keyboard CLK
//PC3           -  PS2 Keyboard DATA

#define CONVERGENT_TX_PIN PD1
#define CONVERGENT_RX_PIN PD0

#define CONVERGENT_RESET (1<<PD2)
#define CONVERGENT_TX (1<<CONVERGENT_TX_PIN)
#define CONVERGENT_RX (1<<CONVERGENT_RX_PIN)

#define PS2_INT PCINT10
#define PS2_CLK (1<<PC2)
#define PS2_DATA (1<<PC3)

#define PS2_CAPSLOCK_SCANCODE 0x58
#define PS2_NUMLOCK_SCANCODE 0x77
#define PS2_PAUSE_SCANCODE 0x77
#define PS2_BACKSLASH_SCANCODE 0x5D
#define PS2_LSHIFT_SCANCODE 0x12
#define PS2_RSHIFT_SCANCODE 0x59
#define PS2_BACKQUOTE_SCANCODE 0x0E

#define CAPSLOCK_LEDBIT 0x04
#define NUMLOCK_LEDBIT 0x02

#define SCANCODE_NONE 0xFF

#define LOW 0
#define HIGH 1

volatile uint8_t kbd_data;
volatile uint8_t kbd_event;
volatile uint8_t ack_event, nak_event;
volatile uint8_t next_extended, e1flag;
volatile uint8_t key_state[16];
volatile uint8_t disable_isr;
volatile uint8_t shift_down;
uint8_t convergent_code;
uint8_t started;
uint8_t bit_count;
uint8_t shift;
uint8_t caps_lock;
uint8_t extended;
uint8_t release;
uint8_t capslock;
uint8_t numlock;
uint8_t ps2_argument;
uint8_t ps2_have_argument;
uint8_t last_reset;
uint8_t convergent_enabled;
uint8_t convergent_echo;

uint8_t convergent_map_code(uint8_t data, uint8_t extended);

ISR(PCINT1_vect)
{
  //make sure clock line is low, if not ignore this transition
  if(PINC & PS2_CLK) {
    return;
  }

  if (disable_isr) {
    return;
  }

  //if we have not started, check for start bit on DATA line
  if(!started){
    if ( (PINC & PS2_DATA) == 0 ) {
      started = 1;
      bit_count = 0;
      kbd_data = 0;
      //printf_P(PSTR("%d"),started);
      return;
    }
  } else if(bit_count < 8) { //we started, read in the new bit
    //put a 1 in the right place of kdb_data if PC2 is high, leave
    //a 0 otherwise
    if (PINC & PS2_DATA) {
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
      if (convergent_code != SCANCODE_NONE) {
          uint8_t mask = 1<<(convergent_code & 0x07);
          key_state[convergent_code >> 3] &= ~mask;
          kbd_event = 1;
      }
    } else {
      convergent_code = convergent_map_code(kbd_data, next_extended);
      next_extended = 0;
      if (convergent_code != SCANCODE_NONE) {
          uint8_t mask = 1<<(convergent_code & 0x07);
          key_state[convergent_code >> 3] |= mask;
          kbd_event = 1;
      }
    }
  }
}

void led_init()
{
  // make pc5 an output
  DDRC |= (1<<PC0);
}

void led_on()
{
  PORTC |= (1<<PC0);
}

void led_off()
{
  PORTC &= ~(1<<PC0);
}

void init_keyboard()
{
  uint8_t i;

  for (i=0; i<16; i++) {
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
  //DDRC &= ~PS2_CLK;
  //turn on pullup resistor
  PORTC |= PS2_CLK;
  
  //Set the mask on Pin change interrupt 1 so that ISR for PS2_CLK fires
  //the interrupt.
  PCMSK1 |= (1<<PS2_INT);
}

void init_isr()
{
  // Clear pin change interrupt flag.
  //PCIFR |= (1<<PCIF1);

  //Enable PIN Change Interrupt 1 - This enables interrupts on pins
  PCICR |= (1<<PCIE1);
}

uint8_t wait_for_bit(uint8_t bit, uint8_t highlow, uint16_t ms)
{
    if (highlow != LOW) {
      highlow = bit;
    }

    uint16_t i;
    for (i=0; i<ms*1000; i++) {
      if ((PINC & bit) == highlow) {
        return 1;
      }
      delayMicroseconds(1);
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
  PORTC &= ~PS2_CLK;
  DDRC |= PS2_CLK;
}

void ps2_clk_release()
{
  // open-collector output; to set high we just release
  DDRC &= ~PS2_CLK;
}

void ps2_data_low()
{
  PORTC &= ~PS2_DATA;
  DDRC |= PS2_DATA;
}

void ps2_data_release()
{
  // open-collector output; to set high we just release
  DDRC &= ~PS2_DATA;
}

uint8_t ps2_send_byte(uint16_t b)
{
    int i;

    wait_for_bit(PS2_CLK, HIGH, 2);

    // bring clock low
    ps2_clk_low();
    // wait 100us
    delayMicroseconds(300);
    // bring data low
    ps2_data_low();
    delayMicroseconds(1);
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
  last_reset = (PIND & CONVERGENT_RESET);
  convergent_enabled = false;
  convergent_echo = false;
}

uint8_t convergent_map_code(uint8_t data, uint8_t extended)
{
    uint8_t scancode;

    if (data >= 128) {
        // Out of bounds of the scancode_map array.
        // Could be a control responds from the keyboard
        //    AA = BAT successful, etc
        return SCANCODE_NONE;
    }

    if (extended!=0) {
        scancode = pgm_read_byte(&(extcode_map[data]));
    } else {
        scancode = pgm_read_byte(&(scancode_map[data]));
    }

    // this should NOT happen.
    if (scancode >= 128) {
        return SCANCODE_NONE;
    }

    return scancode;
}

void convergent_send_packet()
{
  uint8_t i;
  uint8_t prev = SCANCODE_NONE;

  led_on();

  // The largest possible scancode is 127.
  
  for (i=0; i<128; i++) {
    uint8_t mask = 1<<(i & 0x07);
    if (((key_state[i>>3] & mask) != 0) || ((keypad_state[i>>3] & mask) !=0)) {
      if (prev!=SCANCODE_NONE) {
        Serial.write(prev);
      }
      prev = i;
    }
  }

  if (prev != SCANCODE_NONE) {
    // send the last key, with the high bit set to signal end of scan
    Serial.write(prev | 0x80);
  } else {
    // no keys down
    Serial.write(0xC0);
  }

  led_off();
}

void send_id_sequence()
{
  uint8_t i;

  led_on();

  for (i=0; i<120; i++) {
    Serial.write(0xFE);
  }
  // Even though we have only one port, diagnostics
  // will complain if we don't identify a keyboard
  // as dual-ported.
  Serial.write(0xB0); // keyboard, no boot request, dual-ported
  Serial.write(0x04); // K4 Keyboard

  convergent_enabled = false;
  convergent_echo = false;

  led_off();
}

void handle_serial_input(uint8_t b)
{
  if ((b == 0x92) && (convergent_echo)) {
    convergent_echo = false;
  }
  
  if (convergent_echo) {
    Serial.write(b);
    return;
  }
  
  switch (b) {
    case 0x92:
        // software reset
        convergent_enabled = true;
        // force a keyboard scan, should report C0
        kbd_event = 1;
        break;
    case 0x8C:
        // rom checksum test
        led_on();
        Serial.write(0xF0);
        led_off();
        break;
    case 0x9E:
        convergent_echo = true;
        break;
    default:
        break;
  }
}

void check_hardware_reset() {
  // hardware reset is on the transition from low to high
  uint8_t reset = (PIND & CONVERGENT_RESET);
  if (reset && (!last_reset)) {
    // reset went from low to high
    send_id_sequence();
  }
  last_reset = reset;
}

void setup() {
  init_isr();

  init_keyboard();

  convergent_init();

  keypad_init();

  Serial.begin(1200);

  led_init();

  //enable interrupts
  sei();

  send_id_sequence();
}

void loop() {
    int ser_in;
  
    check_hardware_reset();
    keypad_update();
    
    if (Serial.available() > 0) {
        handle_serial_input(Serial.read());  
    }
    
    if (kbd_event || keypad_event) {
        convergent_send_packet();
        kbd_event = 0;
        keypad_event = 0;
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
