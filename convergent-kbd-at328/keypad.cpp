/* keypad.cpp
 *  
 *  This is an optional directly-connected keypad for 12 of the Convergent
 *  special keys that are along the left-hand side of the keyboard. Emulating
 *  these on a PC keyboard gets a bit complicated, so support the option of
 *  constructing a separate dedicated keypad for just those keys.
 *  
 *  These are implemented the same way as the keypad map for the ps2 keyboard.
 *  There's a bitmap (key_state) of keys in the "down" state.
 */

#include <Arduino.h>
#include "keypad.h"

// mapping of bits in keypad_state_bitmap to convergent scancodes
uint8_t keymap[12] = {CONVERGENT_PAGENEXT, CONVERGENT_SCROLLNEXT,
          CONVERGENT_PAGEPREV, CONVERGENT_SCROLLPREV,
          CONVERGENT_COPY, CONVERGENT_FINISH,
          CONVERGENT_OVERTYPE, CONVERGENT_ACTION,
          CONVERGENT_DELETE, CONVERGENT_HELP,
          CONVERGENT_MOVE, CONVERGENT_CANCEL};

uint8_t keypad_state[16], keypad_event;
uint16_t keypad_state_bitmap, keypad_state_bitmap_last;

void keypad_init()
{
  uint8_t i;
  keypad_state_bitmap_last = 0xCF;  // all keys up
  keypad_event = 0;
  for (i=0; i<16; i++) {
    keypad_state[i] = 0;
  }

  // turn on pullups
  PORTB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB6) | (1<<PB7);
  PORTC |= (1<<PC4) | (1<<PC5);
  PORTD |= (1<<PD3) | (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7);
}

uint16_t keypad_scan()
{
   uint16_t bits;

   bits = 0;

   // The keypad lines are distributed wildly around the ports as
   // I had to make room for some of the specialty peripherals
   // (SPI, UART, etc). Still, could have been a little more 
   // orderly.

   if ((PIND & (1<<PD3))==0) {
       bits |= KA1;
   }
   if ((PIND & (1<<PD4))==0) {
       bits |= KA2;
   }
   if ((PINC & (1<<PC4))==0) {
       bits |= KA3;
   }
   if ((PINC & (1<<PC5))==0) {
       bits |= KA4;
   }
   if ((PIND & (1<<PD5))==0) {
       bits |= KB1;
   }
   if ((PIND & (1<<PD6))==0) {
       bits |= KB2;
   }
   if ((PIND & (1<<PD7))==0) {
       bits |= KB3;
   }
   if ((PINB & (1<<PB0))==0) {
       bits |= KB4;
   }
   if ((PINB & (1<<PB1))==0) {
       bits |= KB5;
   }
   if ((PINB & (1<<PB2))==0) {
       bits |= KB6;
   }
   if ((PINB & (1<<PB6))==0) {
       bits |= KB7;
   }
   if ((PINB & (1<<PB7))==0) {
       bits |= KB8;
   }

   return bits;
}

void keypad_update()
{
  keypad_state_bitmap = keypad_scan();
  if (keypad_state_bitmap != keypad_state_bitmap_last) {
    uint16_t t;
    uint8_t i;

    t=1;
    for (i=0; i<12; i++) {
      uint8_t convergent_code = keymap[i];
      uint8_t mask = 1<<(convergent_code & 0x07);
      if (keypad_state_bitmap & t) {
        keypad_state[convergent_code >> 3] |= mask;
      } else {
        keypad_state[convergent_code >> 3] &= ~mask;
      }
      t=t<<1;
    }
    keypad_event = 1;
    keypad_state_bitmap_last = keypad_state_bitmap;
  }
}
