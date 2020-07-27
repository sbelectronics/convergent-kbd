# PS2 Keyboard to Convergent Technologies NGEN Keyboard Adapter

Scott Baker, http://www.smbaker.com/

## Purpose

This is an adapter that lets you use a PS2 keyboard on a Convergent Technologies NGEN workstation. Most of these workstations date to the 80s or 90s. The keyboard interface is a 1200 baud serial protocol that is unique to this architecture, making it problematic to get these old computers working unless you also happen to have the appropriate keyboard.

This adapter allows you to use a PS2 keyboard, which are readily available.

## Convergent special-purpose function keys

The Convergent includes a series of special-purpose keys on the left side of the keyboard. These include keys like the "Help" key, the "Cancel" and "Finish" keys, "Action", etc. Most of these I've attempted to emulate with other keys on a 104 key PS2 keyboard.

I also added the ability to build the adapter with a set of 12 Cherry MX keys. This gives a more Convergent-like feel to the keyboard rather than trying to hunt down bizarre mappings on 104 key keyboard. These additional keys are entirely optional -- you can build the adapter with or without them.

## My first prototype, using ATTINY85

My first prototype used an ATTINY85 processor. I've kept the code in this repo for historical reasons. This adapter was compact, but had a few functionality issues due to the need to implement a software UART -- it was possible to hit the PS2 keyboard fast enough to eventually cause the serial and/or PS2 timing to glitch and register an incorrect keyboard event. Maybe with some work that would be fixable, but switching to a microcontroller with a real UART was a better solution.

## My second (and final) prototype using ATMEGA328

This version used an ATMEGA328, which has a hardware UART. Much simpler and more reliably than trying to make do with software UART. The 328 also has a number of additional GPIO, which I used to implement the 12 additional special-function keys. There were enough spare GPIO that I didn't bother to multiplex the keyboard scanning.

This is the prototype you want to build -- it's in the `convergent-keyboard-at328` directory.

## Fuses

Make sure to burn the fuses in addition to burning the code. The fuses are generally at the defaults with the exception of CKDIV8. There's a screenshot of the fuse settings in minipro.

## Schematics

The Schematics are in the schematics directory. My blog post at https://www.smbaker.com/restoring-a-convergent-technologies-ngen-pc-workstation-burroughs-b25-unisys has a lot more detail on the design work, both on the keyboard adapter and on a video adapter.

The schematics are checked in here into the "schematics" directory. You'll find two schematics, one for the adapter with 4 optional special-function keys, and another for an addon board that adds the remaining 8 special-function keys. As mentioned above, the keys are optional. This was split into two boards to make the board production cost-effective (two small boards can be ordered as "prototypes" whereas one large board must be ordered as production, from the board house that I use).



