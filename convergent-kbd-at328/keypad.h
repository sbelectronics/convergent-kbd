#ifndef keypad_h
#define keypad_h

#define KA1 0x01
#define KA2 0x02
#define KA3 0x04
#define KA4 0x08

#define KB1 0x10
#define KB2 0x20
#define KB3 0x40
#define KB4 0x80
#define KB5 0x100
#define KB6 0x200
#define KB7 0x400
#define KB8 0x800

#define CONVERGENT_MARK       0x02
#define CONVERGENT_BOUND      0x03
#define CONVERGENT_FINISH     0x04
#define CONVERGENT_PAGEPREV   0x05
#define CONVERGENT_CANCEL     0x07
#define CONVERGENT_PAGENEXT   0x0C
#define CONVERGENT_SCROLLNEXT 0x11
#define CONVERGENT_MOVE       0x12
#define CONVERGENT_SCROLLPREV 0x13
#define CONVERGENT_COPY       0x14
#define CONVERGENT_ACTION     0x43
#define CONVERGENT_OVERTYPE   0x44
#define CONVERGENT_HELP       0x00
#define CONVERGENT_DELETE     0x7F

extern uint8_t keypad_state[16];
extern uint8_t keypad_event;

void keypad_init();
void keypad_update();

#endif
