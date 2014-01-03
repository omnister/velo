// Servo drip feed
// using 18F4550 built in full-speed USB and CCS CDC serial driver

#include <18F4550.h>
#device high_ints=true
#priority int_rtcc, int_rda
#fuses HS,NOWDT,NOPROTECT,NOLVP      // use x4 PLL
#fuses PLL5,CPUDIV1,HSPLL,VREGEN
#use delay(clock=48M, oscillator=20M)   // set clock to 40 MHz

// uncomment to use bootloader
// #include "bootloader.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <usb_cdc.h>

// fifo defines
// FIXME: tuning
// size of txque0 should be bigger than rxque0

#define NBUF  600
#define LOBUF 200
#define HIBUF 400

#include <queue.c>

QUEUE rxque0;	// incoming RS-232 byte-codes
QUEUE txque0;	// outgoing stepper pulse bytes


// #use fast_io(A)			// optimize output speed
// #use fast_io(D)

#use rs232(baud=230400, xmit=PIN_C6, rcv=PIN_C7, errors)

#ZERO_RAM		// set all RAM to zero on startup

#define RTS	PIN_B0		// FTDI output low ready for input
#define CTS	PIN_B1		// PIC output low ready for input
#define STROBE  PIN_A1		// parallel port strobe

#define XMASK 	0x01
#define YMASK 	0x02
#define ZMASK 	0x04
#define WMASK 	0x08

#define LED0     PIN_C0		 // status LEDs
#define LED1     PIN_C1
#define LED2     PIN_C2 	 // now PWM output

// #define CONN1	+5
// #define CONN2 	+5
// #define CONN4	+5
// #define CONN29	GND
// #define CONN31	GND
// #define CONN32	GND

#define CONN21	 PIN_A0		//	ena
#define CONN22	 PIN_A1		// 	rst
#define CONN19	 PIN_A2		// 	ms1
#define CONN20	 PIN_A3		// 	ms2
#define CONN17	 PIN_A4		//	ms3
#define CONN18	 PIN_A5
#define CONN25	 PIN_B2		// 	sleep
#define CONN28	 PIN_B3
#define CONN27	 PIN_B4

#define CONN14	 PIN_C0
#define CONN11	 PIN_C1		
// #define CONN9	 PIN_C3
#define CONN6	 PIN_C4
#define CONN3	 PIN_C5


// fixme: rev 2 should be dir 3210, step 3210 
// instead of interleaved to improve processing speed

#define CONN10	 PIN_D0		//	dir0
#define CONN7	 PIN_D1		//	stp0
#define CONN8	 PIN_D2		//	dir1
#define CONN5	 PIN_D3		//	stp1
#define CONN30	 PIN_D4		//	dir2
#define CONN24	 PIN_D5		//	stp2
#define CONN23	 PIN_D6		//	dir3
#define CONN26	 PIN_D7		//	stp3

#define CONN12	 PIN_C2		// CCP1 PWM output to spindle

#define CONN15	 PIN_E0
#define CONN16	 PIN_E1
#define CONN13	 PIN_E2

// MS3 MS2 MS1
// L   L   L	// full step
// L   L   H	// half step
// L   H   L	// quarter step
// L   H   H	// eighth step
// H   H   H	// sixteenth
// ---------------
// STEP
// L->H advances motor
// 200 ns minimum setup/hold on DIR/MS/RST
// 1uS minimum pulse width
// ---------------
// DIR
// L	CW
// H	CCW
// ---------------
// RESET
// H 	NORMAL MODE
// L    RESET - all outputs off
// ---------------
// ENABLE
// H	all outputs off
// L    normal operation
// ---------------
// port D (7:0) S3,D3 S2,D2 S1,D1 S0,D0
// port A (7:0) X,X,MS3,MS2,MS1,RST,ENA

void enable(int8 ss) { 		// enable if ss != 0	
    output_bit(PIN_B2, ss);	// sleep (active low)
    output_bit(PIN_A0, !ss);	// enable (active low)
    output_bit(PIN_A1, ss);	// reset active high
}

// MS3 MS2 MS1
// L   L   L	// full step
// L   L   H	// half step
// L   H   L	// quarter step
// L   H   H	// eighth step
// H   H   H	// sixteenth

void set_step(int8 ss) { 	// set ms3,ms2,ms1
    output_bit(PIN_A4, ss&4);
    output_bit(PIN_A3, ss&2);
    output_bit(PIN_A2, ss&1);
}

// forward declarations
void init();
void blink();

void housekeep() {
    usb_task(); 	// service periodic usb functions
    while (usb_cdc_kbhit() && (nqueue(&rxque0) < HIBUF)) {
	enqueue(&rxque0, usb_cdc_getc());
    }
}

void feed(char *c) {
   while (nqueue(&txque0) >= NBUF-4) {
        output_bit(LED0,1);
	housekeep(); 	// was a spin lock
   }
   output_bit(LED0,0);
   enqueue(&txque0, c);
}

// implements simple s-code interpreter
// DELAY   (7:0) '0'nnn nnnn  ; delay n+1 counts
// DIR     (7:0) '1000' wxyz  ; direction (0=ccw, 1=cw)
// STEP    (7:0) '1001' wxyz  ; step (1=step, 0=idle)
// MODE    (7:0) '1010' 0mmm  ; set ustep mode
// STAT    (7:0) '1010' 1res  ; set reset, enable, sleep
// ----    (7:0) '1011' ----  ; undefined
// SPIN	   (7:0) '11'nn nnnn  ; duty cycle is n/64 0=off


void main() {
    unsigned int8 d;	// delay value
    unsigned int8 i;	
    unsigned int8 obyte=0;
    int16 spin;		// pwm 0->1023
    char c;

    // port D (7:0) SW,DW SZ,DZ SY,DY SX,DX

    init();

    usb_cdc_init();
    usb_init();
    usb_wait_for_enumeration();

    while (TRUE) {

	// get chars from usb
	housekeep();

	// now process the input and feed output queue
        if (nqueue(&rxque0) > 0) {
	   c = dequeue(&rxque0);
	   if ((c&0xf0) == 0x90) {		// step code
              obyte &= 0x55;	// zero step bits
	      if (c&XMASK) { obyte|= 0x02; } 
	      if (c&YMASK) { obyte|= 0x08; } 
	      if (c&ZMASK) { obyte|= 0x20; } 
	      if (c&WMASK) { obyte|= 0x80; } 
	      feed(obyte);
	   } else if ((c&0x80)==0) {	 	// delay code
	      d = ((unsigned int8)c)&0x7f;
	      for (i=1; i<=d; i++) {
	         feed(0x00);	// stall
	      }
	   } else if ((c&0xf0) == 0x80) {	// dir code
              obyte &= 0xaa;	// zero dir bits
	      if (c&XMASK) { obyte|= 0x01; } 
	      if (c&YMASK) { obyte|= 0x04; } 
	      if (c&ZMASK) { obyte|= 0x10; } 
	      if (c&WMASK) { obyte|= 0x40; } 
	   } else if ((c&0xf0) == 0xa0) {	
	      if ((c&0x08)) {
		  d = ((unsigned int8)c)&0x07;
		  enable(d);			// state
	      } else {
		  d = ((unsigned int8)c)&0x07;
		  set_step(d);			// mode
	      }
	   } else if ((c&0xc0) == 0xc0) {	// spindle PWM speed	
	       spin=c&0x3f;	// get the bits 
	       spin = spin<<4;
	       set_pwm1_duty(spin);	// duty cycle is val/(4*(255+1))
	   } else {
	      ; // unknown code, silently ignore
	   }
        } 
    }
}

#int_rtcc  high
void irq_rtcc_hi()
{
    int8 val;
    
    if (nqueue(&txque0) > 0) {
       val=dequeue(&txque0);
       output_d(val);			// output
       output_d(val&0x55);		// mask step (force low)
    }
}

void blink() {
    int i;
    for (i=0; i<4; i++) {
    	output_high(LED0); output_high(LED1);
    	delay_ms(125);
    	output_low(LED0); output_low(LED1);
    	delay_ms(125);
    }
}

// INTS_PER_SECOND  2441	// (40e6/(4*16*256)) // div by 16
// INTS_PER_SECOND  9766	// (40e6/(4*4*256))  // div by 4
// INTS_PER_SECOND 19531	// (40e6/(4*2*256))  // div by 2
//
// INTS_PER_SECOND 23437	// (48e6/(4*2*256))  // div by 2
// INTS_PER_SECOND 46875	// (48e6/(4*1*256))  // div by 1

void init() {
   unsigned int16 i;

   blink();

   init_queue(&rxque0);
   init_queue(&txque0);

   set_step(1);		// 0=full, 1=1/2, 5=1/4, 4=1/8, 7=1/16
   enable(1);

   //setup_counters(T0_INTERNAL, T0_DIV_1 | T0_8_BIT);
   setup_counters(T0_INTERNAL, T0_DIV_2 | T0_8_BIT);

   set_timer0(0);

   enable_interrupts(INT_RTCC);
   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);

   output_bit(CTS,0);		// ready to receive from usb
   output_bit(STROBE,1);	// parallel port strobe

   // input(PIN_C3);		// ESTOP
   input(PIN_C4);		// ALIM
   input(PIN_C0);		// XLIM
   input(PIN_C1);		// YLIM
   
   // input(PIN_C2);		// ZLIM
   output_bit(PIN_C2,0);	// changed from ZLIM to PWM OUTPUT

   setup_ccp1(CCP_PWM);
   setup_timer_2(T2_DIV_BY_16, 255, 1); // mode, period, postscale
   set_timer2(0);
   set_pwm1_duty(0);		// duty cycle is val/(4*(255+1))
}
		
