#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "stepper.h"
#include "bytecodes.h"

void mode(int modeset);
void step(int ch);
void dir(int cw);
void delay(int cnt);
void center(int steps);
int interp2mode(float interp);

// implements simple s-code interpreter
// DELAY   (7:0) '0'nnn nnnn  ; delay n+1 counts
// DIR     (7:0) '1000' wxyz  ; direction (0=ccw, 1=cw)
// STEP    (7:0) '1001' wxyz  ; step (1=step, 0=idle)
// MODE    (7:0) '1010' 0mmm  ; set ustep mode
// 	MS3 MS2 MS1
// 	L   L   L    // full step
// 	L   L   H    // half step
// 	L   H   L    // quarter step
// 	L   H   H    // eighth step
// 	H   H   H    // sixteenth
// STAT    (7:0) '1010' 1res  ; set reset, enable, sleep
// ----    (7:0) '1011' ----  ; undefined
// SPIN    (7:0) '11'nn nnnn  ; duty cycle is n/64 0=off

int bdebug=0;

void step(int ch) {
    if (!bdebug) {
	putchar(0x90 | (ch&0x0f));
    } else {
	printf("step %d\n", ch);
    }
}

void dir(int cw) {
    if (!bdebug) {
	if (cw) {
	    putchar(0x80);	// cw
	} else {
	    putchar(0x8f);	// ccw
	}
    } else {
	printf("dir %d\n", cw);
    }
}

void mode(int modeset) {
    putchar(0xa0 | (modeset&0x07));
}

// delay cnt interrupt steps
void delay(int cnt) {
    if (!bdebug) {
	// fprintf(stderr, "delay called with %d\n", cnt);
	while (cnt > 128) {
	    // fprintf(stderr, "delay 128\n");
	    putchar(0x7f);
	    cnt-=128;
	}
	if (cnt > 0) {
	    // fprintf(stderr, "delay %d\n", cnt);
	    putchar(cnt&0x7f);
	}
    } else {
	printf("delay %d\n", cnt);
    }
}


// center the piston by driving it into wall by full stroke
// and then stepping back to center by half the full stroke
// Should be done before setting mode, because this routine
// sets mode to full stepping 

void center(int steps) {
    int i;
    mode(0);

    // go left full stroke and stall
    dir(0);
    for (i=0; i<steps; i++) {
	delay(50);
	step(7);
    }

    // go right 1/2 stroke to center piston
    dir(1);
    for (i=0; i<steps/2; i++) {
	delay(50);
	step(7);
    }
}

// given an interpolation value 1,2,4,8,16 return the number to be sent
// to stepper controller, else -1

int interp2mode(float interp) {
    int modeset=-1;
    // MS3 MS2 MS1
    // L   L   L    // full step
    // L   L   H    // half step
    // L   H   L    // quarter step
    // L   H   H    // eighth step
    // H   H   H    // sixteenth
    switch ((int) interp) {
	case 1:
	   modeset=0;      // single step
	   break;
	case 2:
	   modeset=1;      // half step
	   break;
	case 4:
	   modeset=2;      // quad step
	   break;
	case 8:
	   modeset=3;      // eighth step
	   break;
	case 16:
	   modeset=7;      // eighth step
	   break;
	default:
	   modeset=-1;
	   break;
    }
    return modeset;
}

