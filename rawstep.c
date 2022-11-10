#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "stepper.h"


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

int debug=0;

void step(int ch) {
    if (!debug) {
	putchar(0x90 | (ch&0x0f));
    } else {
	printf("step %d\n", ch);
    }
}

void dir(int cw) {
    if (!debug) {
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
    if (!debug) {
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

#define MAXSTEPS 600	// half full stroke

void center(void) {
    int i;
    mode(0);

    // go left full stroke and stall
    dir(0);
    for (i=0; i<MAXSTEPS*2; i++) {
	delay(50);
	step(7);
    }

    // go right 1/2 stroke to center piston
    dir(1);
    for (i=0; i<MAXSTEPS; i++) {
	delay(50);
	step(7);
    }
}

int main(int argc, char **argv) {
    int i;

    float ll;
    STEPPARM *s=NULL;

    float l=200.0;
    float res=1.0;	// compute integer steps

    float t0=0.0;
    float t1=0.0;

    float vmax=10.0;
    float amax=1.0;
    float fstep=20000.0;
    int zero=0;
    float interp=1.0;
    int modeset=0;

    float cycletime=0.0;	// in arbitrary units
    float period=1.0;		// in seconds

    extern int optind;
    extern char *optarg;
    int errflg = 0;
    int c;

    while ((c = getopt(argc, argv, "a:p:d:f:m:r:s:v:z")) != EOF) {
	 switch (c) {
		case 'a':                       // set acceleration limit
		    amax = atof(optarg);
		    break;
		case 'p':                       // set period in seconds
		    period = atof(optarg);
		    break;
		case 'd':
		    debug = atof(optarg);
		    break;
		case 'f':                       // set stepper update freq
		    fstep = atof(optarg);
		    break;
		case 'm':                       // set stepper mode
		    // MS3 MS2 MS1
		    // L   L   L    // full step
		    // L   L   H    // half step
		    // L   H   L    // quarter step
		    // L   H   H    // eighth step
		    // H   H   H    // sixteenth
		    interp = atof(optarg);
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
			   fprintf(stderr, "%s error: -s <mode> is one of 1,2,4,8 or 16\n", argv[0]);
			   errflg++;
			   break;
			}
		    break;
		case 's':                       // set steps pk/pk
		    l = atof(optarg);
		    break;
	        case 'v':                       // set velocity limit
                    vmax = atof(optarg);
                    break;
	        case 'z':                       // center the piston
                    zero++;
                    break;
                default:
                    errflg = 1;
                    break;
           }
    }

    if (errflg) {
	    fprintf(stderr, "usage: %s [options] < xyzwfile\n", argv[0]);
	    fprintf(stderr, "     -a <amax>   ; set acceleration limit (default=%f)\n", amax);
            fprintf(stderr, "     -d <debug>  ; verbose debugging bitmask\n");
	    fprintf(stderr, "     -f <fstep>  ; stepper update frequency (default=%f)\n", fstep);
	    fprintf(stderr, "     -p <period> ; set stroke period in seconds (default=%f)\n", period);
	    fprintf(stderr, "     -s <count>  ; set steps pk/pk (default=%f)\n", l);
	    fprintf(stderr, "     -m <factor> ; set interpolation mode (default = %f)\n", interp);
	    fprintf(stderr, "     -v <vmax>   ; set velocity limit (default=%f)\n", vmax);
	    fprintf(stderr, "     -z          ; initialize piston to midpoint (default off)\n");
	    exit(1);
    }


    // correct l by stepper interpolation factor

    l *= interp;

    // stepper_set_parms(float l, float vs, float ve, float amax, float vmax, float res) {

    if ((s=stepper_set_parms(l, 0.0, 0.0, amax, vmax, res)) == NULL) {
        fatal("STEPPARM malloc error");
    }

    cycletime = 2.0*time_at_l(s, l);

    fprintf(stderr, "cycletime is %f units\n", cycletime);
    fprintf(stderr, "period is %f\n", period);
    fprintf(stderr, "fstep is %f\n", fstep);

    float minstep=1000.0;
    float stepdel=0.0;
    int n=0;
    for (ll=0; ll<l; ll+=res) {		// forward direction
	  t1=t0; t0 = time_at_l(s, ll); n++;
	  stepdel = roundf(fabs((t0-t1)*period*fstep/cycletime));
	  if (stepdel < minstep && (n>2)) minstep=stepdel;
    }
    fprintf(stderr, "minstep is %f\n", minstep);

    fprintf(stderr, "setting mode %d = %fx\n", modeset, interp);


    // optionally zero the piston
    if (zero) center();


    if (debug) {
	for (ll=0; ll<l; ll+=res) {		// forward direction
	      t1=t0; t0 = time_at_l(s, ll);
	      //printf("%f %f\n", t0*period/fstep, ll);
	      printf("%f %f %d\n", (period*t0/cycletime), ll/interp, (int) fabs((t0-t1)*period*fstep/cycletime/interp));
	}
	for (ll=l; ll>=0; ll-=res) {		// forward direction
	      t1=t0; t0 = time_at_l(s, ll);
	      //printf("%f %f\n", t0*period/fstep, ll);
	      printf("%f %f %d\n", (period*(cycletime-t0)/cycletime), ll/interp, (int) fabs((t0-t1)*period*fstep/cycletime/interp));
	}
	exit(1);
    }

    mode(modeset); // set stepper interpolation 
	
    while (1) {
	dir(1);

	for (ll=0; ll<l; ll+=res) {		// forward direction
	      t1=t0; t0 = time_at_l(s, ll);
	      delay((int) fabs((t0-t1)*period*fstep/cycletime));
	      step(7);
	      // if (debug) printf("%f %f %f %f\n", ll, t0-t1, t0, fabs((t0-t1)*fstep*period/cycletime));
	}

	dir(0);
	for (ll=l; ll>=0; ll-=res) {		// reverse direction
	      t1=t0; t0 = time_at_l(s, ll);
	      delay((int) fabs((t0-t1)*period*fstep/cycletime));
	      step(7);
	      // if (debug) printf("%f %f %f %f\n", ll, t0-t1, t0, fabs((t0-t1)*fstep*period/cycletime));
	}
    }
}