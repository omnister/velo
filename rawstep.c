#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "stepper.h"
#include "bytecodes.h"

#define FULLSTROKE 1200		// full stroke
#define STEPPERREV 200.0	// stepper motor steps per rev
#define ULPERREV   9.496	// microliters per rev

int debug=0;

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
    float flow=100.0;		// uliters/min

    extern int optind;
    extern char *optarg;
    int errflg = 0;
    int c;

    while ((c = getopt(argc, argv, "a:c:p:d:f:m:r:s:v:z")) != EOF) {
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
		case 'c':                       // set stepper update freq
		    fstep = atof(optarg);
		    break;
		case 'f':                       // set flow in ul/min
		    flow = atof(optarg);
		    l=FULLSTROKE*sqrt(flow/2279.0);
		    period=2.0*(ULPERREV*60.0*l)/(flow*STEPPERREV);
		    break;
		case 'm':                       // set stepper mode
		    interp = atof(optarg);
		    if ((modeset=interp2mode(interp)) < 0) {
			fprintf(stderr, "%s error: -s <mode> is one of 1,2,4,8 or 16\n", argv[0]);
			errflg++;
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
	    fprintf(stderr, "     -a <amax>    ; set acceleration limit (default=%f)\n", amax);
            fprintf(stderr, "     -d <debug>   ; verbose debugging bitmask\n");
	    fprintf(stderr, "     -c <fstep>   ; stepper update frequency (default=%f)\n", fstep);
	    fprintf(stderr, "     -f <uliters> ; set flow in ul/min (default=%f)\n", flow);
	    fprintf(stderr, "     -p <period>  ; set stroke period in seconds (default=%f)\n", period);
	    fprintf(stderr, "     -s <count>   ; set steps pk/pk (default=%f)\n", l);
	    fprintf(stderr, "     -m <factor>  ; set interpolation mode (default = %f)\n", interp);
	    fprintf(stderr, "     -v <vmax>    ; set velocity limit (default=%f)\n", vmax);
	    fprintf(stderr, "     -z           ; initialize piston to midpoint (default off)\n");
	    fprintf(stderr, "	100uL is approximately -p5 -s150\n");
	    exit(1);
    }


    // stepper_set_parms(float l, float vs, float ve, float amax, float vmax, float res) {

    if ((s=stepper_set_parms(l, 0.0, 0.0, amax, vmax, res)) == NULL) {
        fatal("STEPPARM malloc error");
    }

    cycletime = 2.0*time_at_l(s, l);

    // fprintf(stderr, "cycletime is %f units\n", cycletime);
    // fprintf(stderr, "fstep is %f\n", fstep);

#define STEPPERREV 200.0	// stepper motor steps per rev
#define ULPERREV   9.496	// microliters per rev

    fprintf(stderr, "stroke is %f steps\n", l);
    fprintf(stderr, "period is %f\n", period);
    fprintf(stderr, "flow is %f uL/min \n", 2.0*ULPERREV*(l/STEPPERREV)*(60.0/period));

    // do a dry run to compute the minimum step time in fstep clock cycles
    float minstep=1000.0;
    float stepdel=0.0;
    int n=0;
    for (ll=0; ll<l; ll+=res) {		// forward direction
	  t1=t0; t0 = time_at_l(s, ll); n++;
	  stepdel = roundf(fabs((t0-t1)*period*fstep/cycletime));
	  if (stepdel < minstep && (n>2)) minstep=stepdel;
    }
    fprintf(stderr, "minstep is %f\n", minstep);

    // if step size is too big (the stepper will be noisy and vibrate excessively)
    // then increase the interpolation factor to get in to the sweet spot
    while (minstep > 20.0 && interp < 16) {
	fprintf(stderr, "	tuning interp:%f minstep:%f\n", interp, minstep);
	if (minstep  < 20) break;
	interp*=2.0;
	minstep/=2.0;
    }
    // if step size is too small (can't generate them with a fixed clock)
    // then decrease the interpolation factor to get in to the sweet spot
    while (minstep < 5.0 && interp > 1) {
	fprintf(stderr, "	tuning interp:%f minstep:%f\n", interp, minstep);
	if (minstep  > 5.0) break;
	interp/=2.0;
	minstep*=2.0;
    }

    // if after all that, the minstep is still too small, then the motor just
    // cant do what is asked.  Bail out with a help message
    //
    // if minstep is still large, it'll be noisy, but we'll live with it and
    // do the best we can

    if (minstep < 12.0) {
	fprintf(stderr, "step size too small, either decrease stroke or increase cycle time: interp:%f minstep:%f\n", interp, minstep);
	exit(7);
    } else {
	fprintf(stderr, "optimized step interpolation: interp:%f minstep:%f\n", interp, minstep);
    }

    res = 1.0/interp;

    if ((modeset=interp2mode(interp)) < 0) {
	fprintf(stderr, "%s error: -s <mode> is one of 1,2,4,8 or 16\n", argv[0]);
	exit(4);
    }

    // optionally initialize the piston

    if (zero) center(FULLSTROKE);


    if (debug) {
	t0 = time_at_l(s,res);
	for (ll=0; ll<l; ll+=res) {		// forward direction
	      t1=t0; t0 = time_at_l(s, ll);
	      //printf("%f %f\n", t0*period/fstep, ll);
	      printf("%f %f %d\n", (period*t0/cycletime), ll, (int) fabs((t0-t1)*period*fstep/cycletime));
	}
	for (ll=l; ll>=0; ll-=res) {		// forward direction
	      t1=t0; t0 = time_at_l(s, ll);
	      //printf("%f %f\n", t0*period/fstep, ll);
	      printf("%f %f %d\n", (period*(cycletime-t0)/cycletime), ll, (int) fabs((t0-t1)*period*fstep/cycletime));
	}
	exit(1);
    }

    mode(modeset); // set stepper interpolation 
	
    while (1) {
	dir(1);

	t0 = time_at_l(s,res);
	for (ll=res; ll<l; ll+=res) {		// forward direction
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
