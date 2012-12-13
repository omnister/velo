#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "interpolate.h"

//
// based on "An optimal feedrate model and solution algorithm for
// high-speed machine of small line blocks with look-ahead", Lingjian
// Xiao, Jun Hu, Yuhan Wang, Zuyu Wu, Int J Adv Manuf Technol (2004)
// 00:1-6. (web preprint of submitted paper). 
//

// [1,vs1,x1y1,l12 ], [2,vs2,x2y2,l23 ], [3,vs3,x3y3  ] 
// v<n> refers to the speed at start of interval

typedef struct node {
    double x;			// x value for start of this segment
    double y;			// y value for start of this segment
    double z;			// z value for start of this segment
    double vs;			// velocity at start of this segment
    double l;			// distance to the next segment
    int eof;			// marker for missing data
} NODE;

#define MAXLOOK 64		// maximum lookahead
#define MAXBUF 128		// maximum input x,y,z linesize

#define NLOOK 7			// default lookahead
#define AMAX 1.0		// default acceleration
#define VMAX 2.0		// default maximum velocity
#define RES  0.001		// default stepper resolution
#define FSTEP 19500.0		// servo interrupt rate
#define MINSTEP 4.0		// limit on stepper update rate

int nlook = NLOOK;
double amax = AMAX;
double vmax = VMAX;
double res = RES;
double fstep = FSTEP;
int debug = 0;

NODE nodebuf[MAXLOOK];
NODE *pnodebuf = nodebuf;
char buf[MAXBUF];
int readerrors = 0;
int nread = 0;
int n = 0;

NODE *d(int k)		// modulo access point data ring buffer 
{
    return (&nodebuf[(n + k + nlook) % nlook]);
}

int getval()	// read either "x y" or "x y z" from stdin
{
    int c;
    double x, y, z;
    n = (n + 1) % nlook;
    nodebuf[n].eof = 0;
    if (fgets(buf, MAXBUF, stdin) == NULL) {
	nodebuf[n].x = 0.0;
	nodebuf[n].y = 0.0;
	nodebuf[n].z = 0.0;
	nodebuf[n].vs = 0.0;
	nodebuf[n].l = 0.0;
	nodebuf[n].eof = 1;
    } else if (sscanf(buf, "%lf %lf %lf", &x, &y, &z) == 3) {
	nodebuf[n].x = x;
	nodebuf[n].y = y;
	nodebuf[n].z = z;
	nodebuf[n].vs = 0.0;
	nodebuf[n].l = 0.0;
	nread++;
    } else if (sscanf(buf, "%lf %lf", &x, &y) == 2) {
	nodebuf[n].x = x;
	nodebuf[n].y = y;
	nodebuf[n].z = 0.0;
	nodebuf[n].vs = 0.0;
	nodebuf[n].l = 0.0;
	nread++;
    } else {
	readerrors++;
	fprintf(stderr, "error: on line %d, \"%s\"\n", nread, buf);
    }
    if (nread > 1) {
	d(-1)->l = sqrt(pow((d(0)->x - d(-1)->x), 2.0) +
			pow((d(0)->y - d(-1)->y), 2.0) +
			pow((d(0)->z - d(-1)->z), 2.0) );
    }
    return (readerrors);
}


main(int argc, char **argv)
{
    int done = 0;
    int i;
    double x0, y0, z0;
    double x1, y1, z1;
    double x2, y2, z2;
    double cosine;
    double vv;
    double ltotal = 0.0;
    double ttotal = 0.0;

    extern int optind;
    extern char *optarg;
    int errflg = 0;
    int c;

    while ((c = getopt(argc, argv, "a:n:r:dv:")) != EOF) {
	switch (c) {
	case 'a':			// set acceleration limit
	    amax = atof(optarg);
	    break;
	case 'd':
	    debug++;
	    break;
	case 'f':			// set stepper update freq
	    fstep = atof(optarg);
	    break;
	case 'n':			// set lookahead
	    nlook = atoi(optarg);
	    if (nlook < 3) nlook=3;
	    if (nlook > MAXLOOK-1) nlook=MAXLOOK-1;
	    break;
	case 'r':			// set resolution
	    res = atof(optarg);
	    break;
	case 'v':			// set velocity limit
	    vmax = atof(optarg);
	    break;
	default:
	    errflg = 1;
	    break;
	}
    }

    if (errflg) {
	fprintf(stderr, "usage: %s [options] < xyzfile\n", argv[0]);
	fprintf(stderr, "     -a <amax>  ; set acceleration limit\n");
	fprintf(stderr, "     -d         ; verbose debugging info\n");
	fprintf(stderr, "     -f <fstep> ; stepper update frequency\n");
	fprintf(stderr, "     -n <nlook> ; set lookahead length \n");
	fprintf(stderr, "     -r <res>   ; set stepper resolution\n");
	fprintf(stderr, "     -v <vmax>  ; set velocity limit\n");
	exit(1);
    }

    if (debug) {
    fprintf(stderr, "-a (%8.3g) ;accelleration limit (inches/second^2)\n", amax);
    fprintf(stderr, "-f (%8.3g) ;stepper update frequency\n", fstep);
    fprintf(stderr, "-n (%8d) ;number of segments lookahead\n", nlook);
    fprintf(stderr, "-r (%8.3g) ;stepper step size (inches)\n", res);
    fprintf(stderr, "-v (%8.3g) ;velocity limit (inches/second)\n", vmax);
    fprintf(stderr, "\n");
    fprintf(stderr, "%8.3g min step spacing (updates)\n", fstep/(vmax/res)); 
    }

    if ((fstep/(vmax/res)) < MINSTEP) {
        fprintf(stderr, "%s error: exceeded maximum allowable velocity\n",
	argv[0]);
        fprintf(stderr, 
	    "min steps/update = (fstep*res)/(vmax) = %g (must be >= %g)\n",
	    fstep/(vmax/res), MINSTEP);
        exit(2); 
    }

    for (i = 0; i < nlook - 1; i++) {	// fill buffer
	getval();
    }

    while (!done) {
	getval();

	// calculate maximum acceptable velocity at a segment to
	// still be able to turn the next corner without exceeding
	// the AMAX acceleration limit.

	for (i = 2; i < nlook; i++) {
	    if (d(i + 1)->eof == 1) {
		d(i)->vs = 0.0;
	    } else {
		x0 = d(i - 1)->x;
		y0 = d(i - 1)->y;
		z0 = d(i - 1)->z;
		x1 = d(i)->x;
		y1 = d(i)->y;
		z1 = d(i)->z;
		x2 = d(i + 1)->x;
		y2 = d(i + 1)->y;
		z2 = d(i + 1)->z;

		// efficient calculation of cosine of the bend angle in 3d

		cosine = (x2 - x1) * (x1 - x0) +
		       (y2 - y1) * (y1 - y0) + (z2 - z1) * (z1 - z0);
		cosine /= sqrt(pow((x1 - x0), 2.0) + pow((y1 - y0), 2.0) +
		       pow((z1 - z0), 2.0));
		cosine /= sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0) +
		       pow((z2 - z1), 2.0));

		if (sqrt(2.0 - 2.0 * cosine) < (amax * res / vmax)) {
		    d(i)->vs = vmax;
		    if (debug>1)
			printf("1 vs = %g %g: %g %g %g, %g %g %g, %g %g %g\n",
			     d(i)->vs, cosine, x0, y0, z0, x1, y1, z1, x2, y2, z2);
		} else {
		    d(i)->vs = amax * res / sqrt(2.0 - 2.0 * cosine);
		    if (debug>1)
			printf("2 vs = %g %g: %g %g %g, %g %g %g, %g %g %g\n",
			     d(i)->vs, cosine, x0, y0, z0, x1, y1, z1, x2, y2, z2);
		}
	    }
	}

	// calculate decelleration limits due to finite segment
	// length.  Assume last segment in look ahead is a full stop
	//
	// V = V0 + A*t
	// L = V0*t + A*t^2/2
	// V(L) = sqrt(V0^2 + 2*A*L);

	for (i = nlook - 1; i > 1; i--) {	// backwards chaining
	    vv = sqrt(pow(d(i + 1)->vs, 2.0) + 2.0 * amax * d(i)->l);
	    if (debug>1)
		printf("di+1vs=%g divs=%g vv==%g\n", d(i + 1)->vs,
		       d(i)->vs, vv);
	    d(i)->vs = min(d(i)->vs, vv);
	    d(i)->vs = min(d(i)->vs, vmax);
	}

	// forward chaining 
	vv = sqrt(pow(d(1)->vs, 2.0) + 2.0 * amax * d(1)->l);
	d(2)->vs = min(d(2)->vs, vv);

	if (debug>1) {
	    printf("------------------\n");
	    for (i = 1; i <= nlook; i++) {
		printf("n:%d i:%d x:%g y:%g z:%g eof:%d vs:%g l:%g\n",
		       nread, i, d(i)->x, d(i)->y, d(i)->z, d(i)->eof,
		       d(i)->vs, d(i)->l);
	    }
	}


	// initialize velocity calculation code
	setseg(d(i)->l,d(i)->vs,d(i+1)->vs,vmax,amax,res, fstep);

	ttotal+=interpolate(d(i)->x, d(i)->y, d(i)->z, d(i+1)->x, d(i+1)->y,
	d(i+1)->z, ttotal, ltotal);

	ltotal+=d(i)->l;

	if (d(3)->eof == 1)
	    done++;
    }
}

