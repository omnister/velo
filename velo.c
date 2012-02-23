#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "interpolate.h"

// based on Chinese paper ~ "Optimized velocity profiles for CNC"

// [1,vs1,x1y1,l12        ], [2,vs2,x2y2,l23         ], [3,vs3,x3y3      ] 
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
#define AMAX 0.038		// default acceleration
#define VMAX 0.5		// default maximum velocity
#define RES  1.0		// default stepper resolution

int nlook = NLOOK;
double amax = AMAX;
double vmax = VMAX;
double res = RES;
int debug = 0;

NODE nodebuf[MAXLOOK];
NODE *pnodebuf = nodebuf;
char buf[MAXBUF];
int readerrors = 0;
int nread = 0;
int n = 0;

void interpolate();

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
			pow((d(0)->z - d(-1)->z), 2.0)
	    );
    }
    return (readerrors);
}

double min(double x, double y)
{
    return ((x < y) ? x : y);
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
	fprintf(stderr, "usage: %s [-n<nlook>][-d (debug)]\n", argv[0]);
	fprintf(stderr, "     [-n <nlook>] ; set lookahead length \n");
	fprintf(stderr, "     [-d] ; verbose debugging info\n");
	exit(1);
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

		if (sqrt(2.0 - 2.0 * cosine) < amax * res / vmax) {
		    d(i)->vs = vmax;
		    if (debug)
			printf("1 vs = %g %g: %g %g %g, %g %g %g, %g %g %g\n",
			     d(i)->vs, cosine, x0, y0, z0, x1, y1, z1, x2, y2, z2);
		} else {
		    d(i)->vs = amax * res / sqrt(2.0 - 2.0 * cosine);
		    if (debug)
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
	    if (debug)
		printf("di+1vs=%g divs=%g vv==%g\n", d(i + 1)->vs,
		       d(i)->vs, vv);
	    d(i)->vs = min(d(i)->vs, vv);
	    d(i)->vs = min(d(i)->vs, vmax);
	}

	// forward chaining 
	vv = sqrt(pow(d(1)->vs, 2.0) + 2.0 * amax * d(1)->l);
	d(2)->vs = min(d(2)->vs, vv);

	if (debug) {
	    printf("------------------\n");
	    for (i = 1; i <= nlook; i++) {
		printf("n:%d i:%d x:%g y:%g z:%g eof:%d vs:%g l:%g\n",
		       nread, i, d(i)->x, d(i)->y, d(i)->z, d(i)->eof,
		       d(i)->vs, d(i)->l);
	    }
	}

//	printf("(%g, %g, %g) => (%g, %g, %g)\tltotal:%7.3g\tdl:%7.3g\tvstart: %7.3g\tvend: %7.3g\n",
//	     d(i)->x, d(i)->y, d(i)->z, d(i + 1)->x, d(i + 1)->y,
//	     d(i + 1)->z, ltotal += d(i)->l, d(i)->l, d(i)->vs, d(i + 1)->vs);


	interpolate(d(i)->x, d(i)->y, d(i)->z, d(i + 1)->x, d(i + 1)->y, d(i+1)->z, d(i)->vs, d(i+1)->vs, ltotal);
	ltotal+=d(i)->l;

	if (d(3)->eof == 1)
	    done++;
    }
    printf("%g %g (%g, %g, %g)\n", ltotal, 0.0, d(i+1)->x, d(i+1)->y, d(i+1)->z);
}


// extern void setseg(
//     double x1, double y1, double z1, 
//     double x2, double y2, double z2, 
//     double vvs, double vve, double vvmax, double aamax, double rres); 

// extern double timeatl(double l);

void interpolate(double x1, double y1, double z1, 
    double x2, double y2, double z2, 
    double vstart, double vend, double dist) {

    double vm;		// peak value for segment
    double lseg;	// length of this segment
    double s1,s2,s3;	// distances of rampup,constant,rampdown
    double ts1, ts2;	// time to reach s1,s2

    lseg = sqrt(pow((x2-x1),2.0) + pow((y2-y1),2.0) + pow((z2-z1),2.0));

    vm = min(vmax, sqrt((pow(vstart,2.0) + pow(vend,2.0) + 2.0*amax*lseg)/2.0));

    s1 = (vm*vm-vstart*vstart)/(2.0*amax);
    if (fabs(s1) < res/100.0) s1 = 0.0;

    s3 = (vm*vm-vend*vend)/(2.0*amax);
    if (fabs(s3) < res/100.0) s3 = 0.0;

    s2 = lseg-s1-s3;
    if (fabs(s2) < res/100.0) s2 = 0.0;

    ts1 = (sqrt(vstart*vstart+2.0*amax*s1)-vstart)/amax;
    ts2 = ts1 + s2*vm;

    printf("# %8.4g %8.4g %8.4g vs:%g ve:%g \n", s1/lseg, s2/lseg, s3/lseg,vstart,vend);
    // printf("#lseg: %g s1: %g s2: %g s3: %g vs:%g ve:%g \n", lseg, s1, s2, s3,vstart,vend);

    printf("%g %g (%g, %g, %g)\n",  
    	dist, vstart, x1, y1, z1);
    if (s1 < (lseg - res)) {
	printf("%g %g (%g, %g, %g)\n",  
	    dist+s1, vm, x1+(x2-x1)*s1/lseg, y1+(y2-y1)*s1/lseg, z1+(z2-z1)*s1/lseg);
    }
    if (s2 > res) {
	printf("%g %g (%g, %g, %g)\n",  
		dist+s1+s2, vm, x1+(x2-x1)*(s1+s2)/lseg, y1+(y2-y1)*(s1+s2)/lseg, z1+(z2-z1)*(s1+s2)/lseg);
    }
}
