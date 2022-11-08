#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define XMASK 1
#define YMASK 2
#define ZMASK 4
#define WMASK 8

static double vs, ve;		// start and end velocity
static double vm;		// peak velocity value for segment
static double lseg;		// length of this segment
static double s1,s2,s3;		// distances of rampup,constant,rampdown
static double ts1, ts2;		// time to reach s1,s2
static double vmax;
static double amax;
static double res;
static int pen;
static double fupdate;		// pic (servo) interrupt frequency

extern int debug;

int xloc=0;			// actual step counts
int yloc=0;
int zloc=0;
int wloc=0;

double max(double a, double b)
{
    if (a > b) return a;
    return b;
}

double min(double a, double b)
{
    if (a < b) return a;
    return b;
}

// set globals for further computation on this segment

void setseg(double llseg, 
            double vvs, double vve, 
	    double vvmax, double aamax, 
	    double rres, double f) {

    vs = vvs; ve = vve;
    vmax=vvmax;
    amax=aamax;
    res=rres;
    lseg=llseg;
    fupdate=f;

    // peak velocity for this segment
    vm = min(vmax, sqrt((pow(vs,2.0) + pow(ve,2.0) + 2.0*amax*lseg)/2.0));

    // ramp up distance (s1), constant v (s2) and ramp down (s3)
    s1 = (vm*vm-vs*vs)/(2.0*amax); if (fabs(s1) < res/100.0) s1 = 0.0;
    s3 = (vm*vm-ve*ve)/(2.0*amax); if (fabs(s3) < res/100.0) s3 = 0.0;
    s2 = lseg-s1-s3; if (fabs(s2) < res/100.0) s2 = 0.0;

    // time to reach s1, s2
    ts1 = (sqrt(vs*vs+2.0*amax*s1)-vs)/amax;
    ts2 = ts1 + s2/vm;

    if (debug & 1) {
     fprintf(stderr,"#setseg: lseg:%g vmax:%g amax:%g vs:%g \
     ve:%g vm:%g s1:%g s2:%g s3:%g ts1:%g ts2:%g\n", 
     lseg, vmax, amax, vs, ve, vm, s1, s2, s3, ts1, ts2);
    }
}

// return the time it takes to get to fraction alpha of this segment
double time2alpha(double alpha) {

      double l = alpha*lseg;
      double asave;
      double tt;
      int clipped = 0;

      if (alpha > 1.0) {
	 asave=alpha;
         alpha = 1.0;
	 clipped++;
      }

      if (l < s1) {             // accellerating
         tt = (sqrt(vs*vs+2.0*amax*l)-vs)/amax;
	 pen = 2;
      } else if (l < s1+s2) {   // cruising
         tt = ts1 + (l-s1)/vm;
	 pen = 3;
      } else {                  // decellerating
         tt = ts2 + (vm - sqrt(fabs(vm*vm - 2.0*amax*(l-(s2+s1)))))/amax;
	 pen = 4;
      }
      if (debug & 1) {
        printf("atl: alpha:%g l:%g s1:%g s2:%g ts1:%g ts2:%g vm:%g amax:%g tt:%g\n", 
        alpha, l, s1, s2, ts1, ts2, vm, amax, tt);
      }
      if (clipped) return (tt*asave);
      return(tt);
}

double interpolate(double x1, double y1, double z1, double w1,
                   double x2, double y2, double z2, double w2, 
		   double ttotal, double ltotal) {
    double xx, yy, zz, ww;
    double alpha;
    double alphax=0.0;
    double alphay=0.0;
    double alphaz=0.0;
    double alphaw=0.0;
    double xdir, ydir, zdir, wdir;
    int xs, ys, zs, ws;			// was there a step?
    int x0, y0, z0, w0;
    int minstep = 0;
    int minstep2 = 0;
    int xstep, ystep, zstep, wstep;	// next step counts
    int delay;
    int done;

    unsigned char mask;		// bit mask for advancing xyzw
    unsigned char stepmask;	// mask for non-zero dims
    unsigned char dirmask;	// mask for direction 1=increasing
    double eax, eay, eaz, eaw;	// end points for alpha calc
    
    xx=x1; yy=y1; zz=z1, ww=w1;

    xdir=(x2 > x1)?1.0:-1.0;
    ydir=(y2 > y1)?1.0:-1.0;
    zdir=(z2 > z1)?1.0:-1.0;
    wdir=(w2 > w1)?1.0:-1.0;

    dirmask = 0;
    if (x2 > x1) dirmask |= XMASK;
    if (y2 > y1) dirmask |= YMASK;
    if (z2 > z1) dirmask |= ZMASK;
    if (w2 > w1) dirmask |= WMASK;

    if (debug&4) {
	fprintf(stderr,"DIR 0x%.2x\n", dirmask);
    } else {
        putchar(0x80 | dirmask);
    }

// DELY    (7:0) '0'nnn nnnn  ; delay n+1 counts
// DIR     (7:0) '1000' xyzw  ; direction (0=ccw, 1=cw)
// STEP    (7:0) '1001' xyzw  ; step (1=step, 0=idle)
// MODE    (7:0) '1010' 0mmm  ; set ustep mode
// STAT    (7:0) '1010' 1res  ; set reset, enable, sleep

    x0 = x1;
    y0 = y1;
    z0 = z1;
    w0 = w1;

    stepmask = 0;
    if (fabs(x2-x1) > 0.5*res) stepmask |= XMASK;
    if (fabs(y2-y1) > 0.5*res) stepmask |= YMASK;
    if (fabs(z2-z1) > 0.5*res) stepmask |= ZMASK;
    if (fabs(w2-w1) > 0.5*res) stepmask |= WMASK;

    // I assumed the factor 0.0*res below should have
    // been 0.5 to minimize error, but 0.0 empirically
    // 0.0 gives errors bounded by +/- res, so I leave
    // it set to 0.0, although it makes the equation 
    // redundant...

    alphax = alphay = alphaz = alphaw = 0.0;
    eax = 1.0 - fabs(0.0*res/(x2-x1));
    eay = 1.0 - fabs(0.0*res/(y2-y1));
    eaz = 1.0 + fabs(0.0*res/(z2-z1));
    eaw = 1.0 + fabs(0.0*res/(w2-w1));

    mask = XMASK | YMASK | ZMASK | WMASK;	// initial step in all dims
    xstep = ystep = zstep = wstep = 0;

    done = 0;

    while(!done) {
    // while(stepmask) {

       if (debug&16) {
	   fprintf(stderr,
	   	"xs:%d ys:%d zs:%d ws:%d ms:%d ms2:%d mask:%.2x sm:%.2x\n", 
	   	xstep, ystep, zstep, wstep, minstep, minstep2, mask, stepmask);
	   fprintf(stderr,
	   	"xx:%g x2:%g y:%g y2:%g z:%g z2:%g w:%g w2:%g\n", 
		xx, x2, yy, y2, zz, z2, ww, w2);
	   fprintf(stderr,"ax:%g ex:%g ay:%g ey:%g az:%g ez:%g aw:%g ew:%g\n", 
	       alphax, eax, alphay, eay, alphaz, eaz, alphaw, eaw);
       }

       done=1;	// set it and then conditionally clear it

       if ((alphax < eax) && stepmask & XMASK) {
            if(mask & XMASK) { 
	       xx+=res*xdir;
	   } 
	   alphax = (xx-x1)/(x2-x1);
	   if (alphax > eax) { 
		// done so take this axis out of the running
		stepmask &= ~XMASK;
	   } else {
	   	done=0;
	   }
	   xstep = (int)(time2alpha(alphax)*fupdate);	
       }

       if ((alphay < eay) && stepmask & YMASK) {
	   if (mask & YMASK) {
	       yy+=res*ydir;
	   }
	   alphay = (yy-y1)/(y2-y1);
	   if (alphay > eay) { 
		stepmask &= ~YMASK;
	   } else {
	   	done=0;
	   }
	   ystep = (int)(time2alpha(alphay)*fupdate);	
       }

       if ((alphaz < eaz) && stepmask & ZMASK) {
       	   if (mask & ZMASK) {
	       zz+=res*zdir;
	   } 
           alphaz = (zz-z1)/(z2-z1);
	   if (alphaz > eaz) { 
		stepmask &= ~ZMASK;
	   } else {
	   	done=0;
	   }
           zstep = (int)(time2alpha(alphaz)*fupdate);	
       }

       if ((alphaw < eaw) && stepmask & WMASK) {
           if (mask & WMASK) {
	       ww+=res*wdir;
	   }
	   alphaw = (ww-w1)/(w2-w1);
	   if (alphaw > eaw) { 
		stepmask &= ~WMASK;
	   } else {
	   	done=0;
	   }
	   wstep = (int)(time2alpha(alphaw)*fupdate);	
       }

       // set minstep to the step value of the first stepped axis

       if (stepmask & XMASK) {
	   minstep = xstep;
	   mask = XMASK;
       } else if (stepmask & YMASK) {
	   minstep = ystep;
	   mask = YMASK;
       } else if (stepmask & ZMASK) {
	   minstep = zstep;
	   mask = ZMASK;
       } else if (stepmask & WMASK) {
	   minstep = wstep;
	   mask = WMASK;
       }

       // now give every other axis a chance to override
       // a lower minstep, or tag along at current minstep

       if (stepmask & YMASK) {
	   if (ystep < minstep) {
	       minstep = ystep;
	       mask = YMASK;
	   } else if (ystep == minstep) {
	       mask |= YMASK;
	   }
       }

       if (stepmask & ZMASK) {
	   if (zstep < minstep) {
	       minstep = zstep;
	       mask = ZMASK;
	   } else if (zstep == minstep) {
	       mask |= ZMASK;
	   }
       }

       if (stepmask & WMASK) {
	   if (wstep < minstep) {
	       minstep = wstep;
	       mask = WMASK;
	   } else if (wstep == minstep) {
	       mask |= WMASK;
	   }
       }

       delay=(minstep-minstep2);

       if (delay != 0) {

	   // all done, update step locations

	   if (mask & XMASK) { xloc += (int) xdir; }
	   if (mask & YMASK) { yloc += (int) ydir; }
	   if (mask & ZMASK) { zloc += (int) zdir; }
	   if (mask & WMASK) { wloc += (int) wdir; }

	   // printf("%d %d, %.2x ", minstep-minstep2, minstep, mask);
	   // printf("%d %d %d %d %g %g %g %g\n", 
	   //	xstep, ystep, zstep, wstep, xx, yy, zz, ww);

	   if (debug&4) {
	       fprintf(stderr,"DEL %d\n", minstep-minstep2);
	       fprintf(stderr,"STP 0x%.2x\n", mask);
	   } else {
	       delay=(minstep-minstep2);

	       if (delay > 5000) { 
		    if (debug&16) {
			fprintf(stderr, "clipping bad delay val: %d\n", delay);
		    }
		    delay = 5000;		// defensive programming
	       }

	       while(delay>=128) {
		  putchar(0x7f);
		  delay-=128;
	       }
	       if (delay > 0) {
		   putchar(delay&0x7f);
	       }
	       putchar(0x90 | mask);
	    }
	}
        minstep2 = minstep;
    }


    return (time2alpha(1.0));
}
