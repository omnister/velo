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

double max(double x, double y)
{
    if (x > y) return x;
    return y;
}

double min(double x, double y)
{
    if (x < y) return x;
    return y;
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

    // printf("#setseg: lseg:%g vmax:%g amax:%g vs:%g \
    // ve:%g vm:%g s1:%g s2:%g s3:%g ts1:%g ts2:%g\n", 
    // lseg, vmax, amax, vs, ve, vm, s1, s2, s3, ts1, ts2);
}

// return the time it takes to get to fraction alpha of this segment
double time2alpha(double alpha) {

      double l = alpha*lseg;
      double tt;

      // printf("atl: l:%g s1:%g s2:%g ts1:%g ts2:%g vm:%g amax:%g\n", 
      // l, s1, s2, ts1, ts2, vm, amax);
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
      return(tt);
}

double interpolate(double x1, double y1, double z1, 
                    double x2, double y2, double z2, 
		    double ttotal, double ltotal) {
    double xx, yy, zz;
    double alpha;
    double alphax=0.0;
    double alphay=0.0;
    double alphaz=0.0;
    double xdir, ydir, zdir;
    int xs, ys, zs;		// was there a step?
    int x0, y0, z0;
    int minval;
    int xstep, ystep, zstep;	// next step counts
    int minval2 = 0;

    unsigned char mask;		// bit mask for advancing wxyz
    unsigned char stepmask;	// mask for non-zero dims
    unsigned char dirmask;	// mask for direction 1=increasing
    
    xx=x1; yy=y1; zz=z1;

    xdir=(x2 > x1)?1.0:-1.0;
    ydir=(y2 > y1)?1.0:-1.0;
    zdir=(z2 > z1)?1.0:-1.0;

    dirmask = 0;
    if (x2 > x1) dirmask |= XMASK;
    if (y2 > y1) dirmask |= YMASK;
    if (z2 > z1) dirmask |= ZMASK;

    printf("DIR 0x%0.2x\n", dirmask);

    x0 = x1;
    y0 = y1;
    z0 = z1;

    stepmask = 0;
    if (x2 != x1) stepmask |= XMASK;
    if (y2 != y1) stepmask |= YMASK;
    if (z2 != z1) stepmask |= ZMASK;

    alphax = alphay = alphaz = 0.0;

    mask = WMASK | XMASK | YMASK | ZMASK;	// initial step in all dims
    xstep = ystep = zstep = 0;

    while((fabs(xx-x2)>res) || (fabs(yy-y2)>res) || (fabs(zz-z2)>res)) {

       if (stepmask & XMASK && mask & XMASK) { 
	   xx+=res*xdir;
           alphax = (xx-x1)/(x2-x1);
	   xstep = (int)(time2alpha(alphax)*fupdate);	
       }

       if (stepmask & YMASK && mask & YMASK) {
	   yy+=res*ydir;
           alphay = (yy-y1)/(y2-y1);
	   ystep = (int)(time2alpha(alphay)*fupdate);	
       }

       if (stepmask & ZMASK && mask & ZMASK) {
	   zz+=res*zdir;
           alphaz = (zz-z1)/(z2-z1);
	   zstep = (int)(time2alpha(alphaz)*fupdate);	
       }

       if (stepmask & XMASK) {
	   minval = xstep;
	   mask = XMASK;
       } else if (stepmask & YMASK) {
	   minval = ystep;
	   mask = YMASK;
       } else if (stepmask & ZMASK) {
	   minval = zstep;
	   mask = ZMASK;
       }

       if (stepmask & YMASK) {
	   if (ystep < minval) {
	       minval = ystep;
	       mask = YMASK;
	   } else if (ystep == minval) {
	       mask |= YMASK;
	   }
       }

       if (stepmask & ZMASK) {
	   if (zstep < minval) {
	       minval = zstep;
	       mask = ZMASK;
	   } else if (zstep == minval) {
	       mask |= ZMASK;
	   }
       }

       // printf("%d %d, %.2x ", minval-minval2, minval, mask);
       // printf("%d %d %d %g %g %g\n", xstep, ystep, zstep, xx, yy, zz);

       printf("DEL %d\n", minval-minval2-1);
       printf("STP 0x%0.2x\n", mask);

       minval2 = minval;
    }

    return (time2alpha(1.0));
}
