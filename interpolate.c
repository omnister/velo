#include <math.h>

static double vs, ve;		// start and end velocity
static double vm;		// peak velocity value for segment
static double lseg;		// length of this segment
static double s1,s2,s3;		// distances of rampup,constant,rampdown
static double ts1, ts2;		// time to reach s1,s2
static double vmax;
static double amax;
static double res;

// set globals for further computation on this segment

void setseg(double x1, double y1, double z1, 
    double x2, double y2, double z2, 
    double vvs, double vve, double vvmax, double aamax, double rres) {

    vs = vvs; ve = vve;
    vmax=vvmax;
    amax=aamax;
    res=rres;

    lseg = sqrt(pow((x2-x1),2.0) + pow((y2-y1),2.0) + pow((z2-z1),2.0));

    // peak velocity for this segment
    vm = min(vmax, sqrt((pow(vs,2.0) + pow(ve,2.0) + 2.0*amax*lseg)/2.0));

    // ramp up distance (s1), constant v (s2) and ramp down (s3)
    s1 = (vm*vm-vs*vs)/(2.0*amax); if (fabs(s1) < res/100.0) s1 = 0.0;
    s3 = (vm*vm-ve*ve)/(2.0*amax); if (fabs(s3) < res/100.0) s3 = 0.0;
    s2 = lseg-s1-s3; if (fabs(s2) < res/100.0) s2 = 0.0;

    // time to reach s1, s2
    ts1 = (sqrt(vs*vs+2.0*amax*s1)-vs)/amax;
    ts2 = ts1 + s2*vm;
}

// return the time it takes to get to distance l in this segment
double timeatl(double l) {
      double tt;
      if (l < s1) {             // accellerating
         tt = (sqrt(vs*vs+2.0*amax*l)-vs)/amax;
      } else if (l < s1+s2) {   // cruising
         tt = ts1 + (l-s1)*vm;
      } else {                  // decellerating
         tt = ts2 + (vm - sqrt(abs(vm*vm - 2.0*amax*(l-(s2+s1)))))/amax;
      }
      return(tt);
}
