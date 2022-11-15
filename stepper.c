#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stepper.h"

void fatal(char *s) {
    fprintf(stderr, "%s\n", s);
    exit(1);
}
// configure a STEPPARM structure based on the following parameters
//
//	L 	; length of stroke
//	vs	; starting speed (non-zero if chaining, usually zero)
// 	ve	; ending speed (non-zero if chaining, usually zero)
//	vmax	; maximum allowed speed
// 	amax	; maximum acceleration
//	res	; size of a step
//

STEPPARM *stepper_set_parms(float l, float vs, float ve, float amax, float vmax, float res) {
    STEPPARM *s=NULL;
    float vm;

    if ((s=(STEPPARM *)malloc(sizeof(STEPPARM))) == NULL) {
	return NULL;
    }

    s->l = l;
    s->vs = vs;
    s->ve = ve;
    s->vmax = vmax;
    s->amax = amax;
    s->res = res;
    s->vm = vm = fminf(sqrtf((vs*vs+ve*ve+2.0*amax*l)/2.0),vmax);
    s->s1=(vm*vm-vs*vs)/(2.0*amax); if (fabsf(s->s1)<res) s->s1 = 0.0;
    s->s3=(vm*vm-ve*ve)/(2.0*amax); if (fabsf(s->s3)<res) s->s3 = 0.0;
    s->s2=(l - s->s1 - s->s3); if (fabsf(s->s2)<res) s->s2 = 0;
    s->ts1 = (sqrtf(vs*vs+2.0*amax* s->s1)-vs)/amax;  	// time to reach s1
    s->ts2 = s->ts1 + (s->s2)/vm;                   	// time to reach s2

    return s;
}

void setpen(int pen) {
    // printf("pen %d\n", pen);
}

// return the time to get to distance L

float time_at_l(STEPPARM *s, float l) {

    float tt=0.0;

    if (s==NULL) fatal("null stepparm in time_at_l()");

    if (l < s->s1) {             // accellerating
       tt = (sqrtf(s->vs*s->vs+2.0*s->amax*l)-s->vs)/s->amax;
       setpen(2);
    } else if (l < s->s1+s->s2) {   // cruising
       tt = s->ts1 + (l-s->s1)/s->vm;
       setpen(3);
    } else {                  // decellerating
       tt = s->ts2 + (s->vm - sqrtf(fabsf(s->vm*s->vm - 2.0*s->amax*(l-(s->s2+s->s1)))))/s->amax;
       setpen(4);
    }
    return(tt);
}

int test_main(void) {
    float ll;
    STEPPARM *s=NULL;

    float l=200.0;
    float res=1.0;

    float t0=0.0;
    float t1=0.0;

    // stepper_set_parms(float l, float vs, float ve, float vmax, float amax, float res) {

    if ((s=stepper_set_parms(l, 0.0, 0.0, 1.0, 10.0, res)) == NULL) {
	fatal("STEPPARM malloc error");
    }

    while(1) {
	for (ll=0; ll<=l; ll+=res) {
	      t1=t0; t0 = time_at_l(s, ll);
	      printf("%f %f %f\n", ll, t0-t1, t0);
	}
	for (ll=l; ll>=0; ll-=res) {
	      t1=t0; t0 = time_at_l(s, ll);
	      printf("%f %f %f\n", ll, t0-t1, t0);
	}
    }
}
