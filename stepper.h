typedef struct stepparm {
    float l;
    float vs;	// starting speed (non-zero if chaining, usually zero)
    float ve;   // ending speed (non-zero if chaining, usually zero)
    float vmax; // maximum speed
    float amax;  // maximum acceleration
    float res;	// size of a step
    float s1;	// distance from start to reach vmax
    float s2;	// distance for cruising at vmax
    float s3;	// distance from end of cruising to stop point
    float ts1;	// time to reach s1
    float ts2;	// time to reach s2
    float vm;	
} STEPPARM;

STEPPARM *stepper_set_parms(float l, float vs, float ve, float amax, float vmax, float res);
float time_at_l(STEPPARM *s, float l);
void fatal(char *msg);
