extern void setseg(
    double lseg, double vvs, double vve, 
    double vvmax, double aamax, double rres, double f); 

extern double timeatl(double l);

extern double min(double a, double b);

double interpolate(double x1, double y1, double z1, double w1,
                   double x2, double y2, double z2, double w2,
		   double ttotal, double ltotal);

extern int xloc, yloc, zloc, wloc;	// actual step counts
