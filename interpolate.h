extern void setseg(
    double lseg, double vvs, double vve, 
    double vvmax, double aamax, double rres, double f); 

extern double timeatl(double l);

extern double min(double x, double y);

double interpolate(double x1, double y1, double z1,
                   double x2, double y2, double z2, 
		   double ttotal, double ltotal);
