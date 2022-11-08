#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <limits.h>
#include <errno.h>

#define XMASK 1
#define YMASK 2
#define ZMASK 4
#define WMASK 8

#define BAUD    B230400       /* baudrate */
//#define BAUD  B115200       /* baudrate */
//#define BAUD    B9600         /* baudrate */
//#define BAUD    B57600        /* baudrate */
//#define BAUD    B19200        /* baudrate */

#define _BSD_SOURCE 1
#define TIMEOUT 1

#define MODEM "/dev/ttyUSB0"

int fd;
FILE *pfd;
double res = .0001;

#define FIRQ 19531.0

#define FSTART 10
#define FSTOP  6000

#define TRAMP 0.15
#define TSTOP 1000.0

int coded=0;

void myputchar(char c) {
    extern FILE *pfd;
    extern int errno;
    if (fputc(c, pfd) == EOF) {
       printf("can't write to %s: %s\n", MODEM, strerror(errno));
    }
}

main() {
   int i, k;
   int c;

   double t;
   double theta=0.0;
   double val2 = 0.0;
   double val = 0.0;
   int delay=0;
   double dtheta;
   int xdir, ydir, zdir, wdir;
   int xloc, yloc, zloc, wloc;

   init_drip(MODEM);

   xdir = ydir = zdir = wdir = 0;
   xloc = yloc = zloc = wloc = 0;

   while((c=getchar()) != EOF) {

       if ((c&0xf0) == 0x80) {		// dir
	   xdir = ydir = zdir = wdir = -1;
           if (c&XMASK) xdir=1;
           if (c&YMASK) ydir=1;
           if (c&ZMASK) zdir=1;
           if (c&WMASK) wdir=1;
       } else if ((c&0xf0) == 0x90) {	// step
           if (c&XMASK) xloc+=xdir;
           if (c&YMASK) yloc+=ydir;
           if (c&ZMASK) zloc+=zdir;
           if (c&WMASK) wloc+=wdir;
	   fprintf(stderr,"  [X: %#8.4f] [Y: %#8.4f] [Z: %#8.4f] [W: %#8.4f]\r", 
	       xloc*res, yloc*res, zloc*res, wloc*res);
	   fflush(stderr);
       }

      myputchar(c);
   }
   fprintf(stderr,"\n");
}


struct termios oldtio, newtio;

int set_timeout(int decisecs) {
    extern struct termios oldtio, newtio;
    extern int fd;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUD|CS8|CREAD|CRTSCTS;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0; /* non-canonical, no echo, ... */

    newtio.c_cc[VTIME]=decisecs;  /* inter-char timer decisecs */
    newtio.c_cc[VMIN]=0;          /* block till n chars read */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
}

int init_drip(char *tty) {

    extern FILE *pfd;
    extern int fd;
    extern struct termios oldtio, newtio;

    int d;

    fd = open(tty, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror(MODEM);
        exit(-1);
    }

    tcgetattr(fd, &oldtio);     /* save current settings */

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUD|CS8|CREAD|CRTSCTS;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0; /* non-canonical, no echo, ... */

    newtio.c_cc[VTIME]=TIMEOUT; /* inter-char timer decisecs */
    newtio.c_cc[VMIN]=0;        /* block till n chars read */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    // must be opened "a+" instead of "r+" to avoid
    // "illegal seek error"...

    pfd = fdopen(fd, "a+");

    if (pfd == NULL) {
      printf("can't open %s!\n", tty);
      exit (1);
    }

    fflush(pfd);
    return(fd);
}
