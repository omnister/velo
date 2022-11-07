#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <strings.h>

// DIR     (7:0) '1000' wzyx  ; direction (0=ccw, 1=cw)
// STEP    (7:0) '1001' wzyx  ; step (1=step, 0=idle)

struct termios oldtermios;

#define BAUD    B230400       /* baudrate */
#define _BSD_SOURCE 1
#define TIMEOUT 1
#define MODEM "/dev/ttyUSB0"

FILE *init_drip(char *tty);

int ttyraw(int fd)
{
    /* Set terminal mode as follows:
       Noncanonical mode - turn off ICANON.
       Turn off signal-generation (ISIG)
       including BREAK character (BRKINT).
       Turn off any possible preprocessing of input (IEXTEN).
       Turn ECHO mode off.
       Disable CR-to-NL mapping on input.
       Disable input parity detection (INPCK).
       Disable stripping of eighth bit on input (ISTRIP).
       Disable flow control (IXON).
       Use eight bit characters (CS8).
       Disable parity checking (PARENB).
       Disable any implementation-dependent output processing
       (OPOST).
       One byte at a time input (MIN=1, TIME=0).
     */
    struct termios newtermios;
    if (tcgetattr(fd, &oldtermios) < 0)
	return (-1);
    newtermios = oldtermios;

    newtermios.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    /* OK, why IEXTEN? If IEXTEN is on, the DISCARD character
       is recognized and is not passed to the process. This 
       character causes output to be suspended until another
       DISCARD is received. The DSUSP character for job control,
       the LNEXT character that removes any special meaning of
       the following character, the REPRINT character, and some
       others are also in this category.
     */

    newtermios.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /* If an input character arrives with the wrong parity, then
       INPCK is checked.  If this flag is set, then IGNPAR is checked to
       see if input bytes with parity errors should be ignored.  If it
       shouldn't be ignored, then PARMRK determines what character
       sequence the process will actually see. 

       When we turn off IXON, the start and stop characters can
       be read.
     */

    newtermios.c_cflag &= ~(CSIZE | PARENB);
    /* CSIZE is a mask that determines the number of bits per
       byte.  PARENB enables parity checking on input and parity
       generation
       on output.
     */

    newtermios.c_cflag |= CS8;
    /* Set 8 bits per character. */

    newtermios.c_oflag &= ~(OPOST);
    /* This includes things like expanding tabs to spaces. */

    newtermios.c_cc[VMIN] = 1;
    newtermios.c_cc[VTIME] = 0;

    /* You tell me why TCSAFLUSH. */
    if (tcsetattr(fd, TCSAFLUSH, &newtermios) < 0)
	return (-1);
    return (0);
}


int ttyreset(int fd)
{
    if (tcsetattr(fd, TCSAFLUSH, &oldtermios) < 0)
	return (-1);

    return (0);
}

void sigcatch(int sig)
{
    ttyreset(0);
    exit(0);
}

#define STEPRATE 5000 // step rate in HZ
int count=150;

int pulse(FILE *dev, int a, int b, int count) {
    int i;
    for (i=0; i<count; i++) {
	fprintf(dev, "%c%c", a, b);
	fflush(dev);
	usleep(1000000/STEPRATE);
    }
    return(count);
}


void main()
{
    int i;
    char c;
    int x,y,z;
    int debug=0;
    int cc;
    FILE *USBDEV;

    x=y=z=0;

    USBDEV = init_drip(MODEM);

    if ((size_t) signal(SIGINT, sigcatch) < 0) {
	perror("signal");
	exit(1);
    }
    if ((size_t) signal(SIGQUIT, sigcatch) < 0) {
	perror("signal");
	exit(1);
    }
    if ((size_t) signal(SIGTERM, sigcatch) < 0) {
	perror("signal");
	exit(1);
    }

    /* Set raw mode on stdin. */
    if (ttyraw(0) < 0) {
	fprintf(stderr, "Can't go to raw mode.\n");
	exit(1);
    }

    fprintf(stderr, "cnc jog program: use backspace to quit\n\r");
    fprintf(stderr, "k, j, h, l, v, r\n\r");

    int state=0;
    while ((i = read(0, &c, 1)) == 1) {
	switch (state) {
	   case 0:
	      if (c == 27) {	// <ESC>
	         state=1;
	      } else {
		 if (c == 'k') {
		     y+=pulse(USBDEV, 0x8f, 0x92, count);
		 } else if (c == 'j') {
		     y-=pulse(USBDEV, 0x80, 0x92, count);
		 } else if (c == 'h') {
		     x-=pulse(USBDEV, 0x8f, 0x91, count);
		 } else if (c == 'l') {
		     x+=pulse(USBDEV, 0x80, 0x91, count);
		 } else if (c == 'v') {
		     z-=pulse(USBDEV, 0x80, 0x94, count);
		 } else if (c == 'r') {
		     z+=pulse(USBDEV, 0x8f, 0x94, count);
		 } 
	         state=0;
	      }
	      break;
	   case 1:
	      if (c == 91) {	// [
	         state=2;
	      } else {
	         state = 0;
	      }
	      break;
	   case 2:

// DIR     (7:0) '1000' wzyx  ; direction (0=ccw, 1=cw)
// STEP    (7:0) '1001' wzyx  ; step (1=step, 0=idle)

	      if (c == 53) {	// 5
		 z+=pulse(USBDEV, 0x8f, 0x94, count);
	         if (debug)  fprintf(stderr,"got pageup\n\r");
	      } else if (c == 54) {	// 6
		 z-=pulse(USBDEV, 0x80, 0x94, count);
	         if (debug) fprintf(stderr,"got pagedown\n\r");
	      } else if ((c &= 255) == 65) {	// A
		 y+=pulse(USBDEV, 0x8f, 0x92, count);
	         if (debug) fprintf(stderr,"got uparrow\n\r");
	      } else if ((c &= 255) == 66) {	// B
		 y-=pulse(USBDEV, 0x80, 0x92, count);
	         if (debug) fprintf(stderr,"got downarrow\n\r");
	      } else if ((c &= 255) == 67) {	// C
		 x+=pulse(USBDEV, 0x80, 0x91, count);
	         if (debug) fprintf(stderr,"got rightarrow\n\r");
	      } else if ((c &= 255) == 68) {	// D
		 x-=pulse(USBDEV, 0x8f, 0x91, count);
	         if (debug) fprintf(stderr,"got leftarrow\n\r");
	      }
	      fflush(USBDEV);
	      state=0;
	      break;
	    default:
	      state=0;
	      break;
	}
	if ((c &= 255) == 0177)	/* ASCII DELETE */
	    break;
	if ((c & 255) == 'q')	/* quit */
	    break;
	fprintf(stderr,"(x:%4d) (y:%4d) (z:%4d)\r", x, y, z);
	fflush(stdout);
    }

    if (ttyreset(0) < 0) {
	fprintf(stderr, "Cannot reset terminal!\n");
	exit(-1);
    }

    if (i < 0) {
	fprintf(stderr, "Read error.\n");
	exit(-1);
    }

    exit(0);
}

FILE *pfd;
int fd;
struct termios oldtio, newtio;

FILE *init_drip(char *tty) {

    extern FILE *pfd;
    extern int fd;
    extern struct termios oldtio, newtio;

    int d;

    fd = open(tty, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror(tty);
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
    return(pfd);
}

/*
27 91 53 126 pu	"<esc>[5~"
27 91 54 126 pd	"<esc>[6~"
27 91 65 ^		"<esc>[A"
27 91 66 v		"<esc>[B"
27 91 67 >		"<esc>[C"
27 91 68 <		"<esc>[D"
*/
