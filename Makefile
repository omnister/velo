
all: velo jog feed rawstep

velo: velo.c interpolate.c
	cc velo.c interpolate.c -o velo -lm

jog: jog.c
	cc jog.c -o jog -lm

feed: feed.c
	cc feed.c -o feed -lm

rawstep: rawstep.c stepper.c stepper.h
	cc rawstep.c stepper.c -o rawstep -lm

