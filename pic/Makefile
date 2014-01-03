# Example1

PIC		= PIC18F4525

# system programs
RM              = /bin/rm -f
ARC             = tar -cvf
ZIP             = gzip

#ccsc
CC		= /usr/local/picc/ccsc +fh

# gpasm programs
ASM             = gpasm
CODVIEWER       = gpvc -d 

# project data
PROJECT         = servo4
VERSION         = 0.0.1
C_FILE		= $(PROJECT).c
BIGC_FILE	= $(PROJECT).C
ERRFILE		= $(PROJECT).err
ASM_FILE        = $(PROJECT).asm 
FILE_LIST       = $(PROJECT).asm 

# derived variables
ARCHIVE         = $(PROJECT)_$(VERSION).tar
HEX_FILE        = $(PROJECT).hex
COD_FILE        = $(PROJECT).cod

all:	$(HEX_FILE)

$(HEX_FILE): $(C_FILE)
	cp $(C_FILE) $(BIGC_FILE)
	-$(CC) +fh $(BIGC_FILE)
	grep " 0 Errors" $(ERRFILE) >/dev/null 2>&1
	$(RM) $(BIGC_FILE) -- -d-debug.txt

tar:	$(TARS)
	(  d=`date +%F`;\
	   q=`pwd`;\
	   p=`basename $$q`;\
	   rm -rf $$p$$d;\
	   mkdir $$p$$d;\
	   cp -rp $(TARS) $$p$$d;\
	   tar czvf - $$p$$d >$$p$$d.tar.gz;\
	)

clean:
	$(RM) *.err *.esym *.sym  *.hex *.lst *.cod $(PROJECT)*.gz ccsc_log.txt -- -d-debug.txt

prog: $(HEX_FILE)
	picp -c /dev/ttyUSB1 $(PIC) -ef -bp -wp $(HEX_FILE)

progp: $(HEX_FILE)
	pk2cmd -p$(PIC) -m -B/usr/share/pk2 -f$(HEX_FILE)

verify: $(HEX_FILE)
	pk2cmd -p$(PIC) -y -B/usr/share/pk2 -f$(HEX_FILE)
