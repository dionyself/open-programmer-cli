CC = gcc
CFLAGS = -Wall -Os -s #size
#CFLAGS = -w -O3 -s
#CFLAGS = -w -g		#debug
OBJECTS = 	op.o \
			progP12.o \
			progP16.o \
			progP18.o \
			progP24.o \
			progEEPROM.o \
			progAVR.o \
			fileIO.o \
			deviceRW.o \
			I2CSPI.o \
			strings.o

all: op

op : $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o op

%.o : %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f op $(OBJECTS)


.PHONY: clean

prefix	:= /usr/local
    
install: op
	test -d $(prefix) || mkdir $(prefix)
	test -d $(prefix)/bin || mkdir $(prefix)/bin
	install -m 0755 op $(prefix)/bin;
	
.PHONY: install

package:
	tar -cvzf op.tar.gz ../op/*.c ../op/*.h ../op/gpl-2.0.txt ../op/Makefile ../op/readme ../op/leggimi ../op/utils/*.c

