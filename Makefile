CC = gcc
#CFLAGS = -w -O3 -s
CFLAGS = -w
OBJECTS = op.o progP12.o progP16.o progP18.o progP24.o progEEPROM.o progAVR.o fileIO.o deviceRW.o I2CSPI.o

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
