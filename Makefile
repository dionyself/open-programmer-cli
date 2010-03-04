#CC=gcc 
CFLAGS=-w

op: op.o progP12.o progP16.o progP18.o progP24.o progEEPROM.o progAVR.o

op.o: op.c

progP12.o: progP12.c

progP16.o: progP16.c

progP18.o: progP18.c

progP24.o: progP24.c

progEEPROM.o: progEEPROM.c

progAVR.o: progAVR.c

clean:
	rm -f op op.o progP12.o progP16.o progP18.o progP24.o progEEPROM.o progAVR.o

.PHONY: clean

prefix	:= /usr/local
    
install: op
	test -d $(prefix) || mkdir $(prefix)
	test -d $(prefix)/bin || mkdir $(prefix)/bin
	install -m 0755 op $(prefix)/bin;
	
.PHONY: install
