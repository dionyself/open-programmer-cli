/*
 * op.c - control program for the open programmer
 * Copyright (C) 2009-2010 Alberto Maccioni
 * for detailed info see:
 * http://openprog.altervista.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111 USA
 * or see <http://www.gnu.org/licenses/>
 */


#if !defined _WIN32 && !defined __CYGWIN__
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#else
#include <windows.h>
#include <setupapi.h>
#include <ddk/hidusage.h>
#include <ddk/hidpi.h>
#endif

#include <sys/timeb.h>
#include <wchar.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include "strings.h"
#include "instructions.h"

#define COL 16
#define VERSION "0.7.4"
#define G (12.0/34*1024/5)		//=72,2823529412
#define  LOCK	1
#define  FUSE	2
#define  FUSE_H 4
#define  FUSE_X	8
#define  CAL	16
#define  SLOW	256

#if !defined _WIN32 && !defined __CYGWIN__
    #define write() ioctl(fd, HIDIOCSUSAGES, &ref_multi_u); ioctl(fd,HIDIOCSREPORT, &rep_info_u);
    #define read() ioctl(fd, HIDIOCGUSAGES, &ref_multi_i); ioctl(fd,HIDIOCGREPORT, &rep_info_i);
    #define bufferU ref_multi_u.values
    #define bufferI ref_multi_i.values

#else
	#define write()	Result=WriteFile(WriteHandle,bufferU,DIMBUF,&BytesWritten,NULL);
	#define read()	Result = ReadFile(ReadHandle,bufferI,DIMBUF,&NumberOfBytesRead,(LPOVERLAPPED) &HIDOverlapped);\
					Result = WaitForSingleObject(hEventObject,10);\
					ResetEvent(hEventObject);\
					if(Result!=WAIT_OBJECT_0){\
						printf(strings[S_comTimeout]);	/*"Timeout comunicazione\r\n"*/\
					}
#endif

typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char BYTE;
#define CloseLogFile() if(logfile)fclose(logfile);

#if !defined _WIN32 && !defined __CYGWIN__
DWORD GetTickCount();
#endif
void msDelay(double delay);
void Save(char* dev,char* savefile);
int Load(char* dev,char* loadfile);
void SaveEE(char* dev,char* savefile);
void LoadEE(char* dev,char* loadfile);
unsigned int htoi(const char *hex, int length);
void Read12F5xx(int dim,int dim2);
void Read16Fxxx(int dim,int dim2,int dim3,int vdd);
void Read18Fx(int dim,int dim2,int options);
void Read25xx(int dim);
void Write12F5xx(int dim,int OscAddr);
void Write12F6xx(int dim,int dim2);
void Write16F8x(int dim,int dim2);
void Write16F87x(int dim,int dim2);
void Write16F87xA(int dim,int dim2,int seq);
void Write16F62x(int dim,int dim2);
void Write12F62x(int dim,int dim2);
void Write12F61x(int dim);
void Write16F88x(int dim,int dim2);
void Write18Fx(int dim,int dim2,int wbuf,int eraseW1,int eraseW2,int EEalgo);
void ReadI2C(int dim,int addr);
void WriteI2C(int dim, int addr, int page, float wait);
void ReadAT(int dim,int dim2,int options);
void WriteAT(int dim,int dim2);
void WriteATmega(int dim,int dim2,int page,int options);
void Read93Sx(int dim,int na);
void Write93Sx(int dim,int na, int page, double wait);
void Write25xx(int dim,int page,float wait);
void TestHw();
int StartHVReg(double V);
void ProgID();
void DisplayEE();
int FindDevice();
void OpenLogFile();

char** strings;
int saveLog=0,programID=0,MinDly=1,load_osccal=0,load_BKosccal=0,usa_osccal=1,usa_BKosccal=0;
int load_calibword=0,max_err=200;
int lock=0x100,fuse=0x100,fuse_h=0x100,fuse_x=0x100;
int FWVersion=0;
FILE* logfile=0;
char LogFileName[256]="";
char loadfile[256]="",savefile[256]="";
char loadfileEE[256]="",savefileEE[256]="";
WORD *dati_hex;
int size=0,sizeEE=0,sizeCONFIG=0;
unsigned char *memCODE,*memEE,memID[8],memCONFIG[34];
int vid=0x04D8,pid=0x0100,info=0;
#if !defined _WIN32 && !defined __CYGWIN__
int fd = -1;
struct hiddev_report_info rep_info_i,rep_info_u;
struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
int DIMBUF=64;
char path[256]="/dev/usb/hiddev0";
#else
unsigned char bufferU[128],bufferI[128];
DWORD NumberOfBytesRead,BytesWritten;
ULONG Result;
HANDLE WriteHandle,ReadHandle;
OVERLAPPED HIDOverlapped;
HANDLE hEventObject;
int DIMBUF=65;
#endif


int main (int argc, char **argv) {

	int v=0,ee=0,r=0,ver=0,c=0,lista=0,i2c=0,spi_mode=0,i,j,testhw=0;
	char dev[64]="null";
	unsigned char tmpbuf[128];
	opterr = 0;
	int option_index = 0;
	#include "strings.c"
	if(getenv("LANG")&&strstr(getenv("LANG"),"it")!=0) strings=strings_it;
	else strings=strings_en;
	strncpy(LogFileName,strings[S_LogFile],sizeof(LogFileName));
	if(argc==1){
		printf(strings[L_HELP]);
		exit(1);
	}
	struct option long_options[] =
	{
		{"BKosccal",      no_argument,  &load_BKosccal, 1},
		{"calib",         no_argument, &load_calibword, 1},
		{"d",             required_argument,       0, 'd'},
		{"device",        required_argument,       0, 'd'},
		{"D",             required_argument,       0, 'D'},
		{"delay",         required_argument,       0, 'D'},
		{"ee",            no_argument,             &ee, 1},
		{"err",           required_argument,       0, 'e'},
		{"fuse",          required_argument,       0, 'f'},
		{"fuseh",         required_argument,       0, 'F'},
		{"fusex",         required_argument,       0, 'X'},
		{"h",             no_argument,             0, 'h'},
		{"help",          no_argument,             0, 'h'},
		{"HWtest",        no_argument,         &testhw, 1},
		{"info",          no_argument,           &info, 1},
		{"i",             no_argument,           &info, 1},
		{"i2c_r",         no_argument,            &i2c, 1},
		{"i2c_r2",        no_argument,            &i2c, 2},
		{"i2c_w",         no_argument,            &i2c, 3},
		{"i2c_w2",        no_argument,            &i2c, 4},
		{"id",            no_argument,      &programID, 1},
		{"l",             optional_argument,       0, 'l'}, //bug di getopt: ci vuole -l=valore
		{"log",           optional_argument,       0, 'l'},
		{"lock",          required_argument,       0, 'L'},
		{"mode",          required_argument,       0, 'm'},
		{"osccal",        no_argument,    &load_osccal, 1},
#if !defined _WIN32 && !defined __CYGWIN__
		{"p",             required_argument,       0, 'p'},
		{"path",          required_argument,       0, 'p'},
#endif
		{"pid",           required_argument,       0, 'P'},
		{"rep" ,          required_argument,       0, 'r'},
		{"reserved",      no_argument,              &r, 1},
		{"r",             no_argument,              &r, 1},
		{"save",          required_argument,       0, 's'},
		{"s",             required_argument,       0, 's'},
		{"saveEE",        required_argument,       0, 'S'},
		{"se",            required_argument,       0, 'S'},
		{"spi_r",         no_argument,            &i2c, 5},
		{"spi_w",         no_argument,            &i2c, 6},
		{"support",       no_argument,          &lista, 1},
		{"use_BKosccal",  no_argument,   &usa_BKosccal, 1},
		{"version",       no_argument,            &ver, 1},
		{"v",             no_argument,            &ver, 1},
		{"vid",           required_argument,       0, 'V'},
		{"w",             required_argument,       0, 'w'},
		{"write",         required_argument,       0, 'w'},
		{"we",            required_argument,       0, 'W'},
		{"writeEE",       required_argument,       0, 'W'},
		{0, 0, 0, 0}
	};
	while ((c = getopt_long_only (argc, argv, "",long_options,&option_index)) != -1)
/*	{	printf("optarg=%X\n",optarg);
		if(optarg) printf("%s\n",optarg);
		printf("c=%X %c\noption_index=%d name=%s\n",c,c,option_index,long_options[option_index].name);	}
		exit(0);*/
		switch (c)
		{
			case 'h':	//help
				printf(strings[L_HELP]);
				return 1 ;
				break;
			case 'd':	//device
				strncpy(dev,optarg,sizeof(dev)-1);
				break;
			case 'D':	//minimum delay
				MinDly = atoi(optarg);
				break;
			case 'e':	//max write errors
				max_err = atoi(optarg);
				break;
			case 'f':	//Atmel FUSE low
				i=sscanf(optarg, "%x", &fuse);
				if(i!=1||fuse<0||fuse>0xFF) fuse=0x100;
				break;
			case 'F':	//Atmel FUSE high
				i=sscanf(optarg, "%x", &fuse_h);
				if(i!=1||fuse_h<0||fuse_h>0xFF) fuse_h=0x100;
				break;
			case 'l':	//save Log
				saveLog=1;
				if(optarg) strncpy(LogFileName,optarg,sizeof(LogFileName));
				break;
			case 'L':	//Atmel LOCK
				i=sscanf(optarg, "%x", &lock);
				if(i!=1||lock<0||lock>0xFF) lock=0x100;
				break;
			case 'm':	//SPI mode
				spi_mode = atoi(optarg);
				break;
#if !defined _WIN32 && !defined __CYGWIN__
			case 'p':	//hiddev path
				strncpy(path,optarg,sizeof(path)-1);
				break;
#endif
			case 'P':	//pid
				sscanf(optarg, "%x", &pid);
				break;
			case 'r':	//USB HID report size
				DIMBUF = atoi(optarg);
				break;
			case 's':	//save
				strncpy(savefile,optarg,sizeof(savefile)-1);
				break;
			case 'S':	//save EE
				strncpy(savefileEE,optarg,sizeof(savefileEE)-1);
				break;
			case 'V':	//vid
				sscanf(optarg, "%x", &vid);
				break;
			case 'X':	//Atmel extended FUSE
				i=sscanf(optarg, "%x", &fuse_x);
				if(i!=1||fuse_x<0||fuse_x>0xFF) fuse_x=0x100;
				break;
			case 'w':	//write file
				strncpy(loadfile,optarg,sizeof(loadfile)-1);
				break;
			case 'W':	//write EE file
				strncpy(loadfileEE,optarg,sizeof(loadfileEE)-1);
				break;
			case '?':
				fprintf (stderr,strings[L_OPTERR]);		//errore opzioni
				return 1;
			default:

				break;
		}

	for(j=0,i = optind; i < argc&&i<128; i++,j++) sscanf(argv[i], "%x", &tmpbuf[j]);
	for(;j<128;j++) tmpbuf[j]=0;
	if (ver){
		printf("OP v%s\nCopyright (C) Alberto Maccioni 2009-2010\
\n	For detailed info see http://openprog.altervista.org/\
\nThis program is free software; you can redistribute it and/or modify it under \
the terms of the GNU General Public License as published by the Free Software \
Foundation; either version 2 of the License, or (at your option) any later version.\
			\n",VERSION);
		return 0;
	}
	if (lista){
		printf("%s\n\
10F200, 10F202, 10F204, 10F206, 10F220, 10F222,\n\
12F508, 12F509, 12F510, 12F519, 12F609, 12F615, 12F629, 12F635, 12F675, 12F683, \
16F505, 16F506, 16F526, 16F54, 16F610, 16F616, 16F627, 16F627A, 16F628, 16F628A, \
16F630, 16F631, 16F636, 16F639, 16F648A, 16F676, 16F677, 16F684, 16F685, 16F687, \
16F688, 16F689, 16F690, 16F716, 16F73, 16F737, 16F74, 16F747, 16F76, 16F767, 16F77, \
16F777, 16F785, 16F818, 16F819, 16F83, 16F83A, 16C83, 16C83A, 16F84, 16C84, 16F84A, \
16C84A, 16F87, 16F870, 16F871, 16F872, 16F873, 16F873A, 16F874, 16F874A, 16F876, \
16F876A, 16F877, 16F877A, 16F88, 16F882, 16F883, 16F884, 16F886, 16F887, 16F913, \
16F914, 16F916, 16F917, 16F946,\n\
16F1822, 16F1823, 16F1824, 16F1825, 16F1826, 16F1827, 16F1828, 16F1829, 16F1933, \
16F1934, 16F1936, 16F1937, 16F1938, 16F1939, 16F1946, 16F1947,\n\
18F242, 18F248, 18F252, 18F258, 18F442, 18F448, 18F452, 18F458, 18F1220, 18F1230,\
18F1320, 18F1330, 18F13K50, 18F14K50, 18F2220, 18F2221, 18F2320, 18F23K20, 18F2321, \
18F2331, 18F2410, 18F24J10, 18F24J11, 18F2420, 18F24K20, 18F2423, 18F2431, 18F2439, \
18F2450, 18F24J50, 18F2455, 18F2458, 18F2480, 18F2510, 18F25J10, 18F25J11, 18F2515, \
18F25K20, 18F2520, 18F2523, 18F2525, 18F2539, 18F2550, 18F25J50, 18F2553, 18F2580, \
18F2585, 18F2610, 18F26J11, 18F26J13, 18F2620, 18F26K20, 18F26J50, 18F26J53, 18F2680, \
18F2682, 18F2685, 18F27J13, 18F27J53, 18F4220, 18F4221, 18F4320, 18F43K20, 18F4321, \
18F4331, 18F4410, 18F44J10, 18F44J11, 18F4420, 18F44K20, 18F4423, 18F4431, 18F4439, \
18F4450, 18F44J50, 18F4455, 18F4458, 18F4480, 18F4510, 18F45J10, 18F45J11, 18F4515, \
18F4520, 18F45K20, 18F4523, 18F4525, 18F4539, 18F4550, 18F45J50, 18F4553, 18F4580, \
18F4585, 18F4610, 18F46J11, 18F46J13, 18F4620, 18F46K20, 18F46J50, 18F46J53, 18F4680, \
18F4682, 18F4685, 18F47J13, 18F47J53, 18F8722,\n\
24F04KA200, 24F04KA201, 24F08KA101, 24F08KA102, 24F16KA101, 24F16KA102, 24FJ16GA002, \
24FJ16GA004, 24FJ32GA002, 24FJ32GA004, 24FJ48GA002, 24FJ48GA004, 24FJ64GA002, 24FJ64GA004, \
24FJ64GA006, 24FJ64GA008, 24FJ64GA010, 24FJ96GA006, 24FJ96GA008, 24FJ96GA010, 24FJ128GA006, \
24FJ128GA008, 24FJ128GA010, 24FJ32GA102, 24FJ32GA104, 24FJ32GB002, 24FJ32GB004, 24FJ64GA102, \
24FJ64GA104, 24FJ64GB002, 24FJ64GB004, 24FJ64GB106, 24FJ64GB108, 24FJ64GB110, 24FJ128GA106, \
24FJ128GB106, 24FJ128GA108, 24FJ128GB108, 24FJ128GA110, 24FJ128GB110, 24FJ192GA106, \
24FJ192GB106, 24FJ192GA108, 24FJ192GB108, 24FJ192GA110, 24FJ192GB110, 24FJ256GA106, \
24FJ256GB106, 24FJ256GA108, 24FJ256GB108, 24FJ256GA110, 24FJ256GB110, 24HJ12GP201, \
24HJ12GP202, 24HJ16GP304, 24HJ32GP202, 24HJ32GP204, 24HJ32GP302, 24HJ32GP304, 24HJ64GP202, \
24HJ64GP204, 24HJ64GP206, 24HJ64GP210, 24HJ64GP502, 24HJ64GP504, 24HJ64GP506, 24HJ64GP510, \
24HJ128GP202, 24HJ128GP204, 24HJ128GP206, 24HJ128GP210, 24HJ128GP306, 24HJ128GP310, \
24HJ128GP502, 24HJ128GP504, 24HJ128GP506, 24HJ128GP510, 24HJ256GP206, 24HJ256GP210, \
24HJ256GP610,\n\
30F2010, 30F2011, 30F2012, 30F3010, 30F3011, 30F3012, 30F3013, 30F3014, 30F4011, 30F4012, \
30F4013, 30F5011, 30F5013, 30F5015, 30F5016, 30F6010, 30F6011, 30F6012, 30F6013, 30F6014, \
30F6015,\n\
33FJ06GS101, 33FJ06GS102, 33FJ06GS202, 33FJ12GP201, 33FJ12GP202, 33FJ12MC201, 33FJ12MC202, \
33FJ16GP304, 33FJ16GS402, 33FJ16GS404, 33FJ16GS502, 33FJ16GS504, 33FJ16MC304, 33FJ32GP202, \
33FJ32GP204, 33FJ32GP302, 33FJ32GP304, 33FJ32GS406, 33FJ32GS606, 33FJ32GS608, 33FJ32GS610, \
33FJ32MC202, 33FJ32MC204, 33FJ32MC302, 33FJ32MC304, 33FJ64GP202, 33FJ64GP204, 33FJ64GP206, \
33FJ64GP306, 33FJ64GP310, 33FJ64GP706, 33FJ64GP708, 33FJ64GP710, 33FJ64GP802, 33FJ64GP804, \
33FJ64GS406, 33FJ64GS606, 33FJ64GS608, 33FJ64GS610, 33FJ64MC202, 33FJ64MC204, 33FJ64MC506, \
33FJ64MC508, 33FJ64MC510, 33FJ64MC706, 33FJ64MC710, 33FJ64MC802, 33FJ64MC804, 33FJ128GP202, \
33FJ128GP204, 33FJ128GP206, 33FJ128GP306, 33FJ128GP310, 33FJ128GP706, 33FJ128GP708, \
33FJ128GP710, 33FJ128GP802, 33FJ128GP804, 33FJ128MC202, 33FJ128MC204, 33FJ128MC506, \
33FJ128MC510, 33FJ128MC706, 33FJ128MC708, 33FJ128MC710, 33FJ128MC802, 33FJ128MC804, \
33FJ256GP506, 33FJ256GP510, 33FJ256GP710, 33FJ256MC510, 33FJ256MC710,\n\
2400, 2401, 2402, 2404, 2408, 2416, 2432, 2464, 24128, 24256, 24512, 241024, 241025,\n\
25010, 25020, 25040, 25080, 25160, 25320, 25640, 25128, 25256, 25512, 251024,\n\
93S46, 93x46, 93x46A, 93S56, 93x56, 93x56A, 93S66, 93x66, 93x66A, 93x76, 93x76A, 93x86, 93x86A,\n\
AT90S1200, AT90S2313, AT90S8515, AT90S8535, ATmega8, ATmega8A, ATmega8515, ATmega8535,\
ATmega16, ATmega16A, ATmega32, ATmega32A, ATmega64, ATmega64A, ATtiny2313\n\
\n%s\
\n12C508, 12C508A, 12C509, 12C509A, 12C671, 12C672, 12CE673, 12CE674\n"\
		,strings[L_DEV_RW],strings[L_DEV_RO]);
		return 0;
	}
	if(FindDevice()<0) exit(1);
#if !defined _WIN32 && !defined __CYGWIN__
	if(info){
		struct hiddev_devinfo device_info;
		ioctl(fd, HIDIOCGDEVINFO, &device_info);
		printf(strings[L_INFO1],device_info.vendor, device_info.product, device_info.version);
		printf(strings[L_INFO2],device_info.busnum, device_info.devnum, device_info.ifnum);
		char name[256];
		strcpy(name,strings[L_UNKNOWN]);//"Unknown"
		if(ioctl(fd, HIDIOCGNAME(sizeof(name)), name) < 0) perror("evdev ioctl");
		printf(strings[L_NAME], path, name);//"The device on %s says its name is %s\n"
		return 0;
	}
#endif

	DWORD t0,t;
	t=t0=GetTickCount();
	ProgID();
	if(!strncmp(dev,"16F1",4));
	else if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)||testhw) StartHVReg(13);
	else StartHVReg(-1);
	if(testhw){			//test hardware
		TestHw();
		return 0;
	}

#define CS 8
#define HLD 16
	if(i2c){							//I2C, SPI
		j=1;
		bufferU[0]=0;
		bufferU[j++]=EN_VPP_VCC;	//VDD
		bufferU[j++]=0x1;
		if(i2c<5){					//I2C mode
			bufferU[j++]=I2C_INIT;
			bufferU[j++]=0;
		}
		else{						//SPI mode
			if(spi_mode==10) spi_mode=0x10;
			else if(spi_mode==11) spi_mode=0x11;
			else if(spi_mode>1||spi_mode<0) spi_mode=0;
			bufferU[j++]=EXT_PORT;	//CS=1
			bufferU[j++]=CS;
			bufferU[j++]=0;
			bufferU[j++]=EXT_PORT;	//CS=0
			bufferU[j++]=0;
			bufferU[j++]=0;
			bufferU[j++]=SPI_INIT;
			bufferU[j++]=spi_mode;	//00 01 10 11
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		j=1;
		i=0;
		if(i2c==1){				//I2C read
			bufferU[j++]=I2C_READ;
			bufferU[j++]=tmpbuf[0]>(DIMBUF-4)?DIMBUF-4:tmpbuf[0];
			bufferU[j++]=tmpbuf[1];		//Control byte
			bufferU[j++]=tmpbuf[2];		//Address;
		}
		else if(i2c==2){				//I2C read 2
			bufferU[j++]=I2C_READ2;
			bufferU[j++]=tmpbuf[0]>(DIMBUF-4)?DIMBUF-4:tmpbuf[0];
			bufferU[j++]=tmpbuf[1];		//Control byte
			bufferU[j++]=tmpbuf[2];		//Address H;
			bufferU[j++]=tmpbuf[3];		//Address L;
		}
		else if(i2c==3||i2c==4){				//I2C write
			bufferU[j++]=I2C_WRITE;
			if(i2c==4) tmpbuf[0]++;
			bufferU[j++]=tmpbuf[0]>(DIMBUF-5)?DIMBUF-5:tmpbuf[0];
			bufferU[j++]=tmpbuf[1];		//Control byte
			bufferU[j++]=tmpbuf[2];		//Address
			if(i2c==4){
				bufferU[j++]=tmpbuf[3];		//Address L
				for(i=0;i<bufferU[2];i++) bufferU[j++]=tmpbuf[i+4];
			}
			else for(i=0;i<bufferU[2];i++) bufferU[j++]=tmpbuf[i+3];
		}
		else if(i2c==5){				//SPI read
			bufferU[j++]=SPI_READ;
			bufferU[j++]=tmpbuf[0]>(DIMBUF-4)?DIMBUF-4:tmpbuf[0];
			bufferU[j++]=EXT_PORT;		//CS=1
			bufferU[j++]=CS;
			bufferU[j++]=0;
		}
		else if(i2c==6){				//SPI write
			bufferU[j++]=SPI_WRITE;
			bufferU[j++]=tmpbuf[0]>(DIMBUF-5)?DIMBUF-5:tmpbuf[0];
			for(i=0;i<bufferU[2];i++) bufferU[j++]=tmpbuf[i+1];
			bufferU[j++]=EXT_PORT;		//CS=1
			bufferU[j++]=CS;
			bufferU[j++]=0;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(8);
		read();
		if(bufferI[1]==I2C_READ&&bufferI[2]<0xFA){
			printf("\n");
			printf("> %2X %2X\n< ",bufferU[3],bufferU[4]);
			for(i=3;i<bufferI[2]+3&&i<DIMBUF;i++) printf("%2X ",bufferI[i]);
			printf("\n");
		}
		else if(bufferI[1]==I2C_READ2&&bufferI[2]<0xFA){
			printf("\n");
			printf("> %2X %2X %2X\n< ",bufferU[3],bufferU[4],bufferU[5]);
			for(i=3;i<bufferI[2]+3&&i<DIMBUF;i++) printf("%2X ",bufferI[i]);
			printf("\n");
		}
		else if(bufferI[1]==I2C_WRITE&&bufferI[2]<0xFA){
			printf("\n> ");
			for(i=3;i<bufferU[2]+3+2&&i<DIMBUF;i++) printf("%2X ",bufferU[i]);
			printf("\n");
		}
		else if(bufferI[1]==SPI_READ&&bufferI[2]<0xFA){
			printf("\n< ");
			for(i=3;i<bufferI[2]+3&&i<DIMBUF;i++) printf("%2X ",bufferI[i]);
			printf("\n");
		}
		else if(bufferI[1]==SPI_WRITE&&bufferI[2]<0xFA){
			printf("\n> ");
			for(i=3;i<bufferU[2]+3&&i<DIMBUF;i++) printf("%2X ",bufferU[i]);
			printf("\n");
		}
		fflush(stdout);

		return 0 ;
	}

//this is needed to use the same code on both win and linux
#define EQ(s) !strncmp(s,dev,64)
#define PrintMessage printf

	if(loadfile[0]){ 		//write
		if(Load(dev,loadfile)==-1){
			PrintMessage(strings[S_NoCode2]);
			PrintMessage("\n");
			exit(-1);
		}
		if(!strncmp(dev,"AT",2)&&loadfileEE[0]) LoadEE(dev,loadfileEE);
//-------------PIC10-16---------------------------------------------------------
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		if(EQ("10F200")||EQ("10F204")||EQ("10F220")){
			Write12F5xx(0x100,0xFF);						//256
		}
		else if(EQ("12F508")||EQ("10F202")||EQ("10F206")||EQ("10F222")){
			Write12F5xx(0x200,0x1FF);						//512
		}
		else if(EQ("16F54")){
			Write12F5xx(0x200,-1);							//512, no osccal
		}
		else if(EQ("16C83")||EQ("16F83")||EQ("16F83A")){
			Write16F8x(0x200,ee?0x40:0);					//512, 64
		}
		else if(EQ("12F509")||EQ("12F510")||EQ("16F505")||EQ("16F506")){
			Write12F5xx(0x400,0x3FF);						//1K
		}
		else if(EQ("12F519")||EQ("16F526")){
			Write12F5xx(0x440,0x3FF);						//1K + 64
		}
		else if(EQ("12F609")||EQ("12F615")||EQ("16F610")){
			Write12F61x(0x400);								//1K
		}
		else if(EQ("16C84")||EQ("16F84")||EQ("16F84A")){
			Write16F8x(0x400,ee?0x40:0);					//1K, 64
		}
		else if(EQ("12F629")||EQ("12F675")||EQ("16F630")||EQ("16F676")){
			Write12F62x(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("16F627")){
			Write16F62x(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("12F635")||EQ("16F631")||EQ("16F627A")||EQ("16F785")){
			Write12F6xx(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("16F818")){
			Write16F81x(0x400,ee?0x80:0);					//1K, 128, vdd no delay
		}
		else if(EQ("16F57")||EQ("16F59")){
			Write12F5xx(0x800,-1);							//2K, no osccal
		}
		else if(EQ("16F616")){
			Write12F61x(0x800);								//2K
		}
		else if(EQ("16F716")){
			Write16F71x(0x800,1);							//2K, vdd
		}
		else if(EQ("16F870")||EQ("16F871")||EQ("16F872")){
			Write16F87x(0x800,ee?0x40:0);					//2K, 64
		}
		else if(EQ("16F628A")){
			Write12F6xx(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("16F628")){
			Write16F62x(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("16F882")){
			Write16F88x(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("12F683")||EQ("16F636")||EQ("16F639")||EQ("16F677")||EQ("16F684")||EQ("16F687")||EQ("16F785")){
			Write12F6xx(0x800,ee?0x100:0);					//2K, 256
		}
		else if(EQ("16F819")){
			Write16F81x(0x800,ee?0x100:0);					//2K, 256, vdd no delay
		}
		else if(EQ("16F1822")||EQ("16F1823")||EQ("16F1826")){
			Write16F1xxx(0x800,ee?0x100:0,0);				//2K, 256
		}
		else if(EQ("16F73")||EQ("16F74")){
			Write16F7x(0x1000,0);							//4K
		}
		else if(EQ("16F737")||EQ("16F747")){
			Write16F7x(0x1000,1);							//4K, vdd no delay
		}
		else if(EQ("16F873")||EQ("16F874")){
			Write16F87x(0x1000,ee?-0x80:0);					//4K, 128, ee@0x2200
		}
		else if(EQ("16F648A")||EQ("16F685")||EQ("16F688")||EQ("16F689")||EQ("16F690")||EQ("16F913")||EQ("16F914")){
			Write12F6xx(0x1000,ee?0x100:0);					//4K, 256
		}
		else if(EQ("16F873A")||EQ("16F874A")){
			Write16F87xA(0x1000,ee?0x80:0,0);				//4K, 128
		}
		else if(EQ("16F883")||EQ("16F884")){
			Write16F88x(0x1000,ee?0x100:0);					//4K, 256
		}
		else if(EQ("16F87")||EQ("16F88")){
			Write16F81x(0x1000,ee?0x100:0);					//4K, 256, vdd no delay
		}
		else if(EQ("16F1933")||EQ("16F1934")||EQ("16F1824")||EQ("16F1827")||EQ("16F1828")){
			Write16F1xxx(0x1000,ee?0x100:0,0);				//4K, 256
		}
		else if(EQ("16F76")||EQ("16F77")){
			Write16F7x(0x2000,0);							//8K
		}
		else if(EQ("16F767")||EQ("16F777")){
			Write16F7x(0x2000,1);							//8K, vdd no delay
		}
		else if(EQ("16F916")||EQ("16F917")||EQ("16F946")){
			Write12F6xx(0x2000,ee?0x100:0);					//8K, 256
		}
		else if(EQ("16F876")||EQ("16F877")){
			Write16F87x(0x2000,ee?-0x100:0);				//8K, 256, ee@0x2200
		}
		else if(EQ("16F876A")||EQ("16F877A")){
			Write16F87xA(0x2000,ee?0x100:0,0);				//8K, 256,
		}
		else if(EQ("16F886")||EQ("16F887")){
			Write16F88x(0x2000,ee?0x100:0);					//8K, 256
		}
		else if(EQ("16F1936")||EQ("16F1937")||EQ("16F1946")||EQ("16F1825")||EQ("16F1829")){
			Write16F1xxx(0x2000,ee?0x100:0,0);				//8K, 256
		}
		else if(EQ("16F1938")||EQ("16F1939")||EQ("16F1947")){
			Write16F1xxx(0x4000,ee?0x100:0,0);				//16K, 256
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Dispositivo non supportato in scrittura\r\n");
		}
	}
//-------------PIC18---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = vdd before vpp (12V)
//     1 = vdd before vpp (9V)
//     2 = low voltage entry with 32 bit key
//	bit [7:4]
//     0 = normal eeprom write algoritm
//     1 = with unlock sequence 55 AA
//	bit [11:8]
//     0 = 5ms erase delay, 1ms code write time, 5ms EE write delay, 5ms config write time
//     1 = 550ms erase delay, 1.2ms code write time, no config or EEPROM
//     2 = 550ms erase delay, 3.4ms code write time, no config or EEPROM
	else if(!strncmp(dev,"18F",3)){
		if(EQ("18F1230")){
			Write18Fx(0x1000,ee?0x80:0,8,0x0F0F,0x8787,0);		//4K, 128, 8
		}
		else if(EQ("18F2221")||EQ("18F4221")){
			Write18Fx(0x1000,ee?0x100:0,8,0x3F3F,0x8F8F,0);		//4K, 256, 8
		}
		else if(EQ("18F1220")||EQ("18F2220")||EQ("18F4220")){
			Write18Fx(0x1000,ee?0x100:0,8,0x10000,0x80,0x10);		//4K, 256, 8, EE with unlock
		}
		else if(EQ("18F1330")){
			Write18Fx(0x2000,ee?0x80:0,8,0x0F0F,0x8787,0);		//8K, 128, 8
		}
		else if(EQ("18F2321")||EQ("18F4321")){
			Write18Fx(0x2000,ee?0x100:0,8,0x3F3F,0x8F8F,0);		//8K, 256, 8
		}
		else if(EQ("18F1320")||EQ("18F2320")||EQ("18F4320")||EQ("18F2331")||EQ("18F4331")){
			Write18Fx(0x2000,ee?0x100:0,8,0x10000,0x80,0x10);		//8K, 256, 8, EE with unlock
		}
		else if(EQ("18F13K50")){
			Write18Fx(0x2000,ee?0x100:0,8,0x0F0F,0x8F8F,1);		//8K, 256, 9V
		}
		else if(EQ("18F23K20")||EQ("18F43K20")){
			Write18Fx(0x2000,ee?0x100:0,16,0x0F0F,0x8F8F,1);		//8K, 256, 9V
		}
		else if(EQ("18F2439")||EQ("18F4439")){
			Write18Fx(0x3000,ee?0x100:0,8,0x10000,0x80,0x10);		//12K, 256, 8, EE with unlock
		}
		else if(EQ("18F2410")||EQ("18F4410")){
			Write18Fx(0x4000,0,32,0x3F3F,0x8F8F,0);				//16K, 0, 32
		}
		else if(EQ("18F24J10")||EQ("18F44J10")){
			Write18Fx(0x4000,0,64,0x0101,0x8080,0x202);				//16K, 0, 64, LV
		}
		else if(EQ("18F24J11")||EQ("18F24J50")||EQ("18F44J11")||EQ("18F44J50")){
			Write18Fx(0x4000,0,64,0x0101,0x8080,0x102);				//16K, 0, 64, LV
		}
		else if(EQ("18F2450")||EQ("18F4450")){
			Write18Fx(0x4000,0,16,0x3F3F,0x8F8F,0);				//16K, 0, 16
		}
		else if(EQ("18F14K50")){
			Write18Fx(0x4000,ee?0x100:0,16,0x0F0F,0x8F8F,1);	//16K, 256, 9V
		}
		else if(EQ("18F24K20")||EQ("18F44K20")){
			Write18Fx(0x4000,ee?0x100:0,32,0x0F0F,0x8F8F,1);	//16K, 256, 9V
		}
		else if(EQ("18F2431")||EQ("18F4431")||EQ("18F242")||EQ("18F248")||EQ("18F442")||EQ("18F448")){
			Write18Fx(0x4000,ee?0x100:0,8,0x10000,0x80,0x10);		//16K, 256, 8, EE with unlock
		}
		else if(EQ("18F2420")||EQ("18F2423")||EQ("18F4420")||EQ("18F4423")||EQ("18F2480")||EQ("18F4480")){
			Write18Fx(0x4000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//16K, 256, 32
		}
		else if(EQ("18F2455")||EQ("18F2458")||EQ("18F4455")||EQ("18F4458")){
			Write18Fx(0x6000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//24K, 256, 32
		}
		else if(EQ("18F2539")||EQ("18F4539")){
			Write18Fx(0x6000,ee?0x100:0,8,0x10000,0x80,0x10);	//24K, 256, 8, EE with unlock
		}
		else if(EQ("18F2510")||EQ("18F4510")){
			Write18Fx(0x8000,0,32,0x3F3F,0x8F8F,0);				//32K, 0, 32
		}
		else if(EQ("18F25J10")||EQ("18F45J10")){
			Write18Fx(0x8000,0,64,0x0101,0x8080,0x202);			//32K, 0, 64, LV
		}
		else if(EQ("18F25J11")||EQ("18F25J50")||EQ("18F45J11")||EQ("18F45J50")){
			Write18Fx(0x8000,0,64,0x0101,0x8080,0x102);			//32K, 0, 64, LV
		}
		else if(EQ("18F252")||EQ("18F258")||EQ("18F452")||EQ("18F458")){
			Write18Fx(0x8000,ee?0x100:0,8,0x10000,0x80,0x10);	//32K, 256, 8, EE with unlock
		}
		else if(EQ("18F2550")||EQ("18F2553")||EQ("18F4550")||EQ("18F4553")||EQ("18F2520")||EQ("18F2523")||EQ("18F4520")||EQ("18F4523")||EQ("18F2580")||EQ("18F4580")){
			Write18Fx(0x8000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//32K, 256, 32
		}
		else if(EQ("18F25K20")||EQ("18F45K20")){
			Write18Fx(0x8000,ee?0x100:0,32,0x0F0F,0x8F8F,1);	//32K, 256, 32, 9V
		}
		else if(EQ("18F2515")||EQ("18F4515")){
			Write18Fx(0xC000,0,64,0x3F3F,0x8F8F,0);				//48K, 0, 64
		}
		else if(EQ("18F2525")||EQ("18F2585")||EQ("18F4525")||EQ("18F4585")){
			Write18Fx(0xC000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//48K, 1K, 64
		}
		else if(EQ("18F2610")||EQ("18F4610")){
			Write18Fx(0x10000,0,64,0x3F3F,0x8F8F,0);			//64K, 0, 64
		}
		else if(EQ("18F26J11")||EQ("18F26J13")||EQ("18F26J50")||EQ("18F26J53")||EQ("18F46J11")||EQ("18F46J13")||EQ("18F46J50")||EQ("18F46J53")){
			Write18Fx(0x10000,0,64,0x0101,0x8080,0x102);		//64K, 0, 64, LV
		}
		else if(EQ("18F2620")||EQ("18F2680")||EQ("18F4620")||EQ("18F4680")){
			Write18Fx(0x10000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//64K, 1K, 64
		}
		else if(EQ("18F26K20")||EQ("18F46K20")){
			Write18Fx(0x10000,ee?0x100:0,64,0x0F0F,0x8F8F,1);	//64K, 256, 64, 9V
		}
		else if(EQ("18F2682")||EQ("18F4682")){
			Write18Fx(0x14000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//80K, 1K, 64
		}
		else if(EQ("18F2685")||EQ("18F4685")){
			Write18Fx(0x18000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//96K, 1K, 64
		}
		else if(EQ("18F27J13")||EQ("18F27J53")||EQ("18F47J13")||EQ("18F47J53")){
			Write18Fx(0x20000,0,64,0x0101,0x8080,0x102);		//128K, 0, 64, LV
		}
		else if(EQ("18F8722")){
			Write18Fx(0x20000,ee?0x400:0,64,0xFFFF,0x8787,0);	//128K, 1K, 64
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Dispositivo non supportato in scrittura\r\n");
		}
	}
//-------------PIC24---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = low voltage ICSP entry
//     1 = High voltage ICSP entry (6V)
//     2 = High voltage ICSP entry (12V) + PIC30F sequence (additional NOPs)
//	bit [7:4]
//	   0 = config area in the last 2 program words
//	   1 = config area in the last 3 program words
//	   2 = config area in the last 4 program words
//	   3 = 0xF80000 to 0xF80010 except 02 (24F)
//     4 = 0xF80000 to 0xF80016 (24H-33F)
//     5 = 0xF80000 to 0xF8000C (x16 bit, 30F)
//     6 = 0xF80000 to 0xF8000E (30FSMPS)
//	bit [11:8]
//	   0 = code erase word is 0x4064, row write is 0x4004
//	   1 = code erase word is 0x404F, row write is 0x4001
//	   2 = code erase word is 0x407F, row write is 0x4001, 55AA unlock and external timing (2 ms)
//	   3 = code erase word is 0x407F, row write is 0x4001, 55AA unlock and external timing (200 ms)
//	bit [15:12]
//	   0 = eeprom erase word is 0x4050, write word is 0x4004
//	   1 = eeprom erased with bulk erase, write word is 0x4004
//	   2 = eeprom erased with special sequence, write word is 0x4004
//	bit [19:16]
//	   0 = config write is 0x4000
//	   1 = config write is 0x4003
//	   2 = config write is 0x4004
//	   3 = config write is 0x4008
	else if(!strncmp(dev,"24F",3)||!strncmp(dev,"24H",3)||!strncmp(dev,"30F",3)||!strncmp(dev,"33F",3)){
		if(EQ("24F04KA200")||EQ("24F04KA201")){
			Write24Fx(0xB00,0,0x20031,0x05BE,32,2.0);				//1.375KW, HV
		}
		else if(EQ("24F08KA101")||EQ("24F08KA102")){
			Write24Fx(0x1600,ee?0x200:0,0x20031,0x05BE,32,2.0);		//2.75KW, HV, 512
		}
		else if(EQ("24F16KA101")||EQ("24F16KA102")){
			Write24Fx(0x2C00,ee?0x200:0,0x20031,0x05BE,32,2.0);		//5.5KW, HV, 512
		}
		else if(EQ("24FJ16GA002")||EQ("24FJ16GA004")){
			Write24Fx(0x2C00,0,0x10100,0x05BE,64,2.0);				//5.5KW
		}
		else if(EQ("24FJ32GA002")||EQ("24FJ32GA004")){
			Write24Fx(0x5800,0,0x10100,0x05BE,64,2.0);				//11KW
		}
		else if(EQ("24FJ32GA102")||EQ("24FJ32GA104")||EQ("24FJ32GB002")||EQ("24FJ32GB004")){
			Write24Fx(0x5800,0,0x10120,0x07F0,64,2.0);				//11KW
		}
		else if(EQ("24FJ48GA002")||EQ("24FJ48GA004")){
			Write24Fx(0x8400,0,0x10100,0x05BE,64,2.0);				//16.5KW
		}
		else if(EQ("24FJ64GA002")||EQ("24FJ64GA004")||EQ("24FJ64GA006")||EQ("24FJ64GA008")||EQ("24FJ64GA010")){
			Write24Fx(0xAC00,0,0x10100,0x05BE,64,2.0);				//22KW
		}
		else if(EQ("24FJ64GA102")||EQ("24FJ64GA104")||EQ("24FJ64GB002")||EQ("24FJ64GB004")){
			Write24Fx(0xAC00,0,0x10120,0x07F0,64,2.0);				//22KW
		}
		else if(EQ("24FJ64GB106")||EQ("24FJ64GB108")||EQ("24FJ64GB110")){
			Write24Fx(0xAC00,0,0x10110,0x07F0,64,2.0);				//22KW
		}
		else if(EQ("24FJ96GA006")||EQ("24FJ96GA008")||EQ("24FJ96GA010")){
			Write24Fx(0x10000,0,0x10100,0x05BE,64,2.0);				//32KW
		}
		else if(EQ("24FJ128GA006")||EQ("24FJ128GA008")||EQ("24FJ128GA010")){
			Write24Fx(0x15800,0,0x10100,0x05BE,64,2.0);				//44KW
		}
		else if(EQ("24FJ128GA106")||EQ("24FJ128GA108")||EQ("24FJ128GA110")||EQ("24FJ128GB106")||EQ("24FJ128GB108")||EQ("24FJ128GB110")){
			Write24Fx(0x15800,0,0x10110,0x07F0,64,2.0);				//44KW
		}
		else if(EQ("24FJ192GA106")||EQ("24FJ192GA108")||EQ("24FJ192GA110")||EQ("24FJ192GB106")||EQ("24FJ192GB108")||EQ("24FJ192GB110")){
			Write24Fx(0x20C00,0,0x10110,0x07F0,64,2.0);				//68KW
		}
		else if(EQ("24FJ256GA106")||EQ("24FJ256GA108")||EQ("24FJ256GA110")||EQ("24FJ256GB106")||EQ("24FJ256GB108")||EQ("24FJ256GB110")){
			Write24Fx(0x2AC00,0,0x10110,0x07F0,64,2.0);				//88KW
		}
		else if(!strncmp(dev,"33FJ06",6)){
			Write24Fx(0x1000,0,0x00140,0x07F0,64,2.0);				//2KW
		}
		else if(!strncmp(dev,"24HJ12",6)||!strncmp(dev,"33FJ12",6)){
			Write24Fx(0x2000,0,0x00140,0x07F0,64,2.0);				//4KW
		}
		else if(!strncmp(dev,"24HJ16",6)||!strncmp(dev,"33FJ16",6)){
			Write24Fx(0x2C00,0,0x00140,0x07F0,64,2.0);				//5.5KW
		}
		else if(!strncmp(dev,"24HJ32",6)||!strncmp(dev,"33FJ32",6)){
			Write24Fx(0x5800,0,0x00140,0x07F0,64,2.0);				//11KW
		}
		else if(!strncmp(dev,"24HJ64",6)||!strncmp(dev,"33FJ64",6)){
			Write24Fx(0xAC00,0,0x00140,0x07F0,64,2.0);				//22KW
		}
		else if(!strncmp(dev,"24HJ128",7)||!strncmp(dev,"33FJ128",7)){
			Write24Fx(0x15800,0,0x00140,0x07F0,64,2.0);				//44KW
		}
		else if(!strncmp(dev,"24HJ256",7)||!strncmp(dev,"33FJ256",7)){
			Write24Fx(0x2AC00,0,0x00140,0x07F0,64,2.0);				//88KW
		}
		else if(EQ("30F2010")){
			Write24Fx(0x2000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//4KW, 1K, HV12
		}
		else if(EQ("30F2011")||EQ("30F2012")){
			Write24Fx(0x2000,0,0x31252,0x05BE,32,2.0);				//4KW, HV12
		}
		else if(!strncmp(dev,"30F301",6)){
			Write24Fx(0x4000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//8KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F401",6)){
			Write24Fx(0x8000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//16KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F501",6)){
			Write24Fx(0xB000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//22KW, 1K, HV12
		}
		else if(EQ("30F6011")||EQ("30F6013")){
			Write24Fx(0x16000,ee?0x800:0,0x31252,0x05BE,32,2.0);	//44KW, 2K, HV12
		}
		else if(EQ("30F6010")||EQ("30F6012")||EQ("30F6014")||EQ("30F6015")){
			Write24Fx(0x18000,ee?0x1000:0,0x31252,0x05BE,32,2.0);	//49KW, 4K, HV12
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Dispositivo non supportato in scrittura\r\n");
		}
	}
//-------------ATMEL---------------------------------------------------------
	else if(!strncmp(dev,"AT",2)){
		if(EQ("AT90S1200")){
			WriteAT(0x400,ee?0x40:0);						//1K, 64
		}
		else if(EQ("AT90S2313")){
			WriteAT(0x800,ee?0x80:0);						//2K, 128
		}
		else if(EQ("ATtiny2313")){
			WriteATmega(0x800,ee?0x80:0,16,SLOW);			//2K, 128
		}
		else if(EQ("AT90S8515")||EQ("AT90S8535")){
			WriteAT(0x2000,ee?0x100:0);						//8K, 256
		}
		else if(EQ("ATmega8")||EQ("ATmega8A")||EQ("ATmega8515")||EQ("ATmega8535")){
			WriteATmega(0x2000,ee?0x200:0,32,0);				//8K, 512
		}
		else if(EQ("ATmega16")||EQ("ATmega16A")){
			WriteATmega(0x4000,ee?0x200:0,64,0);				//16K, 512
		}
		else if(EQ("ATmega32")||EQ("ATmega32A")){
			WriteATmega(0x8000,ee?0x400:0,64,0);				//32K, 1K
		}
		else if(EQ("ATmega64")||EQ("ATmega64A")){
			WriteATmega(0x10000,ee?0x800:0,128,0);			//64K, 2K
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Dispositivo non supportato in scrittura\r\n");
		}
	}
//-------------I2C---------------------------------------------------------
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"25",2)||!strncmp(dev,"93",2)){
		if(EQ("2400")){
			WriteI2C(0x10,0,1,10);			//16, 1B addr.
		}
		else if(EQ("2401")){
			WriteI2C(0x80,0,8,10);			//128, 1B addr.
		}
		else if(EQ("2402")){
			WriteI2C(0x100,0,8,10);			//256, 1B addr.
		}
		else if(EQ("2404")){
			WriteI2C(0x200,0,16,10);		//512, 1B addr.
		}
		else if(EQ("2408")){
			WriteI2C(0x400,0,16,10);		//1K, 1B addr.
		}
		else if(EQ("2416")){
			WriteI2C(0x800,0,16,10);		//2K, 1B addr.
		}
		else if(EQ("2432")){
			WriteI2C(0x1000,1,32,5);		//4K, 2B addr.
		}
		else if(EQ("2464")){
			WriteI2C(0x2000,1,32,5);		//8K, 2B addr.
		}
		else if(EQ("24128")){
			WriteI2C(0x4000,1,64,5);		//16K, 2B addr.
		}
		else if(EQ("24256")){
			WriteI2C(0x8000,1,64,5);		//32K, 2B addr.
		}
		else if(EQ("24512")){
			WriteI2C(0x10000,1,128,5);		//64K, 2B addr.
		}
		else if(EQ("241024")){
			WriteI2C(0x20000,0x201,256,5);	//128K, 2B addr.
		}
		else if(EQ("241025")){
			WriteI2C(0x20000,0x841,128,5);	//128K, 2B addr.
		}
//-------------Microwire EEPROM---------------------------------------------------------
		else if(EQ("93S46")){
			Write93Sx(0x80,6,8,10);							//128, 4W page, 10ms
		}
		else if(EQ("93x46")){
			Write93Cx(0x80,6,0);							//128,
		}
		else if(EQ("93x46A")){
			Write93Cx(0x80,7,1);							//128, x8
		}
		else if(EQ("93S56")){
			Write93Sx(0x100,8,8,10);						//256, 4W page, 10ms
		}
		else if(EQ("93x56")){
			Write93Cx(0x100,8,0);							//256,
		}
		else if(EQ("93x56A")){
			Write93Cx(0x100,9,1);							//256, x8
		}
		else if(EQ("93S66")){
			Write93Sx(0x200,8,8,10);						//512, 4W page, 10ms
		}
		else if(EQ("93x66")){
			Write93Cx(0x200,8,0);						//512,
		}
		else if(EQ("93x66A")){
			Write93Cx(0x200,9,1);						//512, x8
		}
		else if(EQ("93x76")){
			Write93Cx(0x400,10,0);						//1k
		}
		else if(EQ("93x76A")){
			Write93Cx(0x400,11,1);						//1k, x8
		}
		else if(EQ("93x86")){
			Write93Cx(0x800,10,0);						//2k,
		}
		else if(EQ("93x86A")){
			Write93Cx(0x800,11,1);						//2k, x8
		}
//-------------SPI---------------------------------------------------------
		else if(EQ("25010")){
			Write25xx(0x80,16,10);								//128
		}
		else if(EQ("25020")){
			Write25xx(0x100,16,10);								//256
		}
		else if(EQ("25040")){
			Write25xx(0x200,16,10);								//512
		}
		else if(EQ("25080")){
			Write25xx(0x400,16,5);								//1K
		}
		else if(EQ("25160")){
			Write25xx(0x800,16,5);								//2K
		}
		else if(EQ("25320")){
			Write25xx(0x1000,32,5);								//4K
		}
		else if(EQ("25640")){
			Write25xx(0x2000,32,5);								//8K
		}
		else if(EQ("25128")){
			Write25xx(0x4000,64,5);								//16K
		}
		else if(EQ("25256")){
			Write25xx(0x8000,64,5);								//32K
		}
		else if(EQ("25512")){
			Write25xx(0x10000,128,6);							//64K
		}
		else if(EQ("251024")){
			Write25xx(0x20000,256,5);							//128K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
	}
//-------------Unsupported device---------------------------------------------------------
		else{
			PrintMessage(strings[S_nodev_w]); //"Dispositivo non supportato in scrittura\r\n");
		}
	}


//-------------Read---------------------------------------------------------
	else{
//-------------PIC10-16---------------------------------------------------------
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		if(EQ("10F200")||EQ("10F204")||EQ("10F220")){
			Read12F5xx(0x100,r?0x40:5);						//256
		}
		else if(!strncmp(dev,"12C508",6)||EQ("16F54")){
			Read12F5xx(0x200,r?0x40:4);						//512
		}
		else if(EQ("12F508")||EQ("10F202")||EQ("10F206")||EQ("10F222")){
			Read12F5xx(0x200,r?0x40:5);						//512
		}
		else if(EQ("16C83")||EQ("16F83")||EQ("16F83A")){
			Read16Fxxx(0x200,ee?0x40:0,r?0x10:8,1);			//512, 64, vdd
		}
		else if(!strncmp(dev,"12C509",6)){
			Read12F5xx(0x400,r?0x40:4);						//1K
		}
		else if(EQ("12F509")||EQ("12F510")||EQ("16F505")||EQ("16F506")){
			Read12F5xx(0x400,r?0x40:5);						//1K
		}
		else if(EQ("12F519")||EQ("16F526")){
			Read12F5xx(0x440,r?0x60:8);						//1K + 64
		}
		else if(EQ("12C671")||EQ("12CE673")){
			Read16Fxxx(0x400,0,r?0x100:0,0);				//1K, vpp
		}
		else if(EQ("12F609")||EQ("12F615")||EQ("16F610")){
			Read16Fxxx(0x400,0,r?0x40:9,0);					//1K, vpp, cal1
		}
		else if(EQ("16C84")||EQ("16F84")||EQ("16F84A")){
			Read16Fxxx(0x400,ee?0x40:0,r?0x10:8,1);			//1K, 64, vdd
		}
		else if(EQ("12F635")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x40:10,0);		//1K, 128, vpp, cal1 + cal2
		}
		else if(EQ("16F631")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x80:9,0);			//1K, 128, vpp, cal1
		}
		else if(EQ("12F629")||EQ("12F675")||EQ("16F630")||EQ("16F676")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x20:8,0);			//1K, 128, vpp
		}
		else if(EQ("16F627")){
			Read16Fxxx(0x400,ee?-0x80:0,r?0x10:8,0);		//1K, 128, vpp, ee@0x2200
		}
		else if(EQ("16F627A")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x10:8,0);			//1K, 128, vpp
		}
		else if(EQ("16F818")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x10:8,2);			//1K, 128, vdd short delay
		}
		else if(EQ("16F57")||EQ("16F59")){
			Read12F5xx(0x800,r?0x40:4);						//2K
		}
		else if(EQ("12C672")||EQ("12CE674")){
			Read16Fxxx(0x800,0,r?0x100:0,0);				//2K, vpp
		}
		else if(EQ("16F716")){
			Read16Fxxx(0x800,0,8,2);						//2K, vdd
		}
		else if(EQ("16F616")){
			Read16Fxxx(0x800,0,r?0x40:9,0);					//2K, vpp, cal1
		}
		else if(EQ("16F870")||EQ("16F871")||EQ("16F872")){
			Read16Fxxx(0x800,ee?0x40:0,r?0x100:8,1);		//2K, 64, vdd
		}
		else if(EQ("16F628")){
			Read16Fxxx(0x800,ee?-0x80:0,r?0x10:8,0);		//2K, 128, vpp, ee@0x2200
		}
		else if(EQ("16F628A")){
			Read16Fxxx(0x800,ee?0x80:0,r?0x10:8,0);			//2K, 128, vpp
		}
		else if(EQ("16F882")){
			Read16Fxxx(0x800,ee?0x80:0,r?0x80:10,0);		//2K, 128, vpp, config2 + cal1
		}
		else if(EQ("16F819")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x10:8,2);		//2K, 256, vdd short delay
		}
		else if(EQ("12F683")||EQ("16F684")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x40:9,0);		//2K, 256, vpp, cal1
		}
		else if(EQ("16F636")||EQ("16F639")||EQ("16F785")||EQ("16F785")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x40:10,0);		//2K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F677")||EQ("16F687")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x80:9,0);		//2K, 256, vpp, cal1
		}
		else if(EQ("16F1822")||EQ("16F1823")||EQ("16F1826")){
			Read16F1xxx(0x800,ee?0x100:0,r?0x200:11,0);		//2K, 256, vpp
		}
		else if(EQ("16F73")||EQ("16F74")){
			Read16Fxxx(0x1000,0,r?0x20:8,1);				//4K, vdd
		}
		else if(EQ("16F737")||EQ("16F747")){
			Read16Fxxx(0x1000,0,r?0x40:9,2);				//4K, vdd short delay
		}
		else if(EQ("16F873A")||EQ("16F874A")){
			Read16Fxxx(0x1000,ee?0x80:0,r?0x100:8,1);		//4K, 128, vdd
		}
		else if(EQ("16F873")||EQ("16F874")){
			Read16Fxxx(0x1000,ee?-0x80:0,r?0x100:8,1);		//4K, 128, vdd, ee@0x2200
		}
		else if(EQ("16F685")||EQ("16F689")||EQ("16F690")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x80:9,0);		//4K, 256, vpp, cal1
		}
		else if(EQ("16F688")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x40:9,0);		//4K, 256, vpp, cal1
		}
		else if(EQ("16F883")||EQ("16F884")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x80:10,0);		//4K, 256, vpp, config2 + cal1
		}
		else if(EQ("16F648A")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x10:8,0);		//4K, 256, vpp
		}
		else if(EQ("16F87")||EQ("16F88")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x10:9,2);		//4K, 256, vdd short delay
		}
		else if(EQ("16F913")||EQ("16F914")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x40:10,0);		//4K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F1933")||EQ("16F1934")||EQ("16F1824")||EQ("16F1827")||EQ("16F1828")){
			Read16F1xxx(0x1000,ee?0x100:0,r?0x200:11,0);	//4K, 256, vpp
		}
		else if(EQ("16F76")||EQ("16F77")){
			Read16Fxxx(0x2000,0,r?0x20:8,1);				//8K, vdd
		}
		else if(EQ("16F767")||EQ("16F777")){
			Read16Fxxx(0x2000,0,r?0x40:9,2);				//8K, vdd short delay
		}
		else if(EQ("16F876A")||EQ("16F877A")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x100:8,1);		//8K, 256, vdd
		}
		else if(EQ("16F876")||EQ("16F877")){
			Read16Fxxx(0x2000,ee?-0x100:0,r?0x100:8,1);	//8K, 256, vdd, ee@0x2200
		}
		else if(EQ("16F886")||EQ("16F887")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x80:10,0);		//8K, 256, vpp, config2 + cal1
		}
		else if(EQ("16F916")||EQ("16F917")||EQ("16F946")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x40:10,0);		//8K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F1936")||EQ("16F1937")||EQ("16F1946")||EQ("16F1825")||EQ("16F1829")){
			Read16F1xxx(0x2000,ee?0x100:0,r?0x200:11,0);	//8K, 256, vpp
		}
		else if(EQ("16F1938")||EQ("16F1939")||EQ("16F1947")){
			Read16F1xxx(0x4000,ee?0x100:0,r?0x200:11,0);	//16K, 256, vpp
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
	}
//-------------PIC18---------------------------------------------------------
	else if(!strncmp(dev,"18F",3)){
		if(EQ("18F1230")){
			Read18Fx(0x1000,ee?0x80:0,0);					//4K, 128
		}
		else if(EQ("18F2221")||EQ("18F4221")||EQ("18F1220")||EQ("18F2220")||EQ("18F4220")){
			Read18Fx(0x1000,ee?0x100:0,0);					//4K, 256
		}
		else if(EQ("18F1330")){
			Read18Fx(0x2000,ee?0x80:0,0);					//8K, 128
		}
		else if(EQ("18F2321")||EQ("18F4321")||EQ("18F1320")||EQ("18F2320")||EQ("18F4320")||EQ("18F2331")||EQ("18F4331")){
			Read18Fx(0x2000,ee?0x100:0,0);					//8K, 256
		}
		else if(EQ("18F13K50")||EQ("18F23K20")||EQ("18F43K20")){
			Read18Fx(0x2000,ee?0x100:0,1);					//8K, 256, 9V
		}
		else if(EQ("18F2439")||EQ("18F4439")){
			Read18Fx(0x3000,ee?0x100:0,0);					//12K, 256
		}
		else if(EQ("18F2410")||EQ("18F4410")||EQ("18F2450")||EQ("18F4450")){
			Read18Fx(0x4000,0,0);							//16K, 0
		}
		else if(EQ("18F24J10")||EQ("18F44J10")||EQ("18F24J11")||EQ("18F24J50")||EQ("18F44J11")||EQ("18F44J50")){
			Read18Fx(0x4000,0,2);							//16K, 0, LV
		}
		else if(EQ("18F2420")||EQ("18F2423")||EQ("18F4420")||EQ("18F4423")||EQ("18F2431")||EQ("18F4431")||EQ("18F2480")||EQ("18F4480")||EQ("18F242")||EQ("18F248")||EQ("18F442")||EQ("18F448")){
			Read18Fx(0x4000,ee?0x100:0,0);					//16K, 256
		}
		else if(EQ("18F14K50")||EQ("18F24K20")||EQ("18F44K20")){
			Read18Fx(0x4000,ee?0x100:0,1);					//16K, 256, 9V
		}
		else if(EQ("18F2455")||EQ("18F2458")||EQ("18F4455")||EQ("18F4458")||EQ("18F2539")||EQ("18F4539")){
			Read18Fx(0x6000,ee?0x100:0,0);					//24K, 256
		}
		else if(EQ("18F2510")||EQ("18F4510")){
			Read18Fx(0x8000,0,0);							//32K, 0
		}
		else if(EQ("18F25J10")||EQ("18F25J11")||EQ("18F25J50")||EQ("18F45J10")||EQ("18F45J11")||EQ("18F45J50")){
			Read18Fx(0x8000,0,2);							//32K, 0, LV
		}
		else if(EQ("18F2550")||EQ("18F2553")||EQ("18F4550")||EQ("18F4553")||EQ("18F2520")||EQ("18F2523")||EQ("18F4520")||EQ("18F4523")||EQ("18F2580")||EQ("18F4580")||EQ("18F252")||EQ("18F258")||EQ("18F452")||EQ("18F458")){
			Read18Fx(0x8000,ee?0x100:0,0);					//32K, 256
		}
		else if(EQ("18F25K20")||EQ("18F45K20")){
			Read18Fx(0x8000,ee?0x100:0,1);					//32K, 256, 9V
		}
		else if(EQ("18F2515")||EQ("18F4515")){
			Read18Fx(0xC000,0,0);							//48K, 0
		}
		else if(EQ("18F2525")||EQ("18F2585")||EQ("18F4525")||EQ("18F4585")){
			Read18Fx(0xC000,ee?0x400:0,0);					//48K, 1K
		}
		else if(EQ("18F2610")||EQ("18F4610")){
			Read18Fx(0x10000,0,0);							//64K, 0
		}
		else if(EQ("18F26J11")||EQ("18F26J13")||EQ("18F26J50")||EQ("18F26J53")||EQ("18F46J11")||EQ("18F46J13")||EQ("18F46J50")||EQ("18F46J53")){
			Read18Fx(0x10000,0,2);							//64K, 0, LV
		}
		else if(EQ("18F2620")||EQ("18F2680")||EQ("18F4620")||EQ("18F4680")){
			Read18Fx(0x10000,ee?0x400:0,0);					//64K, 1K
		}
		else if(EQ("18F26K20")||EQ("18F46K20")){
			Read18Fx(0x10000,ee?0x400:0,1);					//64K, 1K, 9V
		}
		else if(EQ("18F2682")||EQ("18F4682")){
			Read18Fx(0x14000,ee?0x400:0,0);					//80K, 1K
		}
		else if(EQ("18F2685")||EQ("18F4685")){
			Read18Fx(0x18000,ee?0x400:0,0);					//96K, 1K
		}
		else if(EQ("18F27J13")||EQ("18F27J53")||EQ("18F47J13")||EQ("18F47J53")){
			Read18Fx(0x20000,0,2);							//128K, 0, LV
		}
		else if(EQ("18F8722")){
			Read18Fx(0x20000,ee?0x400:0,0);					//128K, 1K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
	}
//-------------PIC24---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = low voltage ICSP entry
//     1 = High voltage ICSP entry (6V)
//     2 = High voltage ICSP entry (12V) + PIC30F sequence (additional NOPs)
//	bit [7:4]
//	   0 = config area in the last 2 program words
//	   1 = config area in the last 3 program words
//	   2 = config area in the last 4 program words
//	   3 = 0xF80000 to 0xF80010 except 02 (24F)
//     4 = 0xF80000 to 0xF80016 (24H-33F)
//     5 = 0xF80000 to 0xF8000C (x16 bit, 30F)
//     6 = 0xF80000 to 0xF8000E (30FSMPS)
	else if(!strncmp(dev,"24F",3)||!strncmp(dev,"24H",3)||!strncmp(dev,"30F",3)||!strncmp(dev,"33F",3)){
		if(EQ("24F04KA200")||EQ("24F04KA201")){
			Read24Fx(0xB00,0,0x31,0x05BE,r?0x800:0);				//1.375KW, HV
		}
		else if(EQ("24F08KA101")||EQ("24F08KA102")){
			Read24Fx(0x1600,ee?0x200:0,0x31,0x05BE,r?0x800:0);		//2.75KW, HV, 512
		}
		else if(EQ("24F16KA101")||EQ("24F16KA102")){
			Read24Fx(0x2C00,ee?0x200:0,0x31,0x05BE,r?0x800:0);		//5.5KW, HV, 512
		}
		else if(EQ("24FJ16GA002")||EQ("24FJ16GA004")){
			Read24Fx(0x2C00,0,0,0x05BE,r?0x800:0);					//5.5KW
		}
		else if(EQ("24FJ32GA002")||EQ("24FJ32GA004")){
			Read24Fx(0x5800,0,0,0x05BE,r?0x800:0);					//11KW
		}
		else if(EQ("24FJ48GA002")||EQ("24FJ48GA004")){
			Read24Fx(0x8400,0,0,0x05BE,r?0x800:0);					//16.5KW
		}
		else if(EQ("24FJ64GA002")||EQ("24FJ64GA004")||EQ("24FJ64GA006")||EQ("24FJ64GA008")||EQ("24FJ64GA010")){
			Read24Fx(0xAC00,0,0,0x05BE,r?0x800:0);					//22KW
		}
		else if(EQ("24FJ64GB106")||EQ("24FJ64GB108")||EQ("24FJ64GB110")){
			Read24Fx(0xAC00,0,0x10,0x07F0,r?0x800:0);					//22KW
		}
		else if(EQ("24FJ96GA006")||EQ("24FJ96GA008")||EQ("24FJ96GA010")){
			Read24Fx(0x10000,0,0,0x05BE,r?0x800:0);					//32KW
		}
		else if(EQ("24FJ128GA006")||EQ("24FJ128GA008")||EQ("24FJ128GA010")){
			Read24Fx(0x15800,0,0,0x05BE,r?0x800:0);					//44KW
		}
		else if(EQ("24FJ128GA106")||EQ("24FJ128GA108")||EQ("24FJ128GA110")||EQ("24FJ128GB106")||EQ("24FJ128GB108")||EQ("24FJ128GB110")){
			Read24Fx(0x15800,0,0x10,0x07F0,r?0x800:0);					//44KW
		}
		else if(EQ("24FJ192GA106")||EQ("24FJ192GA108")||EQ("24FJ192GA110")||EQ("24FJ192GB106")||EQ("24FJ192GB108")||EQ("24FJ192GB110")){
			Read24Fx(0x20C00,0,0x10,0x07F0,r?0x800:0);					//68KW
		}
		else if(EQ("24FJ256GA106")||EQ("24FJ256GA108")||EQ("24FJ256GA110")||EQ("24FJ256GB106")||EQ("24FJ256GB108")||EQ("24FJ256GB110")){
			Read24Fx(0x2AC00,0,0x10,0x07F0,r?0x800:0);					//88KW
		}
		else if(!strncmp(dev,"33FJ06",6)){
			Read24Fx(0x1000,0,0x40,0x07F0,r?0x800:0);				//2KW
		}
		else if(!strncmp(dev,"24HJ12",6)||!strncmp(dev,"33FJ12",6)){
			Read24Fx(0x2000,0,0x40,0x07F0,r?0x800:0);				//4KW
		}
		else if(!strncmp(dev,"24HJ16",6)||!strncmp(dev,"33FJ16",6)){
			Read24Fx(0x2C00,0,0x40,0x07F0,r?0x800:0);				//5.5KW
		}
		else if(!strncmp(dev,"24HJ32",6)||!strncmp(dev,"33FJ32",6)){
			Read24Fx(0x5800,0,0x40,0x07F0,r?0x1000:0);				//11KW
		}
		else if(!strncmp(dev,"24HJ64",6)||!strncmp(dev,"33FJ64",6)){
			Read24Fx(0xAC00,0,0x40,0x07F0,r?0x1000:0);				//22KW
		}
		else if(!strncmp(dev,"24HJ128",7)||!strncmp(dev,"33FJ128",7)){
			Read24Fx(0x15800,0,0x40,0x07F0,r?0x1000:0);				//44KW
		}
		else if(!strncmp(dev,"24HJ256",7)||!strncmp(dev,"33FJ256",7)){
			Read24Fx(0x2AC00,0,0x40,0x07F0,r?0x1000:0);				//88KW
		}
		else if(EQ("30F2010")){
			Read24Fx(0x2000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//4KW, 1K, HV12
		}
		else if(EQ("30F2011")||EQ("30F2012")){
			Read24Fx(0x2000,0,0x52,0x05BE,r?0x600:0);				//4KW, HV12
		}
		else if(!strncmp(dev,"30F301",6)){
			Read24Fx(0x4000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//8KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F401",6)){
			Read24Fx(0x8000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//16KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F501",6)){
			Read24Fx(0xB000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//22KW, 1K, HV12
		}
		else if(EQ("30F6011")||EQ("30F6013")){
			Read24Fx(0x16000,ee?0x800:0,0x52,0x05BE,r?0x600:0);		//44KW, 2K, HV12
		}
		else if(EQ("30F6010")||EQ("30F6012")||EQ("30F6014")||EQ("30F6015")){
			Read24Fx(0x18000,ee?0x1000:0,0x52,0x05BE,r?0x600:0);	//49KW, 4K, HV12
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
		}
//-------------ATMEL---------------------------------------------------------
	else if(!strncmp(dev,"AT",2)){
		if(EQ("AT90S1200")){
			ReadAT(0x400,ee?0x40:0,0);							//1K, 64
		}
		else if(EQ("AT90S2313")){
			ReadAT(0x800,ee?0x80:0,0);							//2K, 128
		}
		else if(EQ("ATtiny2313")){
			ReadAT(0x800,ee?0x80:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL+SLOW);//2K, 128
		}
		else if(EQ("AT90S8515")||EQ("AT90S8535")){
			ReadAT(0x2000,ee?0x100:0,0);						//8K, 256
		}
		else if(EQ("ATmega8")||EQ("ATmega8A")||EQ("ATmega8515")||EQ("ATmega8535")){
			ReadAT(0x2000,ee?0x200:0,LOCK+FUSE+FUSE_H+CAL);		//8K, 512
		}
		else if(EQ("ATmega16")||EQ("ATmega16A")){
			ReadAT(0x4000,ee?0x200:0,LOCK+FUSE+FUSE_H+CAL);		//16K, 512
		}
		else if(EQ("ATmega32")||EQ("ATmega32A")){
			ReadAT(0x8000,ee?0x400:0,LOCK+FUSE+FUSE_H+CAL);		//32K, 1K
		}
		else if(EQ("ATmega64")||EQ("ATmega64A")){
			ReadAT(0x10000,ee?0x800:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//64K, 2K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
	}
//-------------I2C---------------------------------------------------------
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"25",2)||!strncmp(dev,"93",2)){
		if(EQ("2400")){
			ReadI2C(0x10,0);						//16, 1B addr.
		}
		else if(EQ("2401")){
			ReadI2C(0x80,0);						//128, 1B addr.
		}
		else if(EQ("2402")){
			ReadI2C(0x100,0);						//256, 1B addr.
		}
		else if(EQ("2404")){
			ReadI2C(0x200,0);						//512, 1B addr.
		}
		else if(EQ("2408")){
			ReadI2C(0x400,0);						//1K, 1B addr.
		}
		else if(EQ("2416")){
			ReadI2C(0x800,0);						//2K, 1B addr.
		}
		else if(EQ("2432")){
			ReadI2C(0x1000,1);						//4K, 2B addr.
		}
		else if(EQ("2464")){
			ReadI2C(0x2000,1);						//8K, 2B addr.
		}
		else if(EQ("24128")){
			ReadI2C(0x4000,1);						//16K, 2B addr.
		}
		else if(EQ("24256")){
			ReadI2C(0x8000,1);						//32K, 2B addr.
		}
		else if(EQ("24512")){
			ReadI2C(0x10000,1);					//64K, 2B addr.
		}
		else if(EQ("241024")){
			ReadI2C(0x20000,0x201);				//128K, 2B addr.
		}
		else if(EQ("241025")){
			ReadI2C(0x20000,0x841);				//128K, 2B addr.
		}
//-------------Microwire EEPROM---------------------------------------------------------
		else if(EQ("93S46")||EQ("93x46")){
			Read93x(0x80,6,0);						//128, 6b addr
		}
		else if(EQ("93x46A")){
			Read93x(0x80,7,1);						//128, 6b addr x8
		}
		else if(EQ("93S56")||EQ("93x56")){
			Read93x(0x100,8,0);						//256, 8b addr
		}
		else if(EQ("93x56A")){
			Read93x(0x100,9,1);						//256, 8b addr x8
		}
		else if(EQ("93S66")||EQ("93x66")){
			Read93x(0x200,8,0);						//512, 8b addr
		}
		else if(EQ("93x66A")){
			Read93x(0x200,9,1);						//512, 8b addr x8
		}
		else if(EQ("93x76")){
			Read93x(0x400,10,0);						//1k, 10b addr
		}
		else if(EQ("93x76A")){
			Read93x(0x400,11,1);						//1k, 10b addr x8
		}
		else if(EQ("93x86")){
			Read93x(0x800,10,0);						//2k, 10b addr
		}
		else if(EQ("93x86A")){
			Read93x(0x800,11,1);						//2k, 10b addr x8
		}
//-------------SPI---------------------------------------------------------
		else if(EQ("25010")){
			Read25xx(0x80);							//128
		}
		else if(EQ("25020")){
			Read25xx(0x100);						//256
		}
		else if(EQ("25040")){
			Read25xx(0x200);						//512
		}
		else if(EQ("25080")){
			Read25xx(0x400);						//1K
		}
		else if(EQ("25160")){
			Read25xx(0x800);						//2K
		}
		else if(EQ("25320")){
			Read25xx(0x1000);						//4K
		}
		else if(EQ("25640")){
			Read25xx(0x2000);						//8K
		}
		else if(EQ("25128")){
			Read25xx(0x4000);						//16K
		}
		else if(EQ("25256")){
			Read25xx(0x8000);						//32K
		}
		else if(EQ("25512")){
			Read25xx(0x10000);						//64K
		}
		else if(EQ("251024")){
			Read25xx(0x20000);						//128K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}
	}
//-------------Unsupported device---------------------------------------------------------
		else{
			PrintMessage(strings[S_nodev_r]); //"Dispositivo non supportato in lettura\r\n");
		}

		if(savefile[0]) Save(dev,savefile);
		if(!strncmp(dev,"AT",2)&&savefileEE[0]) SaveEE(dev,savefileEE);
	}

#if !defined _WIN32 && !defined __CYGWIN__
	close(fd);
#endif
	return 0 ;
}


DWORD GetTickCount(){
	struct timeb now;
	ftime(&now);
	return now.time*1000+now.millitm;
}



void DisplayEE(){
	char s[256],t[256],v[256];
	int valid=0,empty=1;
	int i,j;
	s[0]=0;
	v[0]=0;
	PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
	for(i=0;i<sizeEE;i+=COL){
		valid=0;
		for(j=i;j<i+COL&&j<sizeEE;j++){
			sprintf(t,"%02X ",memEE[j]);
			strcat(s,t);
			sprintf(t,"%c",isprint(memEE[j])?memEE[j]:'.');
			strcat(v,t);
			if(memEE[j]<0xff) valid=1;
		}
		if(valid){
			PrintMessage("%04X: %s %s\r\n",i,s,v);
			empty=0;
		}
		s[0]=0;
		v[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
}

int StartHVReg(double V){
	int j=1,z;
	int vreg=(int)(V*10.0);
	bufferU[0]=0;
	DWORD t0,t;
	if(V==-1){
		bufferU[j++]=VREG_DIS;			//disable HV regulator
		bufferU[j++]=FLUSH;
		write();
		msDelay(40);
		read();
		return -1;
	}
	t=t0=GetTickCount();
	bufferU[j++]=VREG_EN;			//enable HV regulator
	bufferU[j++]=SET_VPP;
	bufferU[j++]=vreg<80?vreg-8:vreg;		//set VPP, compensate for offset at low voltage
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=READ_ADC;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	for(z=1;z<DIMBUF-2&&bufferI[z]!=READ_ADC;z++);
	int v=(bufferI[z+1]<<8)+bufferI[z+2];
//	printf("v=%d=%fV\n",v,v/G);
	if(v==0){
		PrintMessage(strings[S_lowUsbV]);	//"Tensione USB troppo bassa (VUSB<4.5V)\r\n"
		return 0;
	}
	for(;(v<(vreg/10.0-0.5)*G||v>(vreg/10.0+0.5)*G)&&t<t0+1500;t=GetTickCount()){
		j=1;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=READ_ADC;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(20);
		read();
		for(z=1;z<DIMBUF-2&&bufferI[z]!=READ_ADC;z++);
		v=(bufferI[z+1]<<8)+bufferI[z+2];
//		printf("v=%d=%fV\n",v,v/G);
	}
	if(v>(vreg/10.0+0.7)*G){
		PrintMessage(strings[S_HiVPP]);	//"Attenzione: tensione regolatore troppo alta\r\n\r\n"
		return 0;
	}
	else if(v<(vreg/10.0-0.7)*G){
		PrintMessage(strings[S_LowVPP]);	//"Attenzione: tensione regolatore troppo bassa\r\n\r\n"
		return 0;
	}
	else if(v==0){
		PrintMessage(strings[S_lowUsbV]);	//"Tensione USB troppo bassa (VUSB<4.5V)\r\n"
		return 0;
	}
	else{
		PrintMessage(strings[S_reg],t-t0,v/G);	//"Regolatore avviato e funzionante dopo T=%d ms VPP=%.1f\r\n\r\n"
		return vreg;
	}
}

void ProgID()
{
	int j=1;
	bufferU[0]=0;
	bufferU[j++]=PROG_RST;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	printf(strings[S_progver],bufferI[2],bufferI[3],bufferI[4]); //"FW versione %d.%d.%d\r\n"
	FWVersion=(bufferI[2]<<16)+(bufferI[3]<<8)+bufferI[4];
	printf(strings[S_progid],bufferI[5],bufferI[6],bufferI[7]);	//"ID Hw: %d.%d.%d"
	if(bufferI[7]==1) printf(" (18F2550)\r\n\r\n");
	else if(bufferI[7]==2) printf(" (18F2450)\r\n\r\n");
	else printf(" (?)\r\n\r\n");
}

int CheckV33Regulator()
{
	int i,j=1;
	bufferU[j++]=WRITE_RAM;
	bufferU[j++]=0x0F;
	bufferU[j++]=0x93;
	bufferU[j++]=0xFE;	//B0 = output
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0x01;	//B0=1
	bufferU[j++]=0;
	bufferU[j++]=READ_RAM;
	bufferU[j++]=0x0F;
	bufferU[j++]=0x81;	//Check if B1=1
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0x00;	//B0=0
	bufferU[j++]=0;
	bufferU[j++]=READ_RAM;
	bufferU[j++]=0x0F;
	bufferU[j++]=0x81;	//Check if B1=0
	bufferU[j++]=WRITE_RAM;
	bufferU[j++]=0x0F;
	bufferU[j++]=0x93;
	bufferU[j++]=0xFF;	//BX = input
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	for(j=1;j<DIMBUF-3&&bufferI[j]!=READ_RAM;j++);
	i=bufferI[j+3]&0x2;		//B1 should be high
	for(j+=3;j<DIMBUF-3&&bufferI[j]!=READ_RAM;j++);
	return (i+bufferI[j+3]&0x2)==2?1:0;
}

void TestHw() {
	int j=1;
	bufferU[0]=0;
	PrintMessage(strings[I_TestHW]);		//"Test hardware ..."
	getchar();
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	PrintMessage("VDD=5V, VPP=13V, D=0V, CK=0V, PGM=0V");
	getchar();
	j=1;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x15;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;			//VDD
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	PrintMessage("VDD=5V, VPP=0V, D=5V, CK=5V, PGM=5V");
	getchar();
	j=1;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x4;			//VPP
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	PrintMessage("VDD=0V, VPP=13V, D=5V, CK=0V, PGM=0V");
	getchar();
	j=1;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	PrintMessage("VDD=0V, VPP=0V, D=0V, CK=5V, PGM=0V");
	getchar();
	j=1;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;			//VPP
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
}

void msDelay(double delay)
{
#if !defined _WIN32 && !defined __CYGWIN__
	long x=(int)delay*1000.0;
	usleep(x>MinDly?x:MinDly);
#else
	Sleep((long)ceil(delay)>MinDly?(long)ceil(delay):MinDly);
#endif
}

int FindDevice(){
#if !defined _WIN32 && !defined __CYGWIN__
	if ((fd = open(path, O_RDONLY )) < 0) {
		perror("hiddev open");
		exit(1);
	}
	struct hiddev_devinfo device_info;
	ioctl(fd, HIDIOCGDEVINFO, &device_info);
	if(device_info.vendor!=vid||device_info.product!=pid){
		printf(strings[S_noprog]);
		return -1;
	}
	else printf(strings[S_prog]);

	rep_info_u.report_type=HID_REPORT_TYPE_OUTPUT;
	rep_info_i.report_type=HID_REPORT_TYPE_INPUT;
	rep_info_u.report_id=rep_info_i.report_id=HID_REPORT_ID_FIRST;
	rep_info_u.num_fields=rep_info_i.num_fields=1;
	ref_multi_u.uref.report_type=HID_REPORT_TYPE_OUTPUT;
	ref_multi_i.uref.report_type=HID_REPORT_TYPE_INPUT;
	ref_multi_u.uref.report_id=ref_multi_i.uref.report_id=HID_REPORT_ID_FIRST;
	ref_multi_u.uref.field_index=ref_multi_i.uref.field_index=0;
	ref_multi_u.uref.usage_index=ref_multi_i.uref.usage_index=0;
	ref_multi_u.num_values=ref_multi_i.num_values=DIMBUF;

#else
	char string[256];
	PSP_DEVICE_INTERFACE_DETAIL_DATA detailData;
	HANDLE DeviceHandle;
	HANDLE hDevInfo;
	GUID HidGuid;
	int MyDeviceDetected;
	char MyDevicePathName[1024];
	ULONG Length;
	ULONG Required;
	typedef struct _HIDD_ATTRIBUTES {
	    ULONG   Size;
	    USHORT  VendorID;
	    USHORT  ProductID;
	    USHORT  VersionNumber;
	} HIDD_ATTRIBUTES, *PHIDD_ATTRIBUTES;

	typedef void (__stdcall*GETHIDGUID) (OUT LPGUID HidGuid);
	typedef BOOLEAN (__stdcall*GETATTRIBUTES)(IN HANDLE HidDeviceObject,OUT PHIDD_ATTRIBUTES Attributes);
	typedef BOOLEAN (__stdcall*SETNUMINPUTBUFFERS)(IN  HANDLE HidDeviceObject,OUT ULONG  NumberBuffers);
	typedef BOOLEAN (__stdcall*GETNUMINPUTBUFFERS)(IN  HANDLE HidDeviceObject,OUT PULONG  NumberBuffers);
	typedef BOOLEAN (__stdcall*GETFEATURE) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*SETFEATURE) (IN  HANDLE HidDeviceObject, IN PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETREPORT) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*SETREPORT) (IN  HANDLE HidDeviceObject, IN PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETMANUFACTURERSTRING) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETPRODUCTSTRING) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETINDEXEDSTRING) (IN  HANDLE HidDeviceObject, IN ULONG  StringIndex, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	HIDD_ATTRIBUTES Attributes;
	SP_DEVICE_INTERFACE_DATA devInfoData;
	int LastDevice = FALSE;
	int MemberIndex = 0;
	LONG Result;
	char UsageDescription[256];

	Length=0;
	detailData=NULL;
	DeviceHandle=NULL;

	HMODULE hHID=0;
	GETHIDGUID HidD_GetHidGuid=0;
	GETATTRIBUTES HidD_GetAttributes=0;
	SETNUMINPUTBUFFERS HidD_SetNumInputBuffers=0;
	GETNUMINPUTBUFFERS HidD_GetNumInputBuffers=0;
	GETFEATURE HidD_GetFeature=0;
	SETFEATURE HidD_SetFeature=0;
	GETREPORT HidD_GetInputReport=0;
	SETREPORT HidD_SetOutputReport=0;
	GETMANUFACTURERSTRING HidD_GetManufacturerString=0;
	GETPRODUCTSTRING HidD_GetProductString=0;
	hHID = LoadLibrary("hid.dll");
	if(!hHID){
		printf("Can't find hid.dll");
		return 0;
	}
	HidD_GetHidGuid=(GETHIDGUID)GetProcAddress(hHID,"HidD_GetHidGuid");
	HidD_GetAttributes=(GETATTRIBUTES)GetProcAddress(hHID,"HidD_GetAttributes");
	HidD_SetNumInputBuffers=(SETNUMINPUTBUFFERS)GetProcAddress(hHID,"HidD_SetNumInputBuffers");
	HidD_GetNumInputBuffers=(GETNUMINPUTBUFFERS)GetProcAddress(hHID,"HidD_GetNumInputBuffers");
	HidD_GetFeature=(GETFEATURE)GetProcAddress(hHID,"HidD_GetFeature");
	HidD_SetFeature=(SETFEATURE)GetProcAddress(hHID,"HidD_SetFeature");
	HidD_GetInputReport=(GETREPORT)GetProcAddress(hHID,"HidD_GetInputReport");
	HidD_SetOutputReport=(SETREPORT)GetProcAddress(hHID,"HidD_SetOutputReport");
	HidD_GetManufacturerString=(GETMANUFACTURERSTRING)GetProcAddress(hHID,"HidD_GetManufacturerString");
	HidD_GetProductString=(GETPRODUCTSTRING)GetProcAddress(hHID,"HidD_GetProductString");
	if(HidD_GetHidGuid==NULL\
		||HidD_GetAttributes==NULL\
		||HidD_GetFeature==NULL\
		||HidD_SetFeature==NULL\
		||HidD_GetInputReport==NULL\
		||HidD_SetOutputReport==NULL\
		||HidD_GetManufacturerString==NULL\
		||HidD_GetProductString==NULL\
		||HidD_SetNumInputBuffers==NULL\
		||HidD_GetNumInputBuffers==NULL) return -1;


	HMODULE hSAPI=0;
	hSAPI = LoadLibrary("setupapi.dll");
	if(!hSAPI){
		printf("Can't find setupapi.dll");
		return 0;
	}
	typedef HDEVINFO (WINAPI* SETUPDIGETCLASSDEVS) (CONST GUID*,PCSTR,HWND,DWORD);
	typedef BOOL (WINAPI* SETUPDIENUMDEVICEINTERFACES) (HDEVINFO,PSP_DEVINFO_DATA,CONST GUID*,DWORD,PSP_DEVICE_INTERFACE_DATA);
	typedef BOOL (WINAPI* SETUPDIGETDEVICEINTERFACEDETAIL) (HDEVINFO,PSP_DEVICE_INTERFACE_DATA,PSP_DEVICE_INTERFACE_DETAIL_DATA_A,DWORD,PDWORD,PSP_DEVINFO_DATA);
	typedef BOOL (WINAPI* SETUPDIDESTROYDEVICEINFOLIST) (HDEVINFO);
	SETUPDIGETCLASSDEVS SetupDiGetClassDevsA=0;
	SETUPDIENUMDEVICEINTERFACES SetupDiEnumDeviceInterfaces=0;
	SETUPDIGETDEVICEINTERFACEDETAIL SetupDiGetDeviceInterfaceDetailA=0;
	SETUPDIDESTROYDEVICEINFOLIST SetupDiDestroyDeviceInfoList=0;
	SetupDiGetClassDevsA=(SETUPDIGETCLASSDEVS) GetProcAddress(hSAPI,"SetupDiGetClassDevsA");
	SetupDiEnumDeviceInterfaces=(SETUPDIENUMDEVICEINTERFACES) GetProcAddress(hSAPI,"SetupDiEnumDeviceInterfaces");
	SetupDiGetDeviceInterfaceDetailA=(SETUPDIGETDEVICEINTERFACEDETAIL) GetProcAddress(hSAPI,"SetupDiGetDeviceInterfaceDetailA");
	SetupDiDestroyDeviceInfoList=(SETUPDIDESTROYDEVICEINFOLIST) GetProcAddress(hSAPI,"SetupDiDestroyDeviceInfoList");
	if(SetupDiGetClassDevsA==NULL\
		||SetupDiEnumDeviceInterfaces==NULL\
		||SetupDiDestroyDeviceInfoList==NULL\
		||SetupDiGetDeviceInterfaceDetailA==NULL) return -1;


	/*
	The following code is adapted from Usbhidio_vc6 application example by Jan Axelson
	for more information see see http://www.lvr.com/hidpage.htm
	*/

	/*
	API function: HidD_GetHidGuid
	Get the GUID for all system HIDs.
	Returns: the GUID in HidGuid.
	*/
	HidD_GetHidGuid(&HidGuid);

	/*
	API function: SetupDiGetClassDevs
	Returns: a handle to a device information set for all installed devices.
	Requires: the GUID returned by GetHidGuid.
	*/
	hDevInfo=SetupDiGetClassDevs(&HidGuid,NULL,NULL,DIGCF_PRESENT|DIGCF_INTERFACEDEVICE);
	devInfoData.cbSize = sizeof(devInfoData);

	//Step through the available devices looking for the one we want.
	//Quit on detecting the desired device or checking all available devices without success.
	MemberIndex = 0;
	LastDevice = FALSE;
	do
	{
		/*
		API function: SetupDiEnumDeviceInterfaces
		On return, MyDeviceInterfaceData contains the handle to a
		SP_DEVICE_INTERFACE_DATA structure for a detected device.
		Requires:
		The DeviceInfoSet returned in SetupDiGetClassDevs.
		The HidGuid returned in GetHidGuid.
		An index to specify a device.
		*/
		Result=SetupDiEnumDeviceInterfaces (hDevInfo, 0, &HidGuid, MemberIndex, &devInfoData);
		if (Result != 0)
		{
			//A device has been detected, so get more information about it.
			/*
			API function: SetupDiGetDeviceInterfaceDetail
			Returns: an SP_DEVICE_INTERFACE_DETAIL_DATA structure
			containing information about a device.
			To retrieve the information, call this function twice.
			The first time returns the size of the structure in Length.
			The second time returns a pointer to the data in DeviceInfoSet.
			Requires:
			A DeviceInfoSet returned by SetupDiGetClassDevs
			The SP_DEVICE_INTERFACE_DATA structure returned by SetupDiEnumDeviceInterfaces.

			The final parameter is an optional pointer to an SP_DEV_INFO_DATA structure.
			This application doesn't retrieve or use the structure.
			If retrieving the structure, set
			MyDeviceInfoData.cbSize = length of MyDeviceInfoData.
			and pass the structure's address.
			*/
			//Get the Length value.
			//The call will return with a "buffer too small" error which can be ignored.
			Result = SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInfoData, NULL, 0, &Length, NULL);

			//Allocate memory for the hDevInfo structure, using the returned Length.
			detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Length);

			//Set cbSize in the detailData structure.
			detailData -> cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

			//Call the function again, this time passing it the returned buffer size.
			Result = SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInfoData, detailData, Length,&Required, NULL);

			// Open a handle to the device.
			// To enable retrieving information about a system mouse or keyboard,
			// don't request Read or Write access for this handle.
			/*
			API function: CreateFile
			Returns: a handle that enables reading and writing to the device.
			Requires:
			The DevicePath in the detailData structure
			returned by SetupDiGetDeviceInterfaceDetail.
			*/
			DeviceHandle=CreateFile(detailData->DevicePath,
				0, FILE_SHARE_READ|FILE_SHARE_WRITE,
				(LPSECURITY_ATTRIBUTES)NULL,OPEN_EXISTING, 0, NULL);

			/*
			API function: HidD_GetAttributes
			Requests information from the device.
			Requires: the handle returned by CreateFile.
			Returns: a HIDD_ATTRIBUTES structure containing
			the Vendor ID, Product ID, and Product Version Number.
			Use this information to decide if the detected device is
			the one we're looking for.
			*/

			//Set the Size to the number of bytes in the structure.
			Attributes.Size = sizeof(Attributes);
			Result = HidD_GetAttributes(DeviceHandle,&Attributes);

			//Is it the desired device?
			MyDeviceDetected = FALSE;
			char a[256];
			if (Attributes.VendorID == vid)
			{
				if (Attributes.ProductID == pid)
				{
					//Both the Vendor ID and Product ID match.
					MyDeviceDetected = TRUE;
					strcpy(MyDevicePathName,detailData->DevicePath);

					// Get a handle for writing Output reports.
					WriteHandle=CreateFile(detailData->DevicePath,
						GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE,
						(LPSECURITY_ATTRIBUTES)NULL,OPEN_EXISTING,0,NULL);

					//Get a handle to the device for the overlapped ReadFiles.
					ReadHandle=CreateFile(detailData->DevicePath,
						GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,(LPSECURITY_ATTRIBUTES)NULL,
						OPEN_EXISTING,FILE_FLAG_OVERLAPPED,NULL);

					if (hEventObject) CloseHandle(hEventObject);
					hEventObject = CreateEvent(NULL,TRUE,TRUE,"");

					//Set the members of the overlapped structure.
					HIDOverlapped.hEvent = hEventObject;
					HIDOverlapped.Offset = 0;
					HIDOverlapped.OffsetHigh = 0;
					Result=HidD_SetNumInputBuffers(DeviceHandle,64);
				}
				else
					//The Product ID doesn't match.
					CloseHandle(DeviceHandle);
			}
			else
				//The Vendor ID doesn't match.
				CloseHandle(DeviceHandle);
		//Free the memory used by the detailData structure (no longer needed).
		free(detailData);
		}
		else
			//SetupDiEnumDeviceInterfaces returned 0, so there are no more devices to check.
			LastDevice=TRUE;
		//If we haven't found the device yet, and haven't tried every available device,
		//try the next one.
		MemberIndex = MemberIndex + 1;
	} //do
	while ((LastDevice == FALSE) && (MyDeviceDetected == FALSE));
	//Free the memory reserved for hDevInfo by SetupDiClassDevs.
	SetupDiDestroyDeviceInfoList(hDevInfo);

	if (MyDeviceDetected == FALSE){
		printf(strings[S_noprog]);	//"Can't find device\n"
		return -1;
	}

	if(info){
		printf("Device detected: vid=0x%04X pid=0x%04X\nPath: %s\n",vid,pid,MyDevicePathName);
		if(HidD_GetManufacturerString(DeviceHandle,string,sizeof(string))==TRUE) wprintf(L"Manufacturer string: %s\n",string);
		if(HidD_GetProductString(DeviceHandle,string,sizeof(string))==TRUE) wprintf(L"Product string: %s\n",string);
	}
#endif
	return 0;
}
