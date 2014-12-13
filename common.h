//#define DEBUG
#define _APPNAME "OP"
#define _CMD

#if !defined _WIN32 && !defined __CYGWIN__
	#include <sys/ioctl.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <asm/types.h>
	#include <fcntl.h>
	#include <linux/hiddev.h>
	#include <linux/input.h>
	#include <sys/timeb.h>
	#include <stdint.h>
#else
	#include <windows.h>
	#include <setupapi.h>
	#include <ddk/hidusage.h>
	#include <ddk/hidpi.h>
	#include <math.h>
	#include <sys/timeb.h>
	#include <wchar.h>
#endif

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include "strings.h"
#include "instructions.h"

typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char BYTE;

//to use the same code of windows version
#define PrintMessage printf
#define PrintMessage1 printf
#define PrintMessage2 printf
#define PrintMessage3 printf
#define PrintMessage4 printf
#define PrintStatus(s,p1,p2) printf("\b\b\b\b%3d%%",p1); fflush(stdout);
#define	PrintStatusSetup() printf("    ");
#define	PrintStatusEnd() printf("\b\b\b\b");
#define	PrintStatusClear() //only for GUI
#define COL 16
#define VERSION "0.9.1"
#define G (12.0/34*1024/5)		//=72,2823529412
#define LOCK	1
#define FUSE	2
#define FUSE_H  4
#define FUSE_X	8
#define CAL 	16
#define SLOW	256

#if !defined _WIN32 && !defined __CYGWIN__		//Linux
	#define DIMBUF 64
	#define write() ioctl(fd, HIDIOCSUSAGES, &ref_multi_u); ioctl(fd,HIDIOCSREPORT, &rep_info_u);
	#define read() ioctl(fd, HIDIOCGUSAGES, &ref_multi_i); ioctl(fd,HIDIOCGREPORT, &rep_info_i);
	#define bufferU ref_multi_u.values
	#define bufferI ref_multi_i.values
	DWORD GetTickCount();
	struct hiddev_report_info rep_info_i,rep_info_u;
	struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
#else	//Windows
	#define DIMBUF 65
	#define write()	Result = WriteFile(WriteHandle,bufferU,DIMBUF,&BytesWritten,NULL);
	#define read()	Result = ReadFile(ReadHandle,bufferI,DIMBUF,&NumberOfBytesRead,(LPOVERLAPPED) &HIDOverlapped);\
					Result = WaitForSingleObject(hEventObject,10);\
					ResetEvent(hEventObject);\
					if(Result!=WAIT_OBJECT_0){\
						PrintMessage(strings[S_comTimeout]);	/*"comm timeout\r\n"*/\
					}

extern unsigned char bufferU[128],bufferI[128];
extern DWORD NumberOfBytesRead,BytesWritten;
extern ULONG Result;
extern HANDLE WriteHandle,ReadHandle;
extern OVERLAPPED HIDOverlapped;
extern HANDLE hEventObject;

#endif


extern int saveLog;
extern char** strings;
extern int fd;
extern int saveLog,programID,MinDly,load_osccal,load_BKosccal;
extern int use_osccal,use_BKosccal;
extern int load_calibword,max_err;
extern int AVRlock,AVRfuse,AVRfuse_h,AVRfuse_x;
extern int ICDenable,ICDaddr;
extern int FWVersion,HwID;
extern FILE* logfile;
extern char LogFileName[512];
extern char loadfile[512],savefile[512];
extern WORD *memCODE_W;
extern int size,sizeW,sizeEE,sizeCONFIG,sizeUSERID;
extern unsigned char *memCODE,*memEE,memID[8],memCONFIG[48],memUSERID[8];
extern double hvreg;
extern int RWstop;

int StartHVReg(double V);
void msDelay(double delay);
void DisplayEE();
void PrintMessageI2C(const char *msg);
int CheckV33Regulator(void);
void OpenLogFile(void);
void WriteLogIO();
void CloseLogFile();
unsigned int htoi(const char *hex, int length);
void PacketIO(double delay);
void CleanIO();
