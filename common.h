#if !defined _WIN32 && !defined __CYGWIN__
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <sys/timeb.h>
#else
#include <windows.h>
#include <setupapi.h>
#include <ddk/hidusage.h>
#include <ddk/hidpi.h>
#include <math.h>
#include <sys/timeb.h>
#include <wchar.h>
#endif

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
#define VERSION "0.7.9"
#define G (12.0/34*1024/5)		//=72,2823529412
#define LOCK	1
#define FUSE	2
#define FUSE_H  4
#define FUSE_X	8
#define CAL 	16
#define SLOW	256

#if !defined _WIN32 && !defined __CYGWIN__

#define write() ioctl(fd, HIDIOCSUSAGES, &ref_multi_u); ioctl(fd,HIDIOCSREPORT, &rep_info_u);
#define read() ioctl(fd, HIDIOCGUSAGES, &ref_multi_i); ioctl(fd,HIDIOCGREPORT, &rep_info_i);
#define bufferU ref_multi_u.values
#define bufferI ref_multi_i.values
extern DWORD GetTickCount();

#else
	#define write()	Result = WriteFile(WriteHandle,bufferU,DIMBUF,&BytesWritten,NULL);
	#define read()	Result = ReadFile(ReadHandle,bufferI,DIMBUF,&NumberOfBytesRead,(LPOVERLAPPED) &HIDOverlapped);\
					Result = WaitForSingleObject(hEventObject,10);\
					ResetEvent(hEventObject);\
					if(Result!=WAIT_OBJECT_0){\
						printf(strings[S_comTimeout]);	/*"comm timeout\r\n"*/\
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
extern int DIMBUF;
extern int saveLog,programID,MinDly,load_osccal,load_BKosccal;
extern int use_osccal,use_BKosccal;
extern int load_calibword,max_err;
extern int AVRlock,AVRfuse,AVRfuse_h,AVRfuse_x;
extern int ICDenable,ICDaddr;
extern int FWVersion,HwID;
extern FILE* logfile;
extern char LogFileName[512];
extern char loadfile[512],savefile[512];
extern WORD *dati_hex;
extern int size,sizeW,sizeEE,sizeCONFIG;
extern unsigned char *memCODE,*memEE,memID[8],memCONFIG[48];
extern double hvreg;
extern struct hiddev_report_info rep_info_i,rep_info_u;
extern struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;

int StartHVReg(double V);
void msDelay(double delay);
void WriteLogIO();
void PIC_ID(int id);
void DisplayEE();
void CloseLogFile();
//void PrintMessage(const char *msg);
void PrintMessageI2C(const char *msg);
void Read12F5xx(int dim, int dim2);
void Read16Fxxx(int dim,int dim2,int dim3,int vdd);
void Write12F5xx(int dim,int OscAddr);
void Write12F6xx(int dim,int dim2);
void Write12F61x(int dim);
void Write12F62x(int dim,int dim2);
void Write16F7x(int dim,int vdd);
void Write16F71x(int dim,int vdd);
void Write16F72x(int dim);
void Write16F8x(int dim,int dim2);
void Write16F81x(int dim,int dim2);
void Write16F87x(int dim,int dim2);
void Write16F87xA(int dim,int dim2,int seq);
void Write16F88x(int dim,int dim2);
void Read16F1xxx(int dim,int dim2,int dim3,int options);
void Write16F1xxx(int dim,int dim2,int options);
void Read18Fx(int dim,int dim2,int options);
void Write18Fx(int dim,int dim2,int wbuf,int eraseW1,int eraseW2,int options);
void Read24Fx(int dim,int dim2,int options,int appIDaddr,int executiveArea);
void Write24Fx(int dim,int dim2,int options,int appIDaddr,int rowSize, double wait);
void ReadAT(int dim, int dim2, int options);
void WriteAT(int dim, int dim2);
void WriteATmega(int dim, int dim2, int page, int options);
void ReadI2C(int dim,int addr);
void WriteI2C(int dim,int addr,int page);
void Read93x(int dim,int na,int options);
void Write93Sx(int dim,int na,int page);
void Write93Cx(int dim,int na, int options);
void Read25xx(int dim);
void Write25xx(int dim,int page);
void DisplayCODE16F(int size);
void DisplayEE16F(int size);
void DisplayCODE18F(int dim);
void DisplayCODE24F(int dim);
void DisplayEE24F();
