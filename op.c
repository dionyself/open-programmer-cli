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



#include "common.h"
#include "I2CSPI.h"
#include "deviceRW.h"
#include "fileIO.h"


#define CloseLogFile() if(logfile)fclose(logfile);

#if !defined _WIN32 && !defined __CYGWIN__
DWORD GetTickCount();
#endif
void msDelay(double delay);
void TestHw();
int StartHVReg(double V);
void ProgID();
void DisplayEE();
int FindDevice();

char** strings;
int saveLog=0,programID=0,MinDly=1,load_osccal=0,load_BKosccal=0;
int use_osccal=1,use_BKosccal=0;
int load_calibword=0,max_err=200;
int AVRlock=0x100,AVRfuse=0x100,AVRfuse_h=0x100,AVRfuse_x=0x100;
int ICDenable=0,ICDaddr=0x1FF0;
int FWVersion=0,HwID=0;
FILE* logfile=0;
char LogFileName[512]="";
char loadfile[512]="",savefile[512]="";
char loadfileEE[512]="",savefileEE[512]="";
int vid=0x04D8,pid=0x0100,info=0;

WORD *memCODE_W=0;
int size=0,sizeW=0,sizeEE=0,sizeCONFIG=0;
unsigned char *memCODE=0,*memEE=0,memID[8],memCONFIG[48];
double hvreg=0;
#if !defined _WIN32 && !defined __CYGWIN__
int fd = -1;
struct hiddev_report_info rep_info_i,rep_info_u;
struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
int DIMBUF=64;
char path[512]="";
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

	int ee=0,r=0,ver=0,c=0,support=0,i2c=0,spi_mode=0,i,j,testhw=0;
	char dev[64]="null";
	unsigned char tmpbuf[128];
	opterr = 0;
	int option_index = 0;
	//#include "strings.c"
	strinit();
	#if defined _WIN32
	int langID=GetUserDefaultLangID();
	if((langID&0xFF)==0x10)strings=strings_it;
	#else
	if(getenv("LANG")&&strstr(getenv("LANG"),"it")!=0) strings=strings_it;
	#endif
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
		{"icd",           required_argument,       0, 'I'},
		{"l",             optional_argument,       0, 'l'}, //-l=val
		{"log",           optional_argument,       0, 'l'},
		{"lock",          required_argument,       0, 'L'},
		{"mode",          required_argument,       0, 'm'},
		{"osccal",        no_argument,    &load_osccal, 1},
#if !defined _WIN32 && !defined __CYGWIN__
		{"p",             required_argument,       0, 'p'},
		{"path",          required_argument,       0, 'p'},
#endif
		{"pid",           required_argument,       0, 'P'},
		{"rep" ,          required_argument,       0, 'R'},
		{"reserved",      no_argument,              &r, 1},
		{"r",             no_argument,              &r, 1},
		{"save",          required_argument,       0, 's'},
		{"s",             required_argument,       0, 's'},
		{"saveEE",        required_argument,       0, 'S'},
		{"se",            required_argument,       0, 'S'},
		{"spi_r",         no_argument,            &i2c, 5},
		{"spi_w",         no_argument,            &i2c, 6},
		{"support",       no_argument,        &support, 1},
		{"use_BKosccal",  no_argument,   &use_BKosccal, 1},
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
				//MinDly = atoi(optarg);
				break;
			case 'e':	//max write errors
				max_err = atoi(optarg);
				break;
			case 'f':	//Atmel FUSE low
				i=sscanf(optarg, "%x", &AVRfuse);
				if(i!=1||AVRfuse<0||AVRfuse>0xFF) AVRfuse=0x100;
				break;
			case 'F':	//Atmel FUSE high
				i=sscanf(optarg, "%x", &AVRfuse_h);
				if(i!=1||AVRfuse_h<0||AVRfuse_h>0xFF) AVRfuse_h=0x100;
				break;
			case 'I':	//ICD routine address
				i=sscanf(optarg, "%x", &ICDaddr);
				if(i!=1||ICDaddr<0||ICDaddr>0xFFFF) ICDaddr=0x1F00;
				ICDenable=1;
				break;
			case 'l':	//save Log
				saveLog=1;
				if(optarg) strncpy(LogFileName,optarg,sizeof(LogFileName));
				break;
			case 'L':	//Atmel LOCK
				i=sscanf(optarg, "%x", &AVRlock);
				if(i!=1||AVRlock<0||AVRlock>0xFF) AVRlock=0x100;
				break;
			case 'm':	//SPI mode
				spi_mode = atoi(optarg);
				if(spi_mode<0) spi_mode=0;
				if(spi_mode>3) spi_mode=3;
				break;
#if !defined _WIN32 && !defined __CYGWIN__
			case 'p':	//hiddev path
				strncpy(path,optarg,sizeof(path)-1);
				break;
#endif
			case 'P':	//pid
				sscanf(optarg, "%x", &pid);
				break;
			case 'R':	//USB HID report size
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
				i=sscanf(optarg, "%x", &AVRfuse_x);
				if(i!=1||AVRfuse_x<0||AVRfuse_x>0xFF) AVRfuse_x=0x100;
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
		printf("OP v%s\nCopyright (C) Alberto Maccioni 2009-2012\
\n	For detailed info see http://openprog.altervista.org/\
\nThis program is free software; you can redistribute it and/or modify it under \
the terms of the GNU General Public License as published by the Free Software \
Foundation; either version 2 of the License, or (at your option) any later version.\
			\n",VERSION);
		return 0;
	}
	if (support){
		char list[8192]; //make sure list is long enough!!
		AddDevices(list);
		printf("%s\n",list);
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
	if(testhw){
		TestHw();
		return 0;
	}

#define CS 8
#define HLD 16
	if(i2c){	//I2C, SPI
		if(i2c==1){							//I2C receive 8 bit mode
			I2CReceive(0,tmpbuf[0],tmpbuf+1);
		}
		else if(i2c==2){					//I2C receive 16 bit mode
			I2CReceive(1,tmpbuf[0],tmpbuf+1);
		}
		else if(i2c==3){					//I2C transmit 8 bit mode
			I2CSend(0,tmpbuf[0],tmpbuf+1);
		}
		else if(i2c==4){					//I2C transmit 16 bit mode
			I2CSend(1,tmpbuf[0],tmpbuf+1);
		}
		else if(i2c==5){					//SPI receive
			I2CReceive(2+spi_mode,tmpbuf[0],tmpbuf+1);
		}
		else if(i2c==6){					//SPI receive
			I2CSend(2+spi_mode,tmpbuf[0],tmpbuf+1);
		}
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
		Write(dev,ee);	//choose the right function
	}
	else{		//read
		Read(dev,ee,r);	//choose the right function

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
		if(HwID==3) v>>=2;		//if 12 bit ADC
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
	HwID=bufferI[7];
	if(HwID==1) printf(" (18F2550)\r\n\r\n");
	else if(HwID==2) printf(" (18F2450)\r\n\r\n");
	else if(HwID==3) printf(" (18F2458/2553)\r\n\r\n");
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
	return (i+(bufferI[j+3]&0x2))==2?1:0;
}

void TestHw() {
	int j=1;
	bufferU[0]=0;
	PrintMessage(strings[I_TestHW]);		//"Test hardware ..."
	getchar();
	StartHVReg(13);
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
	struct hiddev_devinfo device_info;
	int i;
	if(path[0]==0){	//search all devices
		for(i=0;i<16;i++){
			sprintf(path,"/dev/usb/hiddev%d",i);
			if((fd = open(path, O_RDONLY ))>0){
				ioctl(fd, HIDIOCGDEVINFO, &device_info);
				if(device_info.vendor==vid&&device_info.product==pid) break;
				else close(fd);
			}
		}
		if(i==16){
			printf(strings[S_noprog]);
			return -1;
		}
	}
	else{	//user supplied path
		if ((fd = open(path, O_RDONLY )) < 0) {
			printf(strings[S_DevPermission],path);
			exit(1);
		}
		ioctl(fd, HIDIOCGDEVINFO, &device_info);
		if(device_info.vendor!=vid||device_info.product!=pid){
			printf(strings[S_noprog]);
			return -1;
		}
	}
	printf(strings[S_progDev],path);

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
	//char UsageDescription[256];

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
