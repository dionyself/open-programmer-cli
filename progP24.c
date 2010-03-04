/*
 * progP24F.c - algorithms to program the PIC24 family of microcontrollers
 * Copyright (C) 2009 Alberto Maccioni
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

#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <sys/timeb.h>
#include <string.h>
#include "strings.h"
#include "instructions.h"

typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char BYTE;

#define write() ioctl(fd, HIDIOCSUSAGES, &ref_multi_u); ioctl(fd,HIDIOCSREPORT, &rep_info_u);
#define read() ioctl(fd, HIDIOCGUSAGES, &ref_multi_i); ioctl(fd,HIDIOCGREPORT, &rep_info_i);
#define bufferU ref_multi_u.values
#define bufferI ref_multi_i.values
#define EQ(s) !strncmp(s,dev,64)
#define PrintMessage printf
#define COL 16
#define max_err max_errori

extern int size,saveLog;
extern char** strings;
extern int fd;
extern int DIMBUF,saveLog,programID,MinRit,load_osccal,load_BKosccal,usa_osccal,usa_BKosccal;
extern int load_calibword,max_errori;
extern int lock,fuse,fuse_h,fuse_x;
extern int FWVersion;
extern FILE* RegFile;
extern char LogFileName[256];
extern WORD *dati_hex;
extern int size,sizeEE,sizeCONFIG;
extern unsigned char *memCODE,*memEE,memID[8],memCONFIG[34];
extern struct hiddev_report_info rep_info_i,rep_info_u;
extern struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
extern void msDelay(double delay);
extern void WriteLogIO();
extern void PIC_ID(int id);
extern int StartHVReg(double V);
extern DWORD GetTickCount();

void CheckData(int a,int b,int addr,int *err){
	if(a!=b){
		PrintMessage(strings[S_CodeVError],addr,addr,a,b);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
		(*err)++;
	}
}

void Read24Fx(int dim,int dim2,int options,int appIDaddr,int executiveArea,int EEbaseAddr){
// read 16 bit PIC 24Fxxxx
// deviceID @ 0xFF0000
// dim=program size (16 bit words)
// dim2=eeprom size (in words, area starts at 0x7F0000+EEbaseAddr)
// options:
//	bit 1 = use High voltage MCLR, else low voltage
//	bit 2 = read config area @ 0xF80000, else last two program words
// appIDaddr = application ID word lower address (high is 0x80)
// executiveArea = size of executive area (16 bit words, starting at 0x800000)
// EEbaseAddr = address of EEPROM area
	int k=0,k2=0,z=0,i,j;
	if(FWVersion<0x700){
		PrintMessage(strings[S_FWver2old],"0.7.0");	//"This firmware is too old. Version %s is required\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Read24Fx(%d,%d,%d,%d,%d,%d)    (0x%X,0x%X,0x%X,0x%X,0x%X,0x%X)\n",dim,dim2,options,appIDaddr,executiveArea,EEbaseAddr,dim,dim2,options,appIDaddr,executiveArea,EEbaseAddr);
	}
	dim*=2;
	dim2*=2;
	if(dim>0x40000||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(dim2>0x1000||dim2<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	unsigned char *memExec=0;
	executiveArea*=2;
	if(executiveArea) memExec=malloc(executiveArea);
	for(i=0;i<executiveArea;i++) memExec[i]=0xFF;
	size=dim;
	sizeEE=dim2*2;
	memCODE=malloc(dim);		//CODE
	memEE=malloc(dim2*2);			//EEPROM
	for(i=0;i<dim;i++) memCODE[i]=0xFF;
	for(i=0;i<dim2*2;i++) memEE[i]=0xFF;
	if(options&2){					//only if separate config area
		sizeCONFIG=34;
		for(i=0;i<34;i++) memCONFIG[i]=0xFF;
	}
	if(options&1){				//High voltage programming: 3.3V + 1.5V + R drop + margin
		if(!StartHVReg(6)){
			PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
			return;
		}
	}
	else StartHVReg(-1);		//LVP: current limited to (5-0.7-3.6)/10k = 50uA
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD + VPP
	bufferU[j++]=0x5;
	if(!(options&1)){				//LVP: pulse on MCLR
		bufferU[j++]=EN_VPP_VCC;	//VDD
		bufferU[j++]=0x1;
	}
	else PrintMessage(strings[S_HVICSP]); //"High Voltage ICSP\r\n"
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=TX16;
	bufferU[j++]=2;
	bufferU[j++]=0x4D;
	bufferU[j++]=0x43;
	bufferU[j++]=0x48;
	bufferU[j++]=0x51;
	bufferU[j++]=WAIT_T3;			//min 1ms
	bufferU[j++]=EN_VPP_VCC;		//VDD + VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=30000>>8;
	bufferU[j++]=30000&0xff;
	bufferU[j++]=WAIT_T3;			//min 25ms
	//Additional 5 clock cycles upon entering program mode
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(37);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	//Read DeviceID @0xFF0000, DevRev @0xFF0002
	bufferU[j++]=SIX_N;
	bufferU[j++]=4;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x0F;
	bufferU[j++]=0xF0;				//0xFF
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x20;				//MOV XXXX,W6
	bufferU[j++]=0x00;
	bufferU[j++]=0x06;				//0x0000
	bufferU[j++]=0x20;				//MOV #VISI,W7
	bufferU[j++]=0x78;
	bufferU[j++]=0x47;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x0B;
	bufferU[j++]=0xB6;
	bufferU[j++]=REGOUT;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x0B;
	bufferU[j++]=0xB6;
	bufferU[j++]=REGOUT;
	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	int w0=0,w1=0;
	for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w0=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w1=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage("DevID: 0x%04X\r\nDevRev: 0x%04X\r\n",w0,w1);
	PIC_ID(0x100000+w0);
	//Read ApplicationID @ appIDaddr
	bufferU[j++]=SIX;				//MOV XXXX,W0
	bufferU[j++]=0x20;
	bufferU[j++]=0x08;
	bufferU[j++]=0x00;				//0x80
	bufferU[j++]=SIX;				//MOV W0,TABLPAG
	bufferU[j++]=0x88;
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX;				//MOV XXXX,W0
	bufferU[j++]=0x20+((appIDaddr>>12)&0xF);
	bufferU[j++]=(appIDaddr>>4)&0xFF;
	bufferU[j++]=(appIDaddr<<4)&0xFF;
	bufferU[j++]=SIX;				//MOV #VISI,W1
	bufferU[j++]=0x20;
	bufferU[j++]=0x78;
	bufferU[j++]=0x41;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W0],[W1]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x08;
	bufferU[j++]=0x90;
	bufferU[j++]=REGOUT;

	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_N;
	bufferU[j++]=4;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x20;				//MOV 0,W6
	bufferU[j++]=0x00;
	bufferU[j++]=0x06;
	bufferU[j++]=0x20;				//MOV #VISI,W7
	bufferU[j++]=0x78;
	bufferU[j++]=0x47;
	bufferU[j++]=ICSP_NOP;

	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w0=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage("ApplicationID @ 0x80%04X:  0x%04X\r\n",appIDaddr,w0);
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//lettura codice ...
	PrintMessage("   ");
//Read 6 24 bit words packed in 9 16 bit words
//memory address advances by 24 bytes because of alignment
	int High=0;
	for(i=0;i<dim;i+=24){
		if((i>>17)!=High){	//advance page
			bufferU[j++]=SIX;				//MOV XXXX,W0
			bufferU[j++]=0x20;
			bufferU[j++]=(i>>21)&0xFF;
			bufferU[j++]=(i>>13)&0xF0;
			bufferU[j++]=SIX;				//MOV W0,TABLPAG
			bufferU[j++]=0x88;
			bufferU[j++]=0x01;
			bufferU[j++]=0x90;
			bufferU[j++]=SIX;				//GOTO 0x200
			bufferU[j++]=0x04;
			bufferU[j++]=0x02;
			bufferU[j++]=0x00;
			bufferU[j++]=ICSP_NOP;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(2);
			read();
			j=1;
			if(saveLog)WriteLogIO();
			High=i>>17;
		}
		bufferU[j++]=SIX_LONG;				//GOTO 0x200
		bufferU[j++]=0x04;
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0x96;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xDB;
		bufferU[j++]=0xB6;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xD3;
		bufferU[j++]=0xD6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		if(i<dim-16){		//skip last read (dim is never multiple of 6)
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0x96;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xDB;
			bufferU[j++]=0xB6;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xD3;
			bufferU[j++]=0xD6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
		}
		if(i<dim-8){		//skip last read (dim is never multiple of 6)
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0x96;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xDB;
			bufferU[j++]=0xB6;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xD3;
			bufferU[j++]=0xD6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
		}

		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(3);
		read();
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+1]=bufferI[z+1];	//M0
			memCODE[k]=bufferI[z+2];	//L0
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+2]=bufferI[z+2];	//H0
			memCODE[k+6]=bufferI[z+1];	//H1
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+5]=bufferI[z+1];	//M1
			memCODE[k+4]=bufferI[z+2];	//L1
			k+=8;
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+1]=bufferI[z+1];	//M2
			memCODE[k+0]=bufferI[z+2];	//L2
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+2]=bufferI[z+2];	//H2
			memCODE[k+6]=bufferI[z+1];	//H3
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+5]=bufferI[z+1];	//M3
			memCODE[k+4]=bufferI[z+2];	//L3
			k+=8;
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+1]=bufferI[z+1];	//M4
			memCODE[k+0]=bufferI[z+2];	//L4
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+2]=bufferI[z+2];	//H4
			memCODE[k+6]=bufferI[z+1];	//H5
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCODE[k+5]=bufferI[z+1];	//M5
			memCODE[k+4]=bufferI[z+2];	//L5
			k+=8;
		}
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X) \n"
			WriteLogIO();
		}
	}
	PrintMessage("\b\b\b");
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadCodeErr2],dim,k);	//"Errore in lettura area programma, richiesti %d byte, letti %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
//****************** read config area ********************
	if((options&2)){					//config area @ 0xF80000
		if(saveLog)	fprintf(RegFile,"\nCONFIG:\n");
		bufferU[j++]=SIX_N;
		bufferU[j++]=4;
		bufferU[j++]=0x20;				//MOV XXXX,W0
		bufferU[j++]=0x0F;
		bufferU[j++]=0x80;
		bufferU[j++]=0x88;				//MOV W0,TABLPAG
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=0x20;				//MOV XXXX,W6
		bufferU[j++]=0x00;
		bufferU[j++]=0x06;
		bufferU[j++]=0x20;				//MOV #VISI,W7
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(3);
		read();
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],0xF80000,0xF80000,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
		//save 0xF800000 to 0xF80010
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[0]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[4]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[8]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[12]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[16]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[20]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[24]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[28]=bufferI[z+2];	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			memCONFIG[32]=bufferI[z+2];	//Low byte
		}
	}
//****************** read eeprom ********************
	if(dim2){
		if(saveLog)	fprintf(RegFile,"\nEEPROM:\n");
		PrintMessage(strings[S_ReadEE]);		//lettura eeprom ...
		PrintMessage("   ");
		bufferU[j++]=SIX_N;
		bufferU[j++]=0x45;				//append 1 NOP
		bufferU[j++]=0x20;				//MOV #0x7F,W0
		bufferU[j++]=0x07;
		bufferU[j++]=0xF0;
		bufferU[j++]=0x88;				//MOV W0,TABLPAG
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=0x2F;				//MOV #<ADDR[15:0]>,W6   ;0xFE00
		bufferU[j++]=0xE0;
		bufferU[j++]=0x06;
		bufferU[j++]=0x20;				//MOV #VISI,W7
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=0x04;				//GOTO 0x200
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		for(k2=0,i=0;i<dim2*2;i+=4){
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
			if(j>DIMBUF-5||i==dim2*2-4){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(3);
				read();
				j=1;
				for(z=1;z<DIMBUF-2;z++){
					if(bufferI[z]==REGOUT){
						memEE[k2++]=bufferI[z+2];
						memEE[k2++]=bufferI[z+1];
						z+=3;
						k2+=2;		//skip high word
					}
				}
				PrintMessage("\b\b\b%2d%%",i*100/(dim2*2)); fflush(stdout);
				if(saveLog){
					fprintf(RegFile,strings[S_Log7],i,i,k2,k2);	//"i=%d(0x%X), k=%d(0x%X)\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage("\b\b\b");
		if(k2!=dim2*2){
			PrintMessage("\n");
			PrintMessage(strings[S_ReadEEErr],dim2*2,k2);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		}
		else PrintMessage(strings[S_Compl]);
	}
//****************** read executive area ********************
	if(executiveArea){
		PrintMessage(strings[S_Read_EXE_A]);		//lettura executive ...
		PrintMessage("   ");
		if(saveLog)	fprintf(RegFile,"\nExecutive area:\n");
		bufferU[j++]=SIX_N;
		bufferU[j++]=0x45;				//append 1 NOP
		bufferU[j++]=0x20;				//MOV XXXX,W0
		bufferU[j++]=0x08;
		bufferU[j++]=0x00;
		bufferU[j++]=0x88;				//MOV W0,TABLPAG
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=0x20;				//MOV XXXX,W6
		bufferU[j++]=0x00;
		bufferU[j++]=0x06;
		bufferU[j++]=0x20;				//MOV #VISI,W7
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=0x04;				//GOTO 0x200
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(saveLog)WriteLogIO();
		j=1;
		for(i=0,k=0;i<executiveArea;i+=16){
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0x96;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xDB;
			bufferU[j++]=0xB6;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xD3;
			bufferU[j++]=0xD6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0x96;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xDB;
			bufferU[j++]=0xB6;
			bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
			bufferU[j++]=0xBA;
			bufferU[j++]=0xD3;
			bufferU[j++]=0xD6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(3);
			read();
			j=1;
			for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+1]=bufferI[z+1];	//M0
				memExec[k]=bufferI[z+2];	//L0
			}
			for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+2]=bufferI[z+2];	//H0
				memExec[k+6]=bufferI[z+1];	//H1
			}
			for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+5]=bufferI[z+1];	//M1
				memExec[k+4]=bufferI[z+2];	//L1
			}
			for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+9]=bufferI[z+1];	//M2
				memExec[k+8]=bufferI[z+2];	//L2
			}
			for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+10]=bufferI[z+2];	//H2
				memExec[k+14]=bufferI[z+1];	//H3
			}
			for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
			if(z<DIMBUF-2){
				memExec[k+13]=bufferI[z+1];	//M3
				memExec[k+12]=bufferI[z+2];	//L3
			}
			k+=16;
			PrintMessage("\b\b\b%2d%%",i*100/executiveArea); fflush(stdout);
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X) \n"
				WriteLogIO();
			}
		}
		PrintMessage("\b\b\b");
		if(k!=executiveArea){
			PrintMessage("\n");
			PrintMessage(strings[S_ReadCodeErr2],executiveArea,k);	//"Errore in lettura area programma, richiesti %d byte, letti %d\r\n"
		}
		else PrintMessage(strings[S_Compl]);
	}
	PrintMessage("\n");
//****************** exit ********************
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)	WriteLogIO();
	unsigned int stop=GetTickCount();
//****************** visualize ********************
	if(options&2){					//only if separate config area
		PrintMessage(strings[S_ConfigMem]);				//"\r\nMemoria CONFIG:\r\n"
		PrintMessage("0xF80000: FBS = 0x%02X\r\n",memCONFIG[0]);
		PrintMessage("0xF80004: FGS = 0x%02X\r\n",memCONFIG[8]);
		PrintMessage("0xF80006: FOSCSEL = 0x%02X\r\n",memCONFIG[12]);
		PrintMessage("0xF80008: FOSC = 0x%02X\r\n",memCONFIG[16]);
		PrintMessage("0xF8000A: FWDT = 0x%02X\r\n",memCONFIG[20]);
		PrintMessage("0xF8000C: FPOR = 0x%02X\r\n",memCONFIG[24]);
		PrintMessage("0xF8000E: FICD = 0x%02X\r\n",memCONFIG[28]);
		PrintMessage("0xF80010: FDS = 0x%02X\r\n",memCONFIG[32]);
	}
	else{
		PrintMessage("CONFIG1: 0x%04X\r\nCONFIG2: 0x%04X\r\n",(memCODE[dim-3]<<8)+memCODE[dim-4]\
			,(memCODE[dim-7]<<8)+memCODE[dim-8]);
	}
	PrintMessage(strings[S_CodeMem]);	//"\r\nMemoria programma:\r\n"
	char s[256],t[256],v[256];
	int d,valid,empty=1;
	s[0]=0;
	for(i=0;i<dim;i+=COL*2){
		valid=0;
		for(j=i;j<i+COL*2&&j<dim;j+=4){
			d=(memCODE[j+3]<<24)+(memCODE[j+2]<<16)+(memCODE[j+1]<<8)+memCODE[j];
			sprintf(t,"%08X ",d);
			strcat(s,t);
			if(d!=0xffffffff) valid=1;
		}
		if(valid){
			PrintMessage("%06X: %s\r\n",i/2,s);
			empty=0;
		}
		s[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	if(dim2){
		PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
		empty=1;
		v[0]=0;
		for(i=0;i<dim2*2;i+=COL*2){
			valid=0;
			for(j=i;j<i+COL*2&&j<dim2*2;j+=4){
				sprintf(t,"%02X %02X ",memEE[j],memEE[j+1]);
				strcat(s,t);
				sprintf(t,"%c",isprint(memEE[j])?memEE[j]:'.');
				strcat(v,t);
				if(memEE[j]<0xff) valid=1;
				sprintf(t,"%c",isprint(memEE[j+1])?memEE[j+1]:'.');
				strcat(v,t);
				if(memEE[j+1]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s %s\r\n",i/2,s,v);
				empty=0;
			}
			s[0]=0;
			v[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
	}
	if(executiveArea){
		PrintMessage(strings[S_ExeMem]);	//"\r\nmemoria Executive:\r\n"
		s[0]=0;
		empty=1;
		for(i=0;i<executiveArea;i+=COL*2){
			valid=0;
			for(j=i;j<i+COL*2&&j<executiveArea;j+=4){
				d=(memExec[j+3]<<24)+(memExec[j+2]<<16)+(memExec[j+1]<<8)+memExec[j];
				sprintf(t,"%08X ",d);
				strcat(s,t);
				if(d!=0xffffffff) valid=1;
			}
			if(valid){
				PrintMessage("%06X: %s\r\n",0x800000+i/2,s);
				empty=0;
			}
			s[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
	}
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void Write24Fx(int dim,int dim2,int options,int appIDaddr,int rowSize, double wait,int EEbaseAddr,int EraseWord,int CodeWriteWord){
// write 16 bit PIC 24Fxxxx
// deviceID @ 0xFF0000
// dim=program size (16 bit words)
// dim2=eeprom size (in words, area starts at 0x7F0000+EEbaseAddr)
// options:
//	bit 1 = use High voltage MCLR, else low voltage
//	bit 2 = config area @ 0xF80000, else last two program words
// appIDaddr = application ID word lower address (high is 0x80)
// rowSize = row size in instruction words (a row is written altogether)
// EEbaseAddr = address of EEPROM area
// EraseWord = written to NVMCON to erase all memory
// CodeWriteWord = written to NVMCON to program one row of memory
	int k=0,k2=0,z=0,i,j;
	int err=0,errE=0,errC=0;
	if(FWVersion<0x700){
		PrintMessage(strings[S_FWver2old],"0.7.0");	//"This firmware is too old. Version %s is required\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write24Fx(%d,%d,%d,%d,%d,%.1f,%d,%d,%d)    (0x%X,0x%X,0x%X,0x%X,0x%X,%.3f,0x%X,0x%X,0x%X)\n"
			,dim,dim2,options,appIDaddr,rowSize,wait,EEbaseAddr,EraseWord,CodeWriteWord,dim,dim2,options,appIDaddr,rowSize,wait,EEbaseAddr,EraseWord,CodeWriteWord);
	}
	dim*=2;
	dim2*=2;
	if(dim>0x40000||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(dim2>0x1000||dim2<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	j=size;
	if(j%(rowSize*4)){			//grow to an integer number of rows
		size=(j/(rowSize*4)+1)*rowSize*4;
		memCODE=realloc(memCODE,size);
		for(;j<size;j++) memCODE[j]=0xFF;
	}
	if(dim>size) dim=size;
	if(dim2*2>sizeEE) dim2=sizeEE/2;
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	if(options&1){				//High voltage programming: 3.3V + 1.5V + R drop + margin
		if(!StartHVReg(6)){
			PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
			return;
		}
	}
	else StartHVReg(-1);		//LVP: current limited to (5-0.7-3.6)/10k = 50uA
	if(options&2){					//only if separate config area
		PrintMessage("0xF80000: FBS = 0x%02X\r\n",memCONFIG[0]);
		PrintMessage("0xF80004: FGS = 0x%02X\r\n",memCONFIG[8]);
		PrintMessage("0xF80006: FOSCSEL = 0x%02X\r\n",memCONFIG[12]);
		PrintMessage("0xF80008: FOSC = 0x%02X\r\n",memCONFIG[16]);
		PrintMessage("0xF8000A: FWDT = 0x%02X\r\n",memCONFIG[20]);
		PrintMessage("0xF8000C: FPOR = 0x%02X\r\n",memCONFIG[24]);
		PrintMessage("0xF8000E: FICD = 0x%02X\r\n",memCONFIG[28]);
		PrintMessage("0xF80010: FDS = 0x%02X\r\n\r\n",memCONFIG[32]);
	}
	else{
		PrintMessage("CONFIG1: 0x%04X\r\nCONFIG2: 0x%04X\r\n",(memCODE[dim-3]<<8)+memCODE[dim-4]\
			,(memCODE[dim-7]<<8)+memCODE[dim-8]);
		dim-=8;		//Config words are programmed separately
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD + VPP
	bufferU[j++]=0x5;
	if(!(options&1)){				//LVP: pulse on MCLR
		bufferU[j++]=EN_VPP_VCC;	//VDD
		bufferU[j++]=0x1;
	}
	else PrintMessage(strings[S_HVICSP]); //"High Voltage ICSP\r\n"
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=TX16;
	bufferU[j++]=2;
	bufferU[j++]=0x4D;
	bufferU[j++]=0x43;
	bufferU[j++]=0x48;
	bufferU[j++]=0x51;
	bufferU[j++]=WAIT_T3;			//min 1ms
	bufferU[j++]=EN_VPP_VCC;		//VDD + VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=30000>>8;
	bufferU[j++]=30000&0xff;
	bufferU[j++]=WAIT_T3;			//min 25ms
	//Additional 5 clock cycles upon entering program mode
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x4;				//CK=1
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;				//CK=0
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(37);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	//Read DeviceID @0xFF0000, DevRev @0xFF0002
	bufferU[j++]=SIX_N;
	bufferU[j++]=4;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x0F;
	bufferU[j++]=0xF0;				//0xFF
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x20;				//MOV XXXX,W6
	bufferU[j++]=0x00;
	bufferU[j++]=0x06;				//0x0000
	bufferU[j++]=0x20;				//MOV #VISI,W7
	bufferU[j++]=0x78;
	bufferU[j++]=0x47;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x0B;
	bufferU[j++]=0xB6;
	bufferU[j++]=REGOUT;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x0B;
	bufferU[j++]=0xB6;
	bufferU[j++]=REGOUT;
	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	int w0=0,w1=0;
	for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w0=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w1=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage("DevID: 0x%04X\r\nDevRev: 0x%04X\r\n",w0,w1);
	PIC_ID(0x100000+w0);
	//Read ApplicationID @ appIDaddr
	bufferU[j++]=SIX;				//MOV XXXX,W0
	bufferU[j++]=0x20;
	bufferU[j++]=0x08;
	bufferU[j++]=0x00;				//0x80
	bufferU[j++]=SIX;				//MOV W0,TABLPAG
	bufferU[j++]=0x88;
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX;				//MOV XXXX,W0
	bufferU[j++]=0x20+((appIDaddr>>12)&0xF);
	bufferU[j++]=(appIDaddr>>4)&0xFF;
	bufferU[j++]=(appIDaddr<<4)&0xFF;
	bufferU[j++]=SIX;				//MOV #VISI,W1
	bufferU[j++]=0x20;
	bufferU[j++]=0x78;
	bufferU[j++]=0x41;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_LONG;				//TBLRDL [W0],[W1]
	bufferU[j++]=0xBA;
	bufferU[j++]=0x08;
	bufferU[j++]=0x90;
	bufferU[j++]=REGOUT;

	bufferU[j++]=SIX;				//GOTO 0x200
	bufferU[j++]=0x04;
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_N;
	bufferU[j++]=4;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x20;				//MOV 0,W6
	bufferU[j++]=0x00;
	bufferU[j++]=0x06;
	bufferU[j++]=0x20;				//MOV #VISI,W7
	bufferU[j++]=0x78;
	bufferU[j++]=0x47;
	bufferU[j++]=ICSP_NOP;

	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	if(z<DIMBUF-2) w0=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage("ApplicationID @ 0x80%04X:  0x%04X\r\n",appIDaddr,w0);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	bufferU[j++]=SIX_N;
	bufferU[j++]=5;
	bufferU[j++]=0x24;				//MOV XXXX,W10
	bufferU[j++]=(EraseWord>>4)&0xFF;
	bufferU[j++]=((EraseWord<<4)&0xF0)+0xA;
	bufferU[j++]=0x88;				//MOV W10,NVMCON
	bufferU[j++]=0x3B;
	bufferU[j++]=0x0A;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX_LONG;				//TBLWTL W0,[W0]
	bufferU[j++]=0xBB;
	bufferU[j++]=0x08;
	bufferU[j++]=0x00;
	bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
	bufferU[j++]=0xA8;
	bufferU[j++]=0xE7;
	bufferU[j++]=0x61;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=SIX;				//MOV NVMCON,W2
	bufferU[j++]=0x80;
	bufferU[j++]=0x3B;
	bufferU[j++]=0x02;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=SIX;				//MOV W2,VISI
	bufferU[j++]=0x88;
	bufferU[j++]=0x3C;
	bufferU[j++]=0x22;
	bufferU[j++]=REGOUT;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(7);
	read();
	j=1;
	if(saveLog)WriteLogIO();
	for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
	w0=bufferI[z+1]&0x80;
	//Wait for erase completion (max 1s)
	for(i=0;i<100&&w0;i++){
		bufferU[j++]=SIX;				//MOV NVMCON,W2
		bufferU[j++]=0x80;
		bufferU[j++]=0x3B;
		bufferU[j++]=0x02;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX;				//MOV W2,VISI
		bufferU[j++]=0x88;
		bufferU[j++]=0x3C;
		bufferU[j++]=0x22;
		bufferU[j++]=REGOUT;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(10);
		read();
		j=1;
		if(saveLog)WriteLogIO();
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		w0=bufferI[z+1]&0x80;
	}
//****************** prepare write ********************
	bufferU[j++]=SIX_N;
	bufferU[j++]=5;
	bufferU[j++]=0x24;				//MOV XXXX,W10
	bufferU[j++]=(CodeWriteWord>>4)&0xFF;
	bufferU[j++]=((CodeWriteWord<<4)&0xF0)+0xA;
	bufferU[j++]=0x88;				//MOV W10,NVMCON
	bufferU[j++]=0x3B;
	bufferU[j++]=0x0A;
	bufferU[j++]=0x20;				//MOV XXXX,W0
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=0x88;				//MOV W0,TABLPAG
	bufferU[j++]=0x01;
	bufferU[j++]=0x90;
	bufferU[j++]=0x04;				//GOTO 0x200
	bufferU[j++]=0x02;
	bufferU[j++]=0x00;
	bufferU[j++]=ICSP_NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	j=1;
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	if(saveLog)	fprintf(RegFile,"\nWRITE CODE:\n");
	PrintMessage("   ");
//	instruction words are stored in code memory array as follows:
//	L0 M0 H0 FF L1 M1 H1 FF
	int valid,High=0;
	for(i=0,k=0;i<dim;i+=4*4){		//write 4 instruction words
		if(k==0){				//skip row if empty
			for(valid=0;!valid&&i<dim;i+=valid?0:rowSize*4){
				for(k2=0;k2<rowSize*4&&!valid;k2++) if(memCODE[i+k2]<0xFF) valid=1;
			}
			if(i>=dim) break;
		}
		if((i>>17)!=High){	//advance page
			bufferU[j++]=SIX;				//MOV XXXX,W0
			bufferU[j++]=0x20;
			bufferU[j++]=(i>>21)&0xFF;
			bufferU[j++]=(i>>13)&0xF0;
			bufferU[j++]=SIX;				//MOV W0,TABLPAG
			bufferU[j++]=0x88;
			bufferU[j++]=0x01;
			bufferU[j++]=0x90;
			bufferU[j++]=SIX;				//GOTO 0x200
			bufferU[j++]=0x04;
			bufferU[j++]=0x02;
			bufferU[j++]=0x00;
			bufferU[j++]=ICSP_NOP;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(2);
			read();
			j=1;
			if(saveLog)WriteLogIO();
			High=i>>17;
		}
		bufferU[j++]=SIX_N;
		bufferU[j++]=8;
		bufferU[j++]=0x20+((i>>13)&0xF);						//MOV i/2,W7
		bufferU[j++]=(i>>5)&0xFF;
		bufferU[j++]=((i<<3)&0xF0)+7;
		bufferU[j++]=0x20+((memCODE[i+1]>>4)&0xF);				//MOV LSW0,W0
		bufferU[j++]=((memCODE[i+1]<<4)&0xF0)+((memCODE[i]>>4)&0xF);
		bufferU[j++]=(memCODE[i]<<4)&0xF0;
		bufferU[j++]=0x20+((memCODE[i+6]>>4)&0xF);				//MOV MSB1:MSB0,W1
		bufferU[j++]=((memCODE[i+6]<<4)&0xF0)+((memCODE[i+2]>>4)&0xF);
		bufferU[j++]=((memCODE[i+2]<<4)&0xF0)+1;
		bufferU[j++]=0x20+((memCODE[i+5]>>4)&0xF);				//MOV LSW1,W2
		bufferU[j++]=((memCODE[i+5]<<4)&0xF0)+((memCODE[i+4]>>4)&0xF);
		bufferU[j++]=((memCODE[i+4]<<4)&0xF0)+2;
		bufferU[j++]=0x20+((memCODE[i+9]>>4)&0xF);				//MOV LSW2,W3
		bufferU[j++]=((memCODE[i+9]<<4)&0xF0)+((memCODE[i+8]>>4)&0xF);
		bufferU[j++]=((memCODE[i+8]<<4)&0xF0)+3;
		bufferU[j++]=0x20+((memCODE[i+14]>>4)&0xF);				//MOV MSB3:MSB2,W4
		bufferU[j++]=((memCODE[i+14]<<4)&0xF0)+((memCODE[i+10]>>4)&0xF);
		bufferU[j++]=((memCODE[i+10]<<4)&0xF0)+4;
		bufferU[j++]=0x20+((memCODE[i+13]>>4)&0xF);				//MOV LSW3,W5
		bufferU[j++]=((memCODE[i+13]<<4)&0xF0)+((memCODE[i+12]>>4)&0xF);
		bufferU[j++]=((memCODE[i+12]<<4)&0xF0)+5;
		bufferU[j++]=0xEB;				//CLR W6
		bufferU[j++]=0x03;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_N;
		bufferU[j++]=0x88;				//Append 2 NOP
		bufferU[j++]=0xBB;				//TBLWTL [W6++],[W7]
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTH.B [W6++],[W7++]
		bufferU[j++]=0xDB;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTH.B [W6++],[++W7]
		bufferU[j++]=0xEB;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTL [W6++],[W7++]
		bufferU[j++]=0x1B;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTL [W6++],[W7]
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTH.B [W6++],[W7++]
		bufferU[j++]=0xDB;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTH.B [W6++],[++W7]
		bufferU[j++]=0xEB;
		bufferU[j++]=0xB6;
		bufferU[j++]=0xBB;				//TBLWTL [W6++],[W7++]
		bufferU[j++]=0x1B;
		bufferU[j++]=0xB6;
		k++;
		if(k==rowSize/4){	//Write row
			bufferU[j++]=SIX_LONG;				//GOTO 0x200
			bufferU[j++]=0x04;
			bufferU[j++]=0x02;
			bufferU[j++]=0x00;
			bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
			bufferU[j++]=0xA8;
			bufferU[j++]=0xE7;
			bufferU[j++]=0x61;
			//bufferU[j++]=WAIT_T3;
			k=0;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(3);
		if(k==0)msDelay(wait+1);
		read();
		j=1;
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d, k=%d 0=%d\n"
			WriteLogIO();
		}
	}
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** verify code ********************
	PrintMessage(strings[S_CodeV]);	//"Verifica codice ... "
	if(saveLog)	fprintf(RegFile,"\nVERIFY CODE:\n");
	PrintMessage("   ");
//Read 4 24 bit words packed in 6 16 bit words
//memory address advances by 16 bytes because of alignment
	High=0;
	for(i=0;i<dim;i+=16){
		//skip row if empty
		for(valid=0;!valid&&i<dim;i+=16){
			for(k2=0;k2<16&&!valid;k2++) if(memCODE[i+k2]<0xFF) valid=1;
		}
		if(i>=dim) break;
		if((i>>17)!=High){	//advance page
			bufferU[j++]=SIX;				//MOV XXXX,W0
			bufferU[j++]=0x20;
			bufferU[j++]=(i>>21)&0xFF;
			bufferU[j++]=(i>>13)&0xF0;
			bufferU[j++]=SIX;				//MOV W0,TABLPAG
			bufferU[j++]=0x88;
			bufferU[j++]=0x01;
			bufferU[j++]=0x90;
			bufferU[j++]=SIX;				//GOTO 0x200
			bufferU[j++]=0x04;
			bufferU[j++]=0x02;
			bufferU[j++]=0x00;
			bufferU[j++]=ICSP_NOP;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(2);
			read();
			j=1;
			if(saveLog)WriteLogIO();
			High=i>>17;
		}
		bufferU[j++]=SIX;				//MOV i/2,W6
		bufferU[j++]=0x20+((i>>13)&0xF);
		bufferU[j++]=(i>>5)&0xFF;
		bufferU[j++]=((i<<3)&0xF0)+6;
		bufferU[j++]=SIX;				//MOV #VISI,W7
		bufferU[j++]=0x20;
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0x96;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xDB;
		bufferU[j++]=0xB6;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xD3;
		bufferU[j++]=0xD6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0x96;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [W6++],[W7++]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xDB;
		bufferU[j++]=0xB6;
		bufferU[j++]=SIX_LONG;				//TBLRDH.B [++W6],[W7--]
		bufferU[j++]=0xBA;
		bufferU[j++]=0xD3;
		bufferU[j++]=0xD6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(3);
		read();
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+1],bufferI[z+1], (i+1)/2, &err);
			CheckData(memCODE[i],bufferI[z+2], i/2, &err);
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+2],bufferI[z+2], (i+2)/2, &err);
			CheckData(memCODE[i+6],bufferI[z+1], (i+6)/2, &err);
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+5],bufferI[z+1], (i+5)/2, &err);
			CheckData(memCODE[i+4],bufferI[z+2], (i+4)/2, &err);
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+9],bufferI[z+1], (i+9)/2, &err);
			CheckData(memCODE[i+8],bufferI[z+2], (i+8)/2, &err);
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+10],bufferI[z+2], (i+10)/2, &err);
			CheckData(memCODE[i+14],bufferI[z+1], (i+14)/2, &err);
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			CheckData(memCODE[i+13],bufferI[z+1], (i+13)/2, &err);
			CheckData(memCODE[i+12],bufferI[z+2], (i+12)/2, &err);
		}
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log8],i,i,0,0,err);	//"i=%d, k=%d, errori=%d\n"
			WriteLogIO();
		}
		if(err>=max_err) break;
	}
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],err);	//"terminata: %d errori\r\n"
	if(err>=max_err) PrintMessage(strings[S_MaxErr],err);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
//****************** erase, write and verify EEPROM ********************
	if(dim2&&err<max_err){
		//EEPROM @ 0x7FFE00
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
		PrintMessage("   ");
		if(saveLog)	fprintf(RegFile,"\nWRITE EEPROM:\n");
		bufferU[j++]=SIX;				//MOV 0x4050,W10
		bufferU[j++]=0x24;
		bufferU[j++]=0x05;
		bufferU[j++]=0x0A;
		bufferU[j++]=SIX;				//MOV W10,NVMCON
		bufferU[j++]=0x88;
		bufferU[j++]=0x3B;
		bufferU[j++]=0x0A;
		bufferU[j++]=SIX;				//MOV 0x7F,W0
		bufferU[j++]=0x20;
		bufferU[j++]=0x07;
		bufferU[j++]=0xF0;
		bufferU[j++]=SIX;				//MOV W0,TABLPAG
		bufferU[j++]=0x88;
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=SIX;				//MOV 0xFE00,W0
		bufferU[j++]=0x2F;
		bufferU[j++]=0xE0;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX;				//TBLWTL W0,[W0]
		bufferU[j++]=0xBB;
		bufferU[j++]=0x08;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=ICSP_NOP;
		//Erase EEPROM
		bufferU[j++]=SIX;				//BSET NVMCON,#WR
		bufferU[j++]=0xA8;
		bufferU[j++]=0xE7;
		bufferU[j++]=0x61;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=SIX;				//MOV 0x4004,W10
		bufferU[j++]=0x24;
		bufferU[j++]=0x00;
		bufferU[j++]=0x4A;
		bufferU[j++]=SIX;				//MOV W10,NVMCON
		bufferU[j++]=0x88;
		bufferU[j++]=0x3B;
		bufferU[j++]=0x0A;
		bufferU[j++]=SIX;				//GOTO 0x200
		bufferU[j++]=0x04;
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX;				//MOV 0,W7
		bufferU[j++]=0x2F;
		bufferU[j++]=0xE0;
		bufferU[j++]=0x07;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(9);
		read();
		j=1;
		if(saveLog)WriteLogIO();
		//Write EEPROM
		for(k2=0,i=0;i<dim2*2;i+=4){
			if(memEE[i]<0xFF||memEE[i+1]<0xFF){
			bufferU[j++]=SIX;				//MOV i,W7
				bufferU[j++]=0x20+(EEbaseAddr>>12);
				bufferU[j++]=(EEbaseAddr+i/2)>>4;
				bufferU[j++]=((i/2<<4)&0xF0)+7;
				bufferU[j++]=SIX;				//MOV XXXX,W0
				bufferU[j++]=0x20+((memEE[i+1]>>4)&0xF);
				bufferU[j++]=((memEE[i+1]<<4)&0xF0)+((memEE[i]>>4)&0xF);
				bufferU[j++]=(memEE[i]<<4)&0xF0;
				bufferU[j++]=ICSP_NOP;
				bufferU[j++]=SIX_LONG;				//TBLWTL W0,[W7++]
				bufferU[j++]=0xBB;
				bufferU[j++]=0x1B;
				bufferU[j++]=0x80;
				bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
				bufferU[j++]=0xA8;
				bufferU[j++]=0xE7;
				bufferU[j++]=0x61;
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(wait+2);		//write delay
				read();
				j=1;
				PrintMessage("\b\b\b%2d%%",i*100/(dim2*2)); fflush(stdout);
				if(saveLog){
					fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d, k=%d 0=%d\n"
					WriteLogIO();
				}
			}
		}
		//Verify EEPROM
		if(saveLog)	fprintf(RegFile,"\nVERIFY EEPROM:\n");
		bufferU[j++]=SIX;				//MOV 0xFE00,W6
		bufferU[j++]=0x20+(EEbaseAddr>>12);
		bufferU[j++]=EEbaseAddr>>4;
		bufferU[j++]=0x06;
		bufferU[j++]=SIX;				//MOV #VISI,W7
		bufferU[j++]=0x20;
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=SIX;				//GOTO 0x200
		bufferU[j++]=0x04;
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		int errE=0;
		for(i=k2=0;i<dim2*2;i+=4){
			bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
			bufferU[j++]=0xBA;
			bufferU[j++]=0x0B;
			bufferU[j++]=0xB6;
			bufferU[j++]=REGOUT;
			if(j>DIMBUF-6||i==dim2*2-4){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(3);
				read();
				for(z=1;z<DIMBUF-2;z++){
					if(bufferI[z]==REGOUT){
						CheckData(memEE[k2+1],bufferI[z+1],i/2,&errE);
						CheckData(memEE[k2],bufferI[z+2],i/2,&errE);
						z+=3;
						k2+=4;
					}
				}
				PrintMessage("\b\b\b%2d%%",i*100/(dim2*2)); fflush(stdout);
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k2,k2,errE);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],errE);	//"terminata: %d errori \r\n"
		err+=errE;
	}
//****************** write CONFIG ********************
	if((options&2)&&err<max_err){
		PrintMessage(strings[S_ConfigW]);	//"Programmazione CONFIG ..."
		if(saveLog)	fprintf(RegFile,"\nWRITE CONFIG:\n");
		//for(i=memCONFIG.GetSize();i<9;i++) memCONFIG[i]=0xFF;
		//config area @ 0xF80000
		bufferU[j++]=SIX_N;
		bufferU[j++]=6;
		bufferU[j++]=0x20;				//MOV 0xF8,W0
		bufferU[j++]=0x0F;
		bufferU[j++]=0x80;
		bufferU[j++]=0x88;				//MOV W0,TABLPAG
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=0x24;				//MOV 0x4004,W10
		bufferU[j++]=0x00;
		bufferU[j++]=0x4A;
		bufferU[j++]=0x88;				//MOV W10,NVMCON
		bufferU[j++]=0x3B;
		bufferU[j++]=0x0A;
		bufferU[j++]=0x20;				//MOV 0,W7
		bufferU[j++]=0x00;
		bufferU[j++]=0x07;
		bufferU[j++]=0x04;				//GOTO 0x200
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		for(i=0;i<9;i++){
			//Write CONFIG
			bufferU[j++]=SIX;				//MOV XXXX,W0
			bufferU[j++]=0x20;
			bufferU[j++]=(memCONFIG[i*4]>>4)&0xF;
			bufferU[j++]=(memCONFIG[i*4]<<4)&0xF0;
			bufferU[j++]=ICSP_NOP;
			bufferU[j++]=SIX_LONG;				//TBLWTL W0,[W7++]
			bufferU[j++]=0xBB;
			bufferU[j++]=0x1B;
			bufferU[j++]=0x80;
			bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
			bufferU[j++]=0xA8;
			bufferU[j++]=0xE7;
			bufferU[j++]=0x61;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(wait+2);
			read();
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
		//Verify write
		int errC=0;
		bufferU[j++]=SIX;
		bufferU[j++]=0x20;				//MOV XXXX,W6
		bufferU[j++]=0x00;
		bufferU[j++]=0x06;
		bufferU[j++]=SIX;
		bufferU[j++]=0x20;				//MOV #VISI,W7
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(3);
		read();
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],0xF80000,0xF80000,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[0]&bufferI[z+2])CheckData(memCONFIG[0],bufferI[z+2],0xF80000,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[8]&bufferI[z+2])CheckData(memCONFIG[8],bufferI[z+2],0xF80004,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[12]&bufferI[z+2])CheckData(memCONFIG[12],bufferI[z+2],0xF80006,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[16]&bufferI[z+2])CheckData(memCONFIG[16],bufferI[z+2],0xF80008,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[20]&bufferI[z+2])CheckData(memCONFIG[20],bufferI[z+2],0xF8000A,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[24]&bufferI[z+2])CheckData(memCONFIG[24],bufferI[z+2],0xF8000C,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[28]&bufferI[z+2])CheckData(memCONFIG[28],bufferI[z+2],0xF8000E,&errC);	//Low byte
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			if(!memCONFIG[32]&bufferI[z+2])CheckData(memCONFIG[32],bufferI[z+2],0xF80010,&errC);	//Low byte
		}
		PrintMessage(strings[S_ComplErr],errC);	//"terminata: %d errori \r\n"
		err+=errC;
	}
	else if(!(options&2)&&err<max_err){		//Config words in the last 2 locations
		PrintMessage(strings[S_ConfigW]);	//"Write CONFIG ..."
		if(saveLog)	fprintf(RegFile,"\nWRITE CONFIG:\n");
		bufferU[j++]=SIX_N;
		bufferU[j++]=6;
		bufferU[j++]=0x20;				//MOV high(dim/2),W0
		bufferU[j++]=(dim>>21)&0xFF;
		bufferU[j++]=(dim>>13)&0xF0;
		bufferU[j++]=0x88;				//MOV W0,TABLPAG
		bufferU[j++]=0x01;
		bufferU[j++]=0x90;
		bufferU[j++]=0x24;				//MOV 0x4003,W10
		bufferU[j++]=0x00;
		bufferU[j++]=0x3A;
		bufferU[j++]=0x88;				//MOV W10,NVMCON
		bufferU[j++]=0x3B;
		bufferU[j++]=0x0A;
		bufferU[j++]=0x20+((dim>>13)&0x0F);	//MOV dim/2,W7
		bufferU[j++]=(dim>>5)&0xFF;
		bufferU[j++]=((dim<<3)&0xF0)+7;
		bufferU[j++]=0x04;				//GOTO 0x200
		bufferU[j++]=0x02;
		bufferU[j++]=0x00;
		bufferU[j++]=ICSP_NOP;
		//Write CONFIG
		bufferU[j++]=SIX;				//MOV XXXX,W0
		bufferU[j++]=0x20+((memCODE[dim+1]>>4)&0xF);
		bufferU[j++]=((memCODE[dim+1]<<4)&0xF0)+((memCODE[dim]>>4)&0xF);
		bufferU[j++]=(memCODE[dim]<<4)&0xF0;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLWTL W0,[W7++]
		bufferU[j++]=0xBB;
		bufferU[j++]=0x1B;
		bufferU[j++]=0x80;
		bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
		bufferU[j++]=0xA8;
		bufferU[j++]=0xE7;
		bufferU[j++]=0x61;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=SIX;				//MOV XXXX,W0
		bufferU[j++]=0x20+((memCODE[dim+5]>>4)&0xF);
		bufferU[j++]=((memCODE[dim+5]<<4)&0xF0)+((memCODE[dim+4]>>4)&0xF);
		bufferU[j++]=(memCODE[dim+4]<<4)&0xF0;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLWTL W0,[W7++]
		bufferU[j++]=0xBB;
		bufferU[j++]=0x1B;
		bufferU[j++]=0x80;
		bufferU[j++]=SIX_LONG;				//BSET NVMCON,#WR
		bufferU[j++]=0xA8;
		bufferU[j++]=0xE7;
		bufferU[j++]=0x61;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(10);
		read();
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
		//Verify write
		int errC=0;
		bufferU[j++]=SIX;
		bufferU[j++]=0x20+((dim>>13)&0x0F);	//MOV dim/2,W6
		bufferU[j++]=(dim>>5)&0xFF;
		bufferU[j++]=((dim<<3)&0xF0)+6;
		bufferU[j++]=SIX;
		bufferU[j++]=0x20;				//MOV #VISI,W7
		bufferU[j++]=0x78;
		bufferU[j++]=0x47;
		bufferU[j++]=ICSP_NOP;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=SIX_LONG;				//TBLRDL [W6++],[W7]
		bufferU[j++]=0xBA;
		bufferU[j++]=0x0B;
		bufferU[j++]=0xB6;
		bufferU[j++]=REGOUT;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],dim/2,dim/2,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
		for(z=1;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			i=(memCODE[dim+1]<<8)+memCODE[dim];
			j=(bufferI[z+1]<<8)+bufferI[z+2];
			if(!i&j)CheckData(i,j,dim/2,&errC);		//Low word
		}
		for(z+=2;bufferI[z]!=REGOUT&&z<DIMBUF;z++);
		if(z<DIMBUF-2){
			i=(memCODE[dim+5]<<8)+memCODE[dim+4];
			j=(bufferI[z+1]<<8)+bufferI[z+2];
			if(!i&j)CheckData(i,j,dim/2+2,&errC);	//Low word
		}
		PrintMessage(strings[S_ComplErr],errC);	//"terminata: %d errori \r\n"
		err+=errC;
		j=1;
	}
//****************** exit ********************
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

