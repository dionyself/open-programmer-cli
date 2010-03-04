/*
 * progP18F.c - algorithms to program the PIC18 family of microcontrollers
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


extern int size,saveLog;
extern char** strings;
extern int fd;
extern int DIMBUF,saveLog,programID,MinRit,load_osccal,load_BKosccal,usa_osccal,usa_BKosccal;
extern int load_calibword,max_errori;
extern int lock,fuse,fuse_h,fuse_x;
extern FILE* RegFile;
extern char LogFileName[256];
extern WORD *dati_hex;
extern int size,sizeEE;
extern unsigned char *memCODE,*memEE,memID[8],memCONFIG[14];
extern struct hiddev_report_info rep_info_i,rep_info_u;
extern struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
extern void msDelay(double delay);
extern void WriteLogIO();
extern void PIC_ID(int id);
extern void StartHVReg(double V);
extern DWORD GetTickCount();

void Read18Fx(int dim,int dim2){
// read 16 bit PIC 18Fxxxx
// dim=program size 	dim2=eeprom size
// vdd before vpp
	int k=0,k2=0,z=0,i,j;
	if(dim>0x1FFFFF||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(dim2>0x800||dim2<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Read18F(%d,%d)    (0x%X,0x%X)\n",dim,dim2,dim,dim2);
	}
	size=dim;
	sizeEE=dim2;
	memCODE=malloc(dim);		//CODE
	memEE=malloc(dim2);			//EEPROM
	for(j=0;j<8;j++) memID[j]=0xFF;
	for(j=0;j<14;j++) memCONFIG[j]=0xFF;
	PrintMessage(strings[S_StartRead]);	//"Inizio lettura...\r\n"
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//3F
	bufferU[j++]=0x3F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRU
	bufferU[j++]=0xF8;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//FF
	bufferU[j++]=0xFF;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRH
	bufferU[j++]=0xF7;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//FE
	bufferU[j++]=0xFE;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRL
	bufferU[j++]=0xF6;
	bufferU[j++]=TBLR_INC_N;		//DevID1-2	0x3FFFFE-F
	bufferU[j++]=2;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRU
	bufferU[j++]=0xF8;			//TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH
	bufferU[j++]=0xF7;			//TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRL
	bufferU[j++]=0xF6;			//TBLPTRL
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
	if(z<DIMBUF-3){
		PrintMessage(strings[S_DevID2],bufferI[z+3],bufferI[z+2]);	//"DevID: 0x%02X%02X\r\n"
		PIC_ID(0x10000+bufferI[z+2]+(bufferI[z+3]<<8));
	}
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//lettura codice ...
	PrintMessage("   ");
	for(i=0,j=1;i<dim;i+=DIMBUF-4){
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(bufferI[1]==TBLR_INC_N){
			for(z=3;z<bufferI[2]+3&&z<DIMBUF;z++) memCODE[k++]=bufferI[z];
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
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//TBLPTRU	ID 0x200000
	bufferU[j++]=0x20;			//TBLPTRU	ID 0x200000
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//TBLPTRU
	bufferU[j++]=0xF8;			//TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH
	bufferU[j++]=0xF7;			//TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRL
	bufferU[j++]=0xF6;			//TBLPTRL
	bufferU[j++]=TBLR_INC_N;
	bufferU[j++]=8;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//TBLPTRU	CONFIG 0x300000
	bufferU[j++]=0x30;			//TBLPTRU	CONFIG 0x300000
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//TBLPTRU
	bufferU[j++]=0xF8;			//TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH
	bufferU[j++]=0xF7;			//TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRL
	bufferU[j++]=0xF6;			//TBLPTRL
	bufferU[j++]=TBLR_INC_N;
	bufferU[j++]=14;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(8);
	read();
	for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF-28;z++);
	if(z<DIMBUF-28){
		for(i=0;i<8;i++) memID[k2++]=bufferI[z+i+2];
		for(;i<14+8;i++) memCONFIG[-8+k2++]=bufferI[z+i+8];
	}
	j=1;
	if(saveLog){
		fprintf(RegFile,strings[S_Log7],i,i,k2,k2);	//"i=%d, k2=%d\n"
		WriteLogIO();
	}
	if(k2!=22){
		PrintMessage(strings[S_ReadConfigErr],22,k2);	//"Errore in lettura area configurazione, richiesti %d byte, letti %d\r\n"
	}
//****************** read eeprom ********************
	if(dim2){
		PrintMessage(strings[S_ReadEE]);		//lettura eeprom ...
		PrintMessage("   ");
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9E;				//EEPGD=0
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;				//CFGS=0
		bufferU[j++]=0xA6;
		for(k2=0,i=0;i<dim2;i++){
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;
			bufferU[j++]=i&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//ADDR
			bufferU[j++]=0xA9;			//ADDR
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//ADDRH
			bufferU[j++]=0xAA;			//ADDRH
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x80;			//RD=1 :Read
			bufferU[j++]=0xA6;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x50;			//MOVF EEDATA,W
			bufferU[j++]=0xA8;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//MOVWF TABLAT
			bufferU[j++]=0xF5;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x00;			//NOP
			bufferU[j++]=0x00;
			bufferU[j++]=SHIFT_TABLAT;
			if(j>DIMBUF-26||i==dim2-1){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(10);
				read();
				for(z=1;z<DIMBUF-1;z++){
					if(bufferI[z]==SHIFT_TABLAT){
						memEE[k2++]=bufferI[z+1];
						z+=8;
					}
				}
				PrintMessage("\b\b\b%2d%%",i*100/dim2); fflush(stdout);
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log7],i,i,k2,k2);	//"i=%d, k2=%d 0=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage("\b\b\b");
		if(k2!=dim2){
			PrintMessage("\n");
			PrintMessage(strings[S_ReadEEErr],dim2,k2);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		}
		else PrintMessage(strings[S_Compl]);
	}
	PrintMessage("\n");
//****************** exit ********************
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=1;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
//****************** visualize ********************
	for(i=0;i<8;i+=2){
		PrintMessage(strings[S_ChipID2],i,memID[i],i+1,memID[i+1]);	//"ID%d: 0x%02X   ID%d: 0x%02X\r\n"
	}
	for(i=0;i<7;i++){
		PrintMessage(strings[S_ConfigWordH],i+1,memCONFIG[i*2+1]);	//"CONFIG%dH: 0x%02X\t"
		PrintMessage(strings[S_ConfigWordL],i+1,memCONFIG[i*2]);	//"CONFIG%dL: 0x%02X\r\n"
	}
	PrintMessage(strings[S_CodeMem]);	//"\r\nMemoria programma:\r\n"
	char s[256],t[256],v[256];
	s[0]=0;
	int valid=0,empty=1;
	for(i=0;i<dim;i+=COL*2){
		valid=0;
		for(j=i;j<i+COL*2&&j<dim;j++){
			sprintf(t,"%02X ",memCODE[j]);
			strcat(s,t);
			if(memCODE[j]<0xff) valid=1;
		}
		if(valid){
			PrintMessage("%04X: %s\r\n",i,s);
			empty=0;
		}
		s[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	if(dim2){
		DisplayEE();	//visualize
	}
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}


void Write18Fx(int dim,int dim2,int wbuf,int eraseW1,int eraseW2,int EEalgo)
// write 16 bit PIC 18Fxxxx
// dim=program size 	dim2=eeprom size	wbuf=write buffer size {<=64}
// eraseW1=erase word @3C0005	(not used if > 0x10000)
// eraseW2=erase word @3C0004	(not used if > 0x10000)
// EEalgo=eeprom write algoritm: 0->new, 1->old (with sequence 55 AA)
// vdd before vpp
{
	int k=0,z=0,i,j;
	int errori=0;
	if(dim>0x1FFFFF||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(dim2>0x800||dim2<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	if(wbuf>64){
		PrintMessage(strings[S_WbufLim]);	//"Dimensione buffer scrittura oltre i limiti\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write18F(%d,%d,%d)    (0x%X,0x%X,0x%X)\n",dim,dim2,wbuf,dim,dim2,wbuf);
	}
	if(dim>size) dim=size;
	if(dim2>sizeEE) dim2=sizeEE;
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=5100>>8;
	bufferU[j++]=5100&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//3F
	bufferU[j++]=0x3F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRU
	bufferU[j++]=0xF8;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//FF
	bufferU[j++]=0xFF;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRH
	bufferU[j++]=0xF7;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//FE
	bufferU[j++]=0xFE;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRL
	bufferU[j++]=0xF6;
	bufferU[j++]=TBLR_INC_N;		//DevID1-2	0x3FFFFE-F
	bufferU[j++]=2;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
	if(z<DIMBUF-3){
		PrintMessage(strings[S_DevID2],bufferI[z+3],bufferI[z+2]);	//"DevID: 0x%02X%02X\r\n"
		PIC_ID(0x10000+bufferI[z+2]+(bufferI[z+3]<<8));
	}
	j=1;
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x0E;			//3C
	bufferU[j++]=0x3C;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6E;			//-> TBLPTRU
	bufferU[j++]=0xF8;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH=0
	bufferU[j++]=0xF7;
	if(eraseW1<0x10000){
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//05
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//-> TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;		// eraseW1@3C0005
		bufferU[j++]=(eraseW1>>8)&0xFF; 	//0x3F;
		bufferU[j++]=eraseW1&0xFF; 		//0x3F;
	}
	if(eraseW2<0x10000){
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//04
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//-> TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;		// eraseW2@3C0004
		bufferU[j++]=(eraseW2>>8)&0xFF; 	//0x8F;
		bufferU[j++]=eraseW2&0xFF; 		//0x8F;
	}
	bufferU[j++]=CORE_INS;		//NOP
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=CORE_INS;		//NOP
	bufferU[j++]=0x00;
	bufferU[j++]=0x00;
	bufferU[j++]=WAIT_T3;		//ritardo cancellazione
//****************** prepare write ********************
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x8E;			//EEPGD=1
	bufferU[j++]=0xA6;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x9C;			//CFCGS=0
	bufferU[j++]=0xA6;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRU
	bufferU[j++]=0xF8;			//TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH
	bufferU[j++]=0xF7;			//TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRL
	bufferU[j++]=0xF6;			//TBLPTRL
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(6);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	int ww,dimx,st;
	ww=size;
	for(dimx=dim-1;dimx&&memCODE[dimx]==0xFF;dimx--);
	if(!dimx) PrintMessage(strings[S_NoCode2]);	//"Niente da scrivere"
	dimx++;
	if(dimx%wbuf) dimx+=wbuf-dimx%wbuf;		//arrotonda a wbuf
	if(size<dimx){
		memCODE=realloc(memCODE,dimx);
		for(i=size;i<dimx;i++) memCODE[i]=0xFF;
	}
	if(saveLog){
		fprintf(RegFile,strings[S_Log5],dim,dim,dimx,dimx,dimx/wbuf);	//"dim=%d(0x%X), dimx=%d(0x%X), dimx/wbuf=%d \n\n"
	}
	for(i=0,j=1,st=0;i<dimx;){
		bufferU[j++]=TBLW_INC_N;
		if(st){
			ww=wbuf/2-1-ww;
			st=0;
		}
		else if(j+wbuf+5>DIMBUF){
			ww=(DIMBUF-1-5-j)/2;
			st=1;
		}
		else{
			ww=wbuf/2-1;
			st=0;
		}
		bufferU[j++]=ww;
		for(k=0;k<ww;k++){
			bufferU[j++]=memCODE[i+1];
			bufferU[j++]=memCODE[i];
			i+=2;
		}
		if(!st){
			bufferU[j++]=TBLW_PROG_INC;
			bufferU[j++]=memCODE[i+1];
			bufferU[j++]=memCODE[i];
			bufferU[j++]=1000>>8;
			bufferU[j++]=1000&0xFF;
			i+=2;
		}
		if(j>=DIMBUF-1-6||i>=dimx-1){
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(DIMBUF/wbuf+1);
			read();
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			j=1;
			if(saveLog){
				fprintf(strings[S_Log7],i,i,0,0);	//"i=%d, k=%d 0=%d\n"
				WriteLogIO();
			}
		}
	}
	PrintMessage("\b\b\b");
	if(i!=dimx){
		PrintMessage(strings[S_CodeWError4],i,dimx);	//"Errore in scrittura area programma, richiesti %d byte, scritti %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** write ID ********************
	if(programID){
		PrintMessage(strings[S_IDW]);	//"Scrittura ID ... "
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8E;
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	ID 0x200000
		bufferU[j++]=0x20;			//TBLPTRU	ID 0x200000
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRH
		bufferU[j++]=0xF7;			//TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRL
		bufferU[j++]=0xF6;			//TBLPTRL
		bufferU[j++]=TBLW_INC_N;
		bufferU[j++]=3;
		for(i=0;i<3;i++){
			bufferU[j++]=memID[i*2+1];
			bufferU[j++]=memID[i*2];
		}
		bufferU[j++]=TBLW_PROG;
		bufferU[j++]=memID[i*2+1];
		bufferU[j++]=memID[i*2];
		bufferU[j++]=1000>>8;
		bufferU[j++]=1000&0xFF;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(5);
		read();
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d, k2=%d 0=%d\n"
			WriteLogIO();
		}
		PrintMessage(strings[S_Compl]);	//"terminata\r\n"
	}
//****************** write and verify EEPROM ********************
	if(dim2){
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
		PrintMessage("   ");
		int erroriEE=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9E;			//EEPGD=0
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;			//CFGS=0
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x84;			//WREN=1
		bufferU[j++]=0xA6;
		for(i=0;i<dim2&&errori<=max_errori;i++){
			if(memEE[i]!=0xFF){
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;
				bufferU[j++]=i&0xFF;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;
				bufferU[j++]=0xA9;			//ADDR
				if(EEalgo==0){
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;
					bufferU[j++]=(i>>8)&0xFF;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;
					bufferU[j++]=0xAA;		//ADDRH
				}
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;
				bufferU[j++]=memEE[i];
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;
				bufferU[j++]=0xA8;			//EEDATA
				if(EEalgo==1){				//memory unlock
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;
					bufferU[j++]=0x55;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;
					bufferU[j++]=0xA7;			//EECON2
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;
					bufferU[j++]=0xAA;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;
					bufferU[j++]=0xA7;			//EECON2
				}
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x82;
				bufferU[j++]=0xA6;			//WR=1
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x00;
				bufferU[j++]=0x00;			//NOP
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x00;
				bufferU[j++]=0x00;			//NOP
				bufferU[j++]=WAIT_T3;		//ritardo scrittura
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x80;			//RD=1
				bufferU[j++]=0xA6;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x50;			//MOVF EEDATA,w
				bufferU[j++]=0xA8;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;			//MOVWF TABLAT
				bufferU[j++]=0xF5;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x00;			//NOP
				bufferU[j++]=0x00;
				bufferU[j++]=SHIFT_TABLAT;
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(8);
				read();
				PrintMessage("\b\b\b%2d%%",i*100/dim2); fflush(stdout);
				j=1;
				for(z=DIMBUF-1;z&&bufferI[z]!=SHIFT_TABLAT;z--);
				if(z&&memEE[i]!=bufferI[z+1]) erroriEE++;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,erroriEE);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],erroriEE);	//"terminata: %d errori \r\n"
		errori+=erroriEE;
	}
//****************** verify code ********************
	PrintMessage(strings[S_CodeV]);	//"Verifica codice ... "
	PrintMessage("   ");
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x8E;			//EEPGD=1
	bufferU[j++]=0xA6;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x9C;			//CFCGS=0
	bufferU[j++]=0xA6;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRU
	bufferU[j++]=0xF8;			//TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRH
	bufferU[j++]=0xF7;			//TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//TBLPTRL
	bufferU[j++]=0xF6;			//TBLPTRL
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	for(i=0,j=1,k=0;i<dimx;i+=DIMBUF-4){
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=i<dimx-(DIMBUF-4)?DIMBUF-4:dimx-i;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(bufferI[1]==TBLR_INC_N){
			for(z=0;z<bufferI[2]&&z<DIMBUF;z++){
				if(memCODE[i+z]!=bufferI[z+3]){
					PrintMessage(strings[S_CodeVError],i+z,i+z,memCODE[i+z],bufferI[z+3]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
					errori++;
				}
				k++;
			}
		}
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
			WriteLogIO();
		}
		if(errori>=max_errori) break;
	}
	PrintMessage("\b\b\b");
	if(k<dimx){
		PrintMessage(strings[S_CodeVError2],dimx,k);	//"Errore in verifica area programma, richiesti %d byte, letti %d\r\n"
	}
	PrintMessage(strings[S_ComplErr],errori);	//"terminata: %d errori\r\n"
	if(errori>=max_errori) PrintMessage(strings[S_MaxErr]);	//"Raggiunto il numero massimo di errori, programmazione terminata"
//****************** verify ID ********************
	if(programID&&errori<max_errori){
		PrintMessage(strings[S_IDV]);	//"Verifica ID ... "
		int erroriID=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	ID 0x200000
		bufferU[j++]=0x20;			//TBLPTRU	ID 0x200000
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRH
		bufferU[j++]=0xF7;			//TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRL
		bufferU[j++]=0xF6;			//TBLPTRL
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=8;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		for(z=0;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
		for(i=0;i<8;i++) if(memID[i]!=0xFF&&memID[i]!=bufferI[z+i+2]) erroriID++;
		PrintMessage(strings[S_ComplErr],erroriID);	//"terminata: %d errori\r\n"
		errori+=erroriID;
		if(errori>=max_errori) PrintMessage(strings[S_MaxErr]);	//"Raggiunto il numero massimo di errori, programmazione terminata"
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log8],i,i,0,0,errori);	//"i=%d, k2=%d 0=%d\n"
			WriteLogIO();
		}
	}
//****************** write CONFIG ********************
	if(errori<max_errori){
		PrintMessage(strings[S_ConfigW]);	//"Programmazione CONFIG ..."
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8E;
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8C;
		bufferU[j++]=0xA6;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//CONFIG 0x300000
		bufferU[j++]=0x30;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRH
		bufferU[j++]=0xF7;			//TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRL
		bufferU[j++]=0xF6;			//TBLPTRL
		for(i=0;i<14;){
			bufferU[j++]=TBLW_PROG;
			bufferU[j++]=0;
			bufferU[j++]=memCONFIG[i++];
			bufferU[j++]=1000>>8;
			bufferU[j++]=1000&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x2A;			//INCF
			bufferU[j++]=0xF6;			//TBLPTRL
			bufferU[j++]=TBLW_PROG;
			bufferU[j++]=memCONFIG[i++];
			bufferU[j++]=0;
			bufferU[j++]=1000>>8;
			bufferU[j++]=1000&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x2A;			//INCF
			bufferU[j++]=0xF6;			//TBLPTRL
			if(j>DIMBUF-17||i==13){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(5);
				read();
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d, k2=%d 0=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** verify CONFIG ********************
		PrintMessage(strings[S_ConfigV]);	//"Verifica CONFIG ... "
		int erroriC=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	CONFIG 0x300000
		bufferU[j++]=0x30;			//TBLPTRU	CONFIG 0x300000
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRH
		bufferU[j++]=0xF7;			//TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//TBLPTRL
		bufferU[j++]=0xF6;			//TBLPTRL
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=14;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF-16;z++);
		if(z<DIMBUF-16){
			for(i=0;i<14;i++) if(memCONFIG[i]!=0xFF&&memCONFIG[i]!=bufferI[z+i+2]) erroriC++;
		}
		PrintMessage(strings[S_ComplErr],erroriC);	//"terminata: %d errori\r\n"
		errori+=erroriC;
		if(errori>=max_errori) PrintMessage(strings[S_MaxErr]);	//"Raggiunto il numero massimo di errori, programmazione terminata"
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log8],i,i,0,0,errori);	//"i=%d, k=%d, errori=%d\n"
			WriteLogIO();
		}
	}
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=1;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

