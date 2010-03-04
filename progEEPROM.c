/*
 * progEEPROM.c - algorithms to program various EEPROM types
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
extern void StartHVReg();
extern DWORD GetTickCount();
extern void DisplayEE();

void ReadI2C(int dim,int addr)
// read I2C memories
// dim=size in bytes		addr=0: 1 byte address     =1: 2 byte address
{
	int k=0,z=0,i,j;
	if(dim>0x30000||dim<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"ReadI2C(%d,%d)    (0x%X,0x%X)\n",dim,addr,dim,addr);
	}
	sizeEE=dim;
	memEE=malloc(dim);			//EEPROM
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=I2C_INIT;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** read ********************
	PrintMessage(strings[S_ReadEE]);		//lettura EE ...
	PrintMessage("   ");
	for(i=0,j=1;i<dim;i+=DIMBUF-4){
		if(!addr){									//1 byte address
			bufferU[j++]=I2C_READ;
			bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
			bufferU[j++]=0xA0+(i>>7&0x0E);
			bufferU[j++]=i&0xFF;
		}
		else{										//2 bytes address
			bufferU[j++]=I2C_READ2;
			bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
			bufferU[j++]=0xA0+(i>0xFFFF?8:0);
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=i&0xFF;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(8);
		read();
		if((bufferI[1]==I2C_READ||bufferI[1]==I2C_READ2)&&bufferI[2]<0xFA){
			for(z=3;z<bufferI[2]+3&&z<DIMBUF;z++) memEE[k++]=bufferI[z];
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
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		sizeEE=k;
	}
	else PrintMessage(strings[S_Compl]);
//****************** exit ********************
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	DisplayEE();	//visualize
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void WriteI2C(int dim,int addr,int page, float wait)
// write I2C memories
// dim=size in bytes		addr=0: 1 byte address     =1: 2 byte address
// page=page size		wait=write delay
{
	int k=0,z=0,i,j;
	int errori=0;
	if(dim>0x30000||dim<0){
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
		fprintf(RegFile,"WriteI2C(%d,%d,%d,%f)    (0x%X,0x%X)\n",dim,addr,page,wait,dim,addr);
	}
	if(dim>sizeEE){
		i=sizeEE;
		memEE=realloc(memEE,dim);
		for(;i<dim;i++) memEE[i]=0xFF;
		sizeEE=dim;
	}
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=I2C_INIT;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** write ********************
	PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
	PrintMessage("   ");
	for(;page>=DIMBUF-6;page>>=1);
	for(i=0,j=1;i<dim;i+=page){
		bufferU[j++]=I2C_WRITE;
		if(!addr){									//1 byte address
			bufferU[j++]=page;
			bufferU[j++]=0xA0+(i>>7&0x0E);
			bufferU[j++]=i&0xFF;
		}
		else{										//2 bytes address
			bufferU[j++]=page+1;
			bufferU[j++]=0xA0+(i>0xFFFF?8:0);
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=i&0xFF;
		}
		for(k=0;k<page;k++) bufferU[j++]=memEE[i+k];
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(wait+1);
		read();
		if(bufferI[1]!=I2C_WRITE||bufferI[2]>=0xFA) i=dim+10;
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
	}
	msDelay(wait+1);
	msDelay(20);
	PrintMessage("\b\b\b");
	if(i!=dim){
		PrintMessage(strings[S_CodeWError4],i,dim);	//"Errore in scrittura area programma, richiesti %d byte, scritti %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** verify EEPROM ********************
	PrintMessage(strings[S_EEV]);	//"Verifica EEPROM ... "
	PrintMessage("   ");
	k=0;
	for(i=0,j=1;i<dim;i+=DIMBUF-4){
		if(!addr){									//1 byte address
			bufferU[j++]=I2C_READ;
			bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
			bufferU[j++]=0xA0+(i>>7&0x0E);
			bufferU[j++]=i&0xFF;
		}
		else{										//2 bytes address
			bufferU[j++]=I2C_READ2;
			bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
			bufferU[j++]=0xA0+(i>0xFFFF?8:0);
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=i&0xFF;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(8);
		read();
		if((bufferI[1]==I2C_READ||bufferI[1]==I2C_READ2)&&bufferI[2]<0xFA){
			for(z=3;z<bufferI[2]+3&&z<DIMBUF;z++){
				if(memEE[k++]!=bufferI[z]){
					PrintMessage("\n");
					PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k-1],bufferI[z]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
					errori++;
				}
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
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
	}
	PrintMessage(strings[S_ComplErr],errori);	//"terminata: %d errori\r\n"
//****************** exit ********************
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

#define PRE 0x08	//RB3
#define S 0x10		//RB4
#define W 0x20		//RB5
#define ORG 0x20	//RB5

void Read93x(int dim,int na,int options)
// read 93Sx6 uW memories
// dim=size in bytes
// na=address bits
// options=0: x16 organization     =1: x8 organization
{
	int k=0,z=0,i,j;
	if(dim>0x3000||dim<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(na>13) na=13;
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Read93x(%d,%d,%d)    (0x%X,0x%X)\n",dim,na,options,dim,na);
	}
	sizeEE=dim;
	memEE=malloc(dim);			//EEPROM
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=uW_INIT;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG:S;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;				//READ
	bufferU[j++]=0xC0;				//110aaaaa aaax0000 
	bufferU[j++]=0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** read ********************
	PrintMessage(strings[S_ReadEE]);		//lettura EE ...
	PrintMessage("   ");
	int n=(DIMBUF-2);
	if(n>30) n=30;				//max 240 bit = 30 Byte
	for(i=0,j=1;i<dim;i+=n){
		bufferU[j++]=uWRX;
		bufferU[j++]=i<(dim-n)?n*8:(dim-i)*8;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(bufferI[1]==uWRX){
			for(j=3;j<bufferI[2]/8+3&&j<DIMBUF;j++){
				memEE[k++]=bufferI[j];
			}
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
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		sizeEE=k;
	}
	else PrintMessage(strings[S_Compl]);
	if(options==0){
		for(i=0;i<dim;i+=2){		//swap bytes
			k=memEE[i];
			memEE[i]=memEE[i+1];
			memEE[i+1]=k;
		}
	}
//****************** exit ********************
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	DisplayEE();	//visualize
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void Write93Sx(int dim,int na,int page, double wait)
// write 93Sx6 uW memories
// dim=size in bytes
// na=address bits
// page=page size (bytes)
// wait=write delay
{
	int k=0,z=0,i,j;
	int errori=0;
	if(dim>0x1000||dim<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	if(na>13) na=13;
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write93Sx(%d,%d,%d,%f)    (0x%X,0x%X)\n",dim,na,page,wait,dim,na);
	}
	if(dim>sizeEE){
		i=sizeEE;
		memEE=realloc(memEE,dim);
		for(;i<dim;i++) memEE[i]=0xFF;
		sizeEE=dim;
	}
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=uW_INIT;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=S+W+PRE;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;
	bufferU[j++]=0x98;				//100 11xxx Prot. reg. enable
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=W+PRE;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=S+W+PRE;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;
	bufferU[j++]=0xFF;				//111 11111111 Prot. reg. clear
	bufferU[j++]=0xF0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=W+PRE;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=S+W;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;
	bufferU[j++]=0x98;				//100 11xxx write enable
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=W+PRE;
	bufferU[j++]=0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** write ********************
	PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
	PrintMessage("   ");
	int addr=0;
	for(i=0,j=1;i<dim;i+=page,addr+=(0x10000>>na)*page/2){
		bufferU[j++]=EXT_PORT;
		bufferU[j++]=S+W;
		bufferU[j++]=0;
		bufferU[j++]=uWTX;
		bufferU[j++]=3;
		bufferU[j++]=0xE0;			//111aaaaa aaa(a) D page write
		bufferU[j++]=uWTX;
		bufferU[j++]=na;
		bufferU[j++]=addr>>8;
		if(na>8) bufferU[j++]=addr&0xFF;
		bufferU[j++]=uWTX;
		bufferU[j++]=8*page;
		for(k=0;k<page;k+=2){
			bufferU[j++]=memEE[i+k+1];
			bufferU[j++]=memEE[i+k];
		}
		bufferU[j++]=EXT_PORT;
		bufferU[j++]=W;
		bufferU[j++]=0;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(wait+1);
		read();
		if(bufferI[2]!=uWTX||bufferI[3]>=0xFA) i=dim+10;
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
	}
	msDelay(wait+1);
	PrintMessage("\b\b\b");
	if(i!=dim){
		PrintMessage(strings[S_CodeWError4],i,dim);	//"Errore in scrittura area programma, richiesti %d byte, scritti %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** verify EEPROM ********************
	PrintMessage(strings[S_EEV]);	//"Verifica EEPROM ... "
	PrintMessage("   ");
	bufferU[0]=0;
	j=1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=S;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;				//READ (16bit)
	bufferU[j++]=0xC0;				//110aaaaa aaax0000
	bufferU[j++]=0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	k=0;
	int n=(DIMBUF-2);
	if(n>30) n=30;	//max 240 bit = 30 Byte
	for(i=0,j=1;i<dim;i+=n){
		bufferU[j++]=uWRX;
		bufferU[j++]=i<(dim-n)?n*8:(dim-i)*8;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(bufferI[1]==uWRX&&bufferI[2]<0xFA){
			for(z=3;z<bufferI[2]/8+3&&z<DIMBUF;z+=2,k+=2){
				if(memEE[k+1]!=bufferI[z]){
					PrintMessage("\n");
					PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k+1],bufferI[z]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
					errori++;
				}
				if(memEE[k]!=bufferI[z+1]){
					PrintMessage("\n");
					PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k],bufferI[z+1]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
					errori++;
				}
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
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
	}
	PrintMessage(strings[S_ComplErr],errori);	//"terminata: %d errori\r\n"
//****************** exit ********************
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void Write93Cx(int dim,int na, int options)
// write 93Cx6 uW memories
// dim=size in bytes
// na=address bits
// options=0: x16 organization     =1: x8 organization
{
	int k=0,z=0,i,j;
	int errori=0;
	if(dim>0x1000||dim<0){
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	if(na>13) na=13;
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write93Cx(%d,%d,%d)    (0x%X,0x%X)\n",dim,na,options,dim,na);
	}
	if(dim>sizeEE){
		i=sizeEE;
		memEE=realloc(memEE,dim);
		for(;i<dim;i++) memEE[i]=0xFF;
		sizeEE=dim;
	}
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=uW_INIT;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG+PRE:S+PRE;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;
	bufferU[j++]=0x98;				//100 11xxx EWEN
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?ORG+PRE:PRE;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG+PRE:S+PRE;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;
	bufferU[j++]=0x90;				//100 10xxx ERAL
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?ORG+PRE:PRE;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG+PRE:S+PRE;
	bufferU[j++]=0;
	bufferU[j++]=uWRX;
	bufferU[j++]=1;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	j=1;
	bufferU[j++]=uWRX;
	bufferU[j++]=1;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	for(i=0,k=0;i<30&&!k;i++){		//Wait until ready
		write();
		msDelay(2);
		read();
		if(saveLog)WriteLogIO();
		k=bufferI[3];
	}
//****************** write ********************
	PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
	PrintMessage("   ");
	int addr=0;
	j=1;
	for(i=0;i<dim;i+=options==0?2:1,addr+=0x10000>>na){
		if(memEE[i]<0xFF||(options==0&&memEE[i+1]<0xFF)){
			bufferU[j++]=uWTX;
			bufferU[j++]=3;
			bufferU[j++]=0xA0;			//101aaaaa aaa(a) write
			bufferU[j++]=uWTX;
			bufferU[j++]=na;
			bufferU[j++]=addr>>8;
			if(na>8) bufferU[j++]=addr&0xFF;
			bufferU[j++]=uWTX;
			if(options==0){		//x16
				bufferU[j++]=16;
				bufferU[j++]=memEE[i+1];
				bufferU[j++]=memEE[i];
			}
			else{				//x8
				bufferU[j++]=8;
				bufferU[j++]=memEE[i];
			}
			bufferU[j++]=EXT_PORT;
			bufferU[j++]=options==0?ORG+PRE:PRE;
			bufferU[j++]=0;
			bufferU[j++]=EXT_PORT;
			bufferU[j++]=options==0?S+ORG+PRE:S+PRE;
			bufferU[j++]=0;
			bufferU[j++]=uWRX;
			bufferU[j++]=1;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(1.5);
			read();
			if(bufferI[1]!=uWTX||bufferI[2]>=0xFA) i=dim+10;
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
			bufferU[j++]=uWRX;
			bufferU[j++]=1;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			for(z=0,k=0;z<30&&!k;z++){		//Wait until ready
				write();
				msDelay(1.5);
				read();
				if(saveLog)WriteLogIO();
				k=bufferI[3];
			}
			j=1;
		}
	}
	msDelay(1);
	PrintMessage("\b\b\b");
	if(i!=dim){
		PrintMessage(strings[S_CodeWError4],i,dim);	//"Errore in scrittura area programma, richiesti %d byte, scritti %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** verify EEPROM ********************
	PrintMessage(strings[S_EEV]);	//"Verifica EEPROM ... "
	PrintMessage("   ");
	bufferU[0]=0;
	j=1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?ORG+PRE:PRE;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG+PRE:S+PRE;
	bufferU[j++]=0;
	bufferU[j++]=FLUSH;
	write();
	msDelay(1.5);
	read();
	if(saveLog)WriteLogIO();/**/
	bufferU[0]=0;
	j=1;
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=options==0?S+ORG:S;
	bufferU[j++]=0;
	bufferU[j++]=uWTX;
	bufferU[j++]=na+3;				//READ (16bit)
	bufferU[j++]=0xC0;				//110aaaaa aaax0000
	bufferU[j++]=0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	k=0;
	int n=(DIMBUF-2);
	if(n>30) n=30;	//max 240 bit = 30 Byte
	for(i=0,j=1;i<dim;i+=n){
		bufferU[j++]=uWRX;
		bufferU[j++]=i<(dim-n)?n*8:(dim-i)*8;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(options==0){		//x16
			if(bufferI[1]==uWRX&&bufferI[2]<0xFA){
				for(z=3;z<bufferI[2]/8+3&&z<DIMBUF;z+=2,k+=2){
					if(memEE[k+1]!=bufferI[z]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k+1],bufferI[z]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
						errori++;
					}
					if(memEE[k]!=bufferI[z+1]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k],bufferI[z+1]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
						errori++;
					}
				}
			}
		}
		else{				//x8
			if(bufferI[1]==uWRX&&bufferI[2]<0xFA){
				for(z=3;z<bufferI[2]/8+3&&z<DIMBUF;z++,k++){
					if(memEE[k]!=bufferI[z]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k],bufferI[z]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
						errori++;
					}
				}
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
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
	}
	PrintMessage(strings[S_ComplErr],errori);	//"terminata: %d errori\r\n"
//****************** exit ********************
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

#define CS 8
#define HLD 16		//Hold
#define WP 0x40		//Write protect

void Read25xx(int dim)
// read 25xx SPI memories
// dim=size in bytes
{
	int k=0,z=0,i,j;
	if(dim>0x1000000||dim<0){
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
		fprintf(RegFile,"Read25xx(%d)    (0x%X)\n",dim,dim);
	}
	sizeEE=dim;
	memEE=malloc(dim);			//EEPROM
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=SPI_INIT;
	bufferU[j++]=2;				//0=100k, 1=200k, 2=300k
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EXT_PORT;	//CS=1, HLD=1, WP=0
	bufferU[j++]=CS+HLD;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;	//CS=0, HLD=1, WP=0
	bufferU[j++]=HLD;
	bufferU[j++]=0;
	bufferU[j++]=SPI_WRITE;		//Read
	if(dim>0x10000){				//24 bit address
		bufferU[j++]=4;
		bufferU[j++]=3;
		bufferU[j++]=0;
		bufferU[j++]=0;
		bufferU[j++]=0;
	}
	else if(dim>0x200){				//16 bit address
		bufferU[j++]=3;
		bufferU[j++]=3;
		bufferU[j++]=0;
		bufferU[j++]=0;
	}
	else{						//8 bit address
		bufferU[j++]=2;
		bufferU[j++]=3;
		bufferU[j++]=0;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** read ********************
	PrintMessage(strings[S_ReadEE]);		//lettura EE ...
	PrintMessage("   ");
	for(i=0,j=1;i<dim;i+=DIMBUF-4){
		bufferU[j++]=SPI_READ;
		bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(4);
		read();
		if(bufferI[1]==SPI_READ&&bufferI[2]<0xFA){
			for(z=3;z<bufferI[2]+3&&z<DIMBUF;z++) memEE[k++]=bufferI[z];
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
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		sizeEE=k;
	}
	else PrintMessage(strings[S_Compl]);
//****************** exit ********************
	bufferU[j++]=EXT_PORT;
	bufferU[j++]=0;
	bufferU[j++]=0;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	DisplayEE();	//visualize
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void Write25xx(int dim,int page,float wait)
// write SPI memories
// dim=size in bytes
// page=page size		wait=write delay
{
	int k=0,z=0,i,j;
	int errori=0;
	if(dim>0x1000000||dim<0){
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
		fprintf(RegFile,"Write25xx(%d,%d,%f)    (0x%X)\n",dim,page,wait,dim);
	}
	if(dim>sizeEE){
		i=sizeEE;
		memEE=realloc(memEE,dim);
		for(;i<dim;i++) memEE[i]=0xFF;
		sizeEE=dim;
	}
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Area dati vuota\r\n"
		return;
	}
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=VREG_DIS;
	bufferU[j++]=SPI_INIT;
	bufferU[j++]=2;				//0=100k, 1=200k, 2=300k
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EXT_PORT;	//CS=1, HLD=1, WP=1
	bufferU[j++]=CS+HLD;
	bufferU[j++]=WP;
	bufferU[j++]=EXT_PORT;	//CS=0, HLD=1, WP=1
	bufferU[j++]=HLD;
	bufferU[j++]=WP;
	bufferU[j++]=SPI_WRITE;		//WRITE STATUS
	bufferU[j++]=2;
	bufferU[j++]=1;
	bufferU[j++]=0;
	bufferU[j++]=EXT_PORT;	//CS=1, HLD=1, WP=1
	bufferU[j++]=CS+HLD;
	bufferU[j++]=WP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(wait+1);
	read();
	if(saveLog)WriteLogIO();
//****************** write ********************
	PrintMessage(strings[S_EEAreaW]);	//"Scrittura EEPROM ... "
	for(;page>=DIMBUF-22;page>>=1);
	//page=1;
	for(i=0,j=1;i<dim;i+=page){
		bufferU[j++]=EXT_PORT;	//CS=0, HLD=1, WP=1
		bufferU[j++]=HLD;
		bufferU[j++]=WP;
		bufferU[j++]=SPI_WRITE;		//WRITE ENABLE
		bufferU[j++]=1;
		bufferU[j++]=6;
		bufferU[j++]=EXT_PORT;	//CS=1, HLD=1, WP=1
		bufferU[j++]=CS+HLD;
		bufferU[j++]=WP;/**/
		bufferU[j++]=EXT_PORT;	//CS=0, HLD=1, WP=1
		bufferU[j++]=HLD;
		bufferU[j++]=WP;
		bufferU[j++]=SPI_WRITE;		//WRITE
		if(dim>0x10000){				//24 bit address
			bufferU[j++]=4+page;
			bufferU[j++]=2;
			bufferU[j++]=i>>16;
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=i&0xFF;
		}
		else if(dim>0x200){				//16 bit address
			bufferU[j++]=3+page;
			bufferU[j++]=2;
			bufferU[j++]=i>>8;
			bufferU[j++]=i&0xFF;
		}
		else{						//8 bit address
			bufferU[j++]=2+page;
			bufferU[j++]=2+(i&0x100?8:0);
			bufferU[j++]=i&0xFF;
		}
		for(k=0;k<page;k++) bufferU[j++]=memEE[i+k];
		bufferU[j++]=EXT_PORT;	//CS=1, HLD=1, WP=1
		bufferU[j++]=CS+HLD;
		bufferU[j++]=WP;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(wait+2);
		read();
		if(bufferI[2]!=SPI_WRITE||bufferI[3]>=0xFA) i=dim+10;
		PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
		j=1;
		if(saveLog){
			fprintf(RegFile,strings[S_Log7],i,i,0,0);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
	}
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_Compl]);	//"terminata\r\n"
//****************** verify EEPROM ********************
	PrintMessage(strings[S_EEV]);	//"Verifica EEPROM ... "
	PrintMessage("   ");
	j=1;
	bufferU[j++]=EXT_PORT;	//CS=0, HLD=1, WP=0
	bufferU[j++]=HLD;
	bufferU[j++]=0;
	bufferU[j++]=SPI_WRITE;		//Read
	if(dim>0x10000){				//24 bit address
		bufferU[j++]=4;
		bufferU[j++]=3;
		bufferU[j++]=0;
		bufferU[j++]=0;
		bufferU[j++]=0;
	}
	else if(dim>0x200){				//16 bit address
		bufferU[j++]=3;
		bufferU[j++]=3;
		bufferU[j++]=0;
		bufferU[j++]=0;
	}
	else{						//8 bit address
		bufferU[j++]=2;
		bufferU[j++]=3;
		bufferU[j++]=0;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	k=0;
	for(i=0,j=1;i<dim;i+=DIMBUF-4){
		bufferU[j++]=SPI_READ;
		bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(4);
		read();
		if(bufferI[1]==SPI_READ&&bufferI[2]<0xFA){
			for(z=3;z<bufferI[2]+3&&z<DIMBUF;z++){
				if(memEE[k++]!=bufferI[z]){
					PrintMessage("\n");
					PrintMessage(strings[S_CodeVError],i+z-3,i+z-3,memEE[k-1],bufferI[z]);	//"Errore in verifica, indirizzo %04X (%d), scritto %02X, letto %02X\r\n"
					errori++;
				}
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
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadEEErr],dim,k);	//"Errore in lettura area EEPROM, richiesti %d byte, letti %d\r\n"
		sizeEE=k;
	}
	PrintMessage(strings[S_ComplErr],errori);	//"terminata: %d errori\r\n"
//****************** exit ********************
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	unsigned int stop=GetTickCount();
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}















