/*
 * progP16.c - algorithms to program the PIC16 (14 bit word) family of microcontrollers
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

void Read16Fxxx(int dim,int dim2,int dim3,int vdd){
// read 14 bit PIC
// dim=program size 	dim2=eeprom size   dim3=config size
// dim2<0 -> eeprom @ 0x2200
// vdd=0 -> vpp before vdd
// vdd=1 -> vdd (+50ms) before vpp
// vdd=2 -> vdd before vpp
// DevID@0x2006
// Config@0x2007
// Calib1/Config2@0x2008
// Calib2/Calib1@0x2009
// eeprom@0x2100
	int k=0,k2=0,z=0,i,j,ee2200=0;
	char str[512],s[256],t[256];
	if(dim2<0){
		dim2=-dim2;
		ee2200=1;
	}
	if(dim>0x2000||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Dimensione programma oltre i limiti\r\n"
		return;
	}
	if(dim2>0x400||dim2<0){		//Max 1K
		PrintMessage(strings[S_EELim]);	//"Dimensione eeprom oltre i limiti\r\n"
		return;
	}
	if(dim3>0x100||dim3<0){
		PrintMessage(strings[S_ConfigLim]);	//"Dimensione area config oltre i limiti\r\n"
		return;
	}
	if(dim3<8)dim3=8;
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Read12F6xx(%d,%d,%d,%d)\n",dim,dim2,dim3,vdd);
	}
	size=0x2100+dim2;
	dati_hex=malloc(sizeof(WORD)*size);
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	if(vdd==0){						//VPP before VDD
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=4;				//VPP
		bufferU[j++]=NOP;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x5;			//VDD+VPP
	}
	else if(vdd==1){				//VDD before VPP with delay 50ms
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=1;				//VDD
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=25000>>8;
		bufferU[j++]=25000&0xff;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=2000>>8;
		bufferU[j++]=2000&0xff;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x5;			//VDD+VPP
	}
	else if(vdd==2){				//VDD before VPP without delay
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=1;				//VDD
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x5;			//VDD+VPP
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	if(vdd) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//lettura codice ...
	PrintMessage("   ");
	for(i=0,j=1;i<dim;i++){
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==dim-1){		//2 istruzioni -> 4 risposte
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					dati_hex[k++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	PrintMessage("\b\b\b");
	if(k!=dim){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadCodeErr],dim,k);	//"Errore in lettura area programma, richieste %d word, lette %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=k;i<0x2000;i++) dati_hex[i]=0x3fff;
//****************** read config area ********************
	PrintMessage(strings[S_Read_CONFIG_A]);		//lettura config ...
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	for(i=0x2000;i<0x2000+dim3;i++){		//Config
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==0x2000+dim3-1){		//2 istruzioni -> 4 risposte
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					dati_hex[0x2000+k2++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,k2,k2);	//"i=%d, k=%d\n"
				WriteLogIO();
			}
		}
	}
	if(k2!=dim3){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigErr],dim3,k2);	//"Errore in lettura area configurazione, richieste %d word, lette %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=0x2000+k2;i<0x2000+dim3;i++) dati_hex[i]=0x3fff;
//****************** read eeprom ********************
	if(dim2){
		PrintMessage(strings[S_ReadEE]);		//lettura EE ...
		PrintMessage("   ");
		if(ee2200){		//eeprom a 0x2200
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0xFF;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x100-dim3;
		for(k2=0,i=0x2100;i<0x2100+dim2;i++){
			bufferU[j++]=READ_DATA_DATA;
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF*2/4-2||i==0x2100+dim2-1){		//2 istruzioni -> 4 risposte
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(10);
				read();
				for(z=1;z<DIMBUF-1;z++){
					if(bufferI[z]==READ_DATA_DATA){
						dati_hex[0x2100+k2++]=bufferI[z+1];
						z++;
					}
				}
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log7],i,i,k2,k2);	//"i=%d, k2=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintMessage("\b\b\b");
		if(k2!=dim2){
			PrintMessage("\n");
			PrintMessage(strings[S_ReadEEErr],dim2,k2);	//"Errore in lettura area EE, ..."
			for(i=0x2100+k2;i<0x2100+dim2;i++) dati_hex[i]=0x3fff;
		}
		else PrintMessage(strings[S_Compl]);
	}
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=1;					//VDD
	bufferU[j++]=EN_VPP_VCC;
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
	for(i=0;i<4;i+=2){
		PrintMessage("ID%d: 0x%04X\tID%d: 0x%04X\r\n",i, dati_hex[0x2000+i],i+1, dati_hex[0x2000+i+1]);
	}
	PrintMessage(strings[S_DevID],dati_hex[0x2006]);	//"DevID: 0x%04X\r\n"
	PIC_ID(dati_hex[0x2006]);
	PrintMessage(strings[S_ConfigWord],dati_hex[0x2007]);	//"Configuration word: 0x%04X\r\n"
	if(dim3>8){
		PrintMessage(strings[S_Config2Cal1],dati_hex[0x2008]);	//"Config2 or cal1: 0x%04X\r\n"
	}
	if(dim3>9){
		PrintMessage(strings[S_Calib1_2],dati_hex[0x2009]);	//"Calibration word 1 or 2: 0x%04X\r\n"
	}
	PrintMessage(strings[S_CodeMem2]);	//"\r\nMemoria programma:\r\n"
	s[0]=0;
	int valid=0,empty=1;
	for(i=0;i<dim;i+=COL){
		valid=0;
		for(j=i;j<i+COL&&j<dim;j++){
			sprintf(t,"%04X ",dati_hex[j]);
			strcat(s,t);
			if(dati_hex[j]<0x3fff) valid=1;
		}
		if(valid){
			PrintMessage("%04X: %s\r\n",i,s);
			empty=0;
		}
		s[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	if(dim3>8){
		empty=1;
		PrintMessage(strings[S_ConfigResMem]);	//"\r\nMemoria configurazione e riservata:\r\n"
		for(i=0x2000;i<0x2000+dim3;i+=COL){
			valid=0;
			for(j=i;j<i+COL&&j<0x2000+dim3;j++){
				sprintf(t,"%04X ",dati_hex[j]);
				strcat(s,t);
				if(dati_hex[j]<0x3fff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s\r\n",i,s);
				empty=0;
			}
			s[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
	}
	if(dim2){
		empty=1;
		str[0]=0;
		PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
		for(i=0x2100;i<0x2100+dim2;i+=COL){
			valid=0;
			for(j=i;j<i+COL&&j<0x2100+dim2;j++){
				sprintf(t,"%02X ",dati_hex[j]&0xff);
				strcat(s,t);
				sprintf(t,"%c",isprint(dati_hex[j]&0xff)?dati_hex[j]&0xff:'.');
				strcat(str,t);
				if(dati_hex[j]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s %s\r\n",i,s,str);
				empty=0;
			}
			s[0]=0;
			str[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
	}
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
	if(saveLog&&RegFile) fclose(RegFile);
}

void Write12F6xx(int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vpp before vdd
// DevID@0x2006
// Config@0x2007
// Calib1@0x2008 (save)
// Calib2@0x2009 (save)
// eeprom@0x2100
// erase: BULK_ERASE_PROG (1001) +10ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 4ms
// eeprom:	BULK_ERASE_DATA (1011) + 16ms
//			LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 8ms
// verify during write
{
	int errori=0;
	WORD devID=0x3fff,calib1=0x3fff,calib2=0x3fff;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(load_calibword){
		if(size>0x2009) load_calibword=2;
		else if(size>0x2008) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Impossibile trovare i dati di calibrazione\r\n"
			load_calibword=0;
		}
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write12F6xx(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib1
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib2
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff){
		PrintMessage(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	}
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib2=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib2<0x3fff){
		PrintMessage(strings[S_CalibWord2],calib2);	//"Calib2: 0x%04X\r\n"
	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID||load_calibword){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			if(load_calibword==2) bufferU[j++]=0x09;
			else bufferU[j++]=0x08;
		}
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	if(dim2){
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	}
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms tra exit e rientro program mode
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=4000>>8;
	bufferU[j++]=4000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write eeprom ********************
	if(dim2){
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=8000>>8;
		bufferU[j++]=8000&0xff;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// ritardo T3=8ms
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=INC_ADDR;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2001;		//azzera il contatore EEPROM
		for(w=2,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms min
				bufferU[j++]=WAIT_T3;				//Tprogram 8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-12||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*9+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							errori++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		errori+=i-k;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],i-k);	//"completata, %d errori\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=4000>>8;
	bufferU[j++]=4000&0xff;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 4ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 1
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 4ms
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 2
		bufferU[j++]=dati_hex[0x2009]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2009]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 4ms
		bufferU[j++]=READ_DATA_PROG;
	}
	else bufferU[j++]=INC_ADDR;			//0x2009
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_Calib1Err],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura Calib1: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		if(load_calibword==2){
			for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
			if (dati_hex[0x2009]!=(bufferI[z+1]<<8)+bufferI[z+2]){
				PrintMessage("\n");
				PrintMessage(strings[S_Calib2Err],dati_hex[0x2009],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura Calib2: scritto %04X, letto %04X\r\n"
				err_c++;
			}
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F8x (int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vdd + 50ms + vdd&vpp
// DevID@0x2006
// Config@0x2007
// eeprom@0x2100
// erase if protected:
// LOAD_CONF (0)(0x3FFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 10ms
// + CUST_CMD (0001) + CUST_CMD (0111)
// erase if not protected and DevID=16F84A:
// LOAD_DATA_PROG (0010)(0x3FFF) + BULK_ERASE_PROG (1001) +10ms
// LOAD_DATA_DATA (0011)(0xFF) + BULK_ERASE_DATA (1011) + BEGIN_PROG (1000) + 10ms
// erase erase if not protected and DevID!=16F84A:
// LOAD_DATA_PROG (0010)(0x3FFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 10ms + CUST_CMD (0001) + CUST_CMD (0111)
// LOAD_DATA_DATA (0011)(0xFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 10ms + CUST_CMD (0001) + CUST_CMD (0111)
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 20ms o 8ms(16F84A)
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 20ms o 8ms(16F84A)
// verify during write
{
	int errori=0;
	WORD devID,config;
	int k=0,z=0,i,j,w,r;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F8x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=25000>>8;
	bufferU[j++]=25000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=WAIT_T3;			//50ms
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(140);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	if(config<0x3FF0){
		PrintMessage(strings[S_ProtErase]);	//"Il dispositivo ï¿½ protetto, sovrascrivo la protezione.\r\n"
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0x3F;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x07;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
	}
	else if(devID>>5==0x2B){			//16F84A
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=0x3f;				//MSB
		bufferU[j++]=0xff;				//LSB
		bufferU[j++]=BULK_ERASE_PROG;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		if(dim2){
			bufferU[j++]=LOAD_DATA_DATA;
			bufferU[j++]=0xff;				//LSB
			bufferU[j++]=BULK_ERASE_DATA;
			bufferU[j++]=BEGIN_PROG;
			bufferU[j++]=WAIT_T3;
		}
	}
	else{								//altri
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=0x3f;				//MSB
		bufferU[j++]=0xff;				//LSB
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		if(dim2){
			bufferU[j++]=LOAD_DATA_DATA;	//EEPROM:  errore nelle spec?
			bufferU[j++]=0xff;				//LSB
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x01;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x07;
			bufferU[j++]=BEGIN_PROG;
			bufferU[j++]=WAIT_T3;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x01;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x07;
		}
	}
	if(!programID){					//torna in memoria programma
		bufferU[j++]=NOP;				//exit program mode
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x0;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
		bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
		bufferU[j++]=WAIT_T3;			//50ms
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
	}
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	r=(devID>>5==0x2B)?8000:20000;
	bufferU[j++]=r>>8;
	bufferU[j++]=r&0xff;
	bufferU[j++]=FLUSH;
	r/=1000;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(60);
	if(!programID) msDelay(80);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*r+4);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//internally timed
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	if(programID) msDelay(90);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2008;		//azzera il contatore EEPROM
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*r+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							errori++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		errori+=i-k;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],i-k);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F62x (int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vpp before vdd
// DevID@0x2006
// Config@0x2007
// eeprom@0x2200
// erase if protected:
// LOAD_CONF (0000)(0) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 15ms + CUST_CMD (0001) + CUST_CMD (0111)
// erase if not protected:
// LOAD_DATA_PROG (0010)(0x3FFF) + BULK_ERASE_PROG (1001) +5ms
// LOAD_DATA_DATA (0011)(0xFF) + BULK_ERASE_DATA (1011) + BEGIN_PROG (1000) + 5ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 8ms
// write ID: LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 16ms
// write CONFIG: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 8ms
// eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG2 (11000) + 8ms
// verify during write
{
	int errori=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F62x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo exit-enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(12);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=5000>>8;
	bufferU[j++]=5000&0xff;
	if(config<0x3C00){
		PrintMessage(strings[S_ProtErase]);	//"Il dispositivo è protetto, sovrascrivo la protezione.\r\n"
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0x3F;				//config fasulla	ERRORE spec!!! c'era scritto dati=0!!
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x07;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=BEGIN_PROG;		//Tera+Tprog=5+8 ms
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
	}
	else{
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=0x3f;				//MSB
		bufferU[j++]=0xff;				//LSB
		bufferU[j++]=BULK_ERASE_PROG;
		bufferU[j++]=BEGIN_PROG;		//Tera=5ms
		bufferU[j++]=WAIT_T3;
		if(dim2){
			bufferU[j++]=LOAD_DATA_DATA;
			bufferU[j++]=0xff;				//LSB
			bufferU[j++]=BULK_ERASE_DATA;
			bufferU[j++]=BEGIN_PROG;		//Tera=5ms
			bufferU[j++]=WAIT_T3;
		}
	}
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(60);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//program only, internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram=8ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*9+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//erase + prog internally timed, T=8+5 ms
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;				//prog only, internally timed, T=8 ms
	bufferU[j++]=WAIT_T3;					//Tprogram 8ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	if(programID) msDelay(90);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2102-0x2008;		//azzera il contatore EEPROM
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2200-0x2102;		//azzera il contatore EEPROM
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram=8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*14+1);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							errori++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		errori+=i-k;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],i-k);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write12F62x(int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vpp before vdd
// salva OSCCAL a dim-1
// CONFIG@0x2007 includes 2  calibration bits
// DevID@0x2006
// eeprom@0x2100
// erase: BULK_ERASE_PROG (1001) +10ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 3ms
// eeprom: BULK_ERASE_DATA (1011) + 9ms
// LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 6ms
// verify during write
{
	int errori=0;
	WORD devID,config,osccal;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write12F62x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<size;i++) dati_hex[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	for(i=0;i<dim-0xff;i+=0xff){
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0xff;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=dim-1-i;
	bufferU[j++]=READ_DATA_PROG;	// OSCCAL
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=1;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	osccal=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+1];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms tra exit e rientro program mode
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=3000>>8;
	bufferU[j++]=3000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	if(!load_osccal) dati_hex[dim-1]=osccal;	//backup osccal
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*6.5);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	PrintMessage("\b\b\b");
	errori+=i-k;
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	if(!load_calibword)	dati_hex[0x2007]=(dati_hex[0x2007]&0xfff)+(config&0x3000);
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms
	bufferU[j++]=WAIT_T3;				//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=6000>>8;
		bufferU[j++]=6000&0xff;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2007;		//azzera il contatore EEPROM
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// ritardo=12ms
		bufferU[j++]=WAIT_T3;
		for(w=3,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms
				bufferU[j++]=WAIT_T3;				//Tprogram 6ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*7+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							errori++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		errori+=i-k;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],i-k);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F87x (int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// dim2<0 -> eeprom @ 0x2200
// vdd + (50ms?) + vdd&vpp
// DevID@0x2006
// Config@0x2007
// eeprom@0x2100
// erase if protected:
// LOAD_CONF (0000)(0x3FFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 8ms
// + CUST_CMD (0001) + CUST_CMD (0111)
// erase if not protected:
// LOAD_DATA_PROG (0010)(0x3FFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 8ms + CUST_CMD (0001) + CUST_CMD (0111)
// LOAD_DATA_DATA (0011)(0xFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 8ms + CUST_CMD (0001) + CUST_CMD (0111)
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 4ms
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 8ms
// verify during write
{
	int errori=0;
	WORD devID,config;
	int k=0,z=0,i,j,w,ee2200=0;
	if(dim2<0){
		dim2=-dim2;
		ee2200=1;
	}
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F87x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<size;i++) dati_hex[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(60);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	if((config&0x3130)!=0x3130){
		PrintMessage(strings[S_ProtErase]);	//"Il dispositivo è protetto, sovrascrivo la protezione.\r\n"
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0x3F;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x07;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=NOP;				//exit program mode
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x0;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
		bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
	}
	else{
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=0x3f;				//MSB
		bufferU[j++]=0xff;				//LSB
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x01;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x07;
		if(dim2){
			bufferU[j++]=LOAD_DATA_DATA;	//EEPROM:  errore nelle spec?
			bufferU[j++]=0xff;				//LSB
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x01;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x07;
			bufferU[j++]=BEGIN_PROG;
			bufferU[j++]=WAIT_T3;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x01;
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x07;
		}
	}
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=4000>>8;
	bufferU[j++]=4000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(60);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;			//internally timed
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	if(programID) msDelay(90);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		if(ee2200){		//eeprom a 0x2200
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0xFF;
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=1;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2007;		//azzera il contatore EEPROM
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//internally timed ?????
				bufferU[j++]=WAIT_T3;				//Tprogram         ?????
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*8+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							err_e++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		errori+=err_e;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],err_e);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F87xA (int dim,int dim2,int seq)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// seq=0: vdd + (50ms) + vdd&vpp
// seq=1: vdd + (50us) + vdd&vpp
// DevID@0x2006
// Config@0x2007
// write CONFIG2@0x2008 if different from 3FFF
// eeprom@0x2100
// erase:
// CHIP ERASE (11111) + 8ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 1.5ms + END_PROGX (10111)
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 8ms
// verify during write
{
	int errori=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F87x(%d,%d,%d)\n",dim,dim2,seq);
	}
	for(i=0;i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=25000>>8;
	bufferU[j++]=25000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(seq==0){
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	if(seq==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0x3F;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
	}
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x1F;					// CHIP_ERASE (11111)
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=1200>>8;
	bufferU[j++]=1200&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(60);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*1.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=7;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed
	bufferU[j++]=WAIT_T3;					//Tprogram
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(dati_hex[0x2008]!=0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;				//internally timed
		bufferU[j++]=WAIT_T3;					//Tprogram
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=7;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(dati_hex[0x2008]!=0x3fff){
		for(z+=7;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0xFF;				//azzera il contatore EEPROM
		bufferU[j++]=INC_ADDR;
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*9+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							err_e++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		errori+=err_e;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],err_e);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F81x (int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// seq=0: vdd + (50ms) + vdd&vpp
// seq=1: vdd + (50us) + vdd&vpp
// DevID@0x2006
// Config@0x2007
// write CONFIG2@0x2008 if different from 3FFF
// erase if protected: CHIP ERASE (11111) + 8ms
// erase if not protected:
// BULK_ERASE_PROG (1001) + BEGIN_PROG (1001) + 2ms + END_PROGX (10111)
// BULK_ERASE_DATA (1011) + BEGIN_PROG (1001) + 2ms + END_PROGX (10111)
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 1.5ms + END_PROGX (10111)
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG2 (11000) + 1.5ms + END_PROGX (10111)
// verify during write
{
	int errori=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F81x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=25000>>8;
	bufferU[j++]=25000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0x3F;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
	}
	if((config&0x2100)!=0x2100){
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x1F;					// CHIP_ERASE (11111)
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
	}
	else{
		bufferU[j++]=BULK_ERASE_PROG;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x17;					//END_PROGX (10111)
	}
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*2.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=7;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x0;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
		bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
		bufferU[j++]=LOAD_DATA_DATA;
		bufferU[j++]=0x01;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=BEGIN_PROG;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x17;					//END_PROGX (10111)
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(5);
		read();
		if(saveLog)WriteLogIO();
		j=1;
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//externally timed
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=CUST_CMD;
				bufferU[j++]=0x17;					//END_PROGX (10111)
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*2.5+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-5;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+4]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+5]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+5]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							err_e++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=6;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		errori+=err_e;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],err_e);	//"completata, %d errori\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;				//externally timed
	bufferU[j++]=WAIT_T3;					//Tprogram
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x17;						//END_PROGX (10111)
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(dati_hex[0x2008]!=0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG2;				//externally timed
		bufferU[j++]=WAIT_T3;					//Tprogram
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x17;						//END_PROGX (10111)
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=7;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(dati_hex[0x2008]!=0x3fff){
		for(z+=7;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write12F61x(int dim)
// write 14 bit PIC
// dim=program size
// vpp before vdd
// DevID@0x2006
// Config@0x2007
// Calib1@0x2008 (save)
// erase: BULK_ERASE_PROG (1001) +10ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 4ms + END_PROG (1010)
// verify during write
{
	int errori=0;
	WORD devID=0x3fff,calib1=0x3fff;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(load_calibword){
		if(size>0x2008) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Impossibile trovare i dati di calibrazione\r\n"
			load_calibword=0;
		}
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write12F61x(%d)\n",dim);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib1
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib2
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff){
		PrintMessage(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID||load_calibword){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0x08;
		}
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms tra exit e rientro program mode
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=4000>>8;
	bufferU[j++]=4000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=END_PROG;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-7;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+5]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+6]<<8)+bufferI[z+7]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=8;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=END_PROG;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 4ms
	bufferU[j++]=END_PROG;
	bufferU[j++]=WAIT_T2;				//Tdischarge 100us
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 1
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 4ms
		bufferU[j++]=END_PROG;
		bufferU[j++]=WAIT_T2;				//Tdischarge 100us
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=8;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=8;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_Calib1Err],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura Calib1: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F88x(int dim,int dim2)
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vpp before vdd
// DevID@0x2006
// Config@0x2007
// Config2@0x2008
// Calib1@0x2009 (salva)
// eeprom@0x2100
// erase: BULK_ERASE_PROG (1001) +6ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 3ms
// eeprom:	BULK_ERASE_DATA (1011) + 6ms
//			LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 6ms
// verify during write
{
	int errori=0;
	WORD devID=0x3fff,calib1=0x3fff;
	int k=0,z=0,i,j,w;
	if(size<0x2009){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(load_calibword){
		if(size>0x200A) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Impossibile trovare i dati di calibrazione\r\n"
			load_calibword=0;
		}
	}
	if(dim2){
		if(size<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Impossibile trovare i dati EEPROM\r\n"
		}
		else if(dim2>size-0x2100) dim2=size-0x2100;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F88x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x200A&&i<size;i++) dati_hex[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib1
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=6000>>8;
	bufferU[j++]=6000&0xff;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff){
		PrintMessage(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID||load_calibword){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=9;
		}
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=6ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// ritardo T3=6ms tra exit e rientro program mode
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=3000>>8;
	bufferU[j++]=3000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(40);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*3+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=6;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
	bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;		//Calib word 1
		bufferU[j++]=dati_hex[0x2009]>>8;	//MSB
		bufferU[j++]=dati_hex[0x2009]&0xff;	//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 3ms
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2009]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_Calib1Err],dati_hex[0x2009],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura Calib1: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** write eeprom ********************
	if(dim2){
		for(i=2100;i<0x2100+dim2;i++) dati_hex[i]&=0xff;
		PrintMessage(strings[S_EEAreaW]);	//"Scrittura area EEPROM ... "
		PrintMessage("   ");
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=6000>>8;
		bufferU[j++]=6000&0xff;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// ritardo T3=6ms
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2009;		//azzera il contatore EEPROM
		for(w=2,i=k=0x2100;i<0x2100+dim2;i++){
			if(dati_hex[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=dati_hex[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms min
				bufferU[j++]=WAIT_T3;				//Tprogram 8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-12||i==0x2100+dim2-1){
				PrintMessage("\b\b\b%2d%%",(i-0x2100)*100/dim2); fflush(stdout);
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*6.5+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (dati_hex[k]!=bufferI[z+4]){
							PrintMessage("\n");
							PrintMessage(strings[S_CodeWError3],k,dati_hex[k],bufferI[z+4]);	//"Errore in scrittura all'indirizzo %4X: scritto %02X, letto %02X\r\n"
							errori++;
							if(max_errori&&errori>max_errori){
								PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
								PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
								i=0x2200;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
					WriteLogIO();
				}
			}
		}
		errori+=i-k;
		PrintMessage("\b\b\b");
		PrintMessage(strings[S_ComplErr],i-k);	//"completata, %d errori\r\n"
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F7x(int dim,int vdd)
// dim=program size
// write 14 bit PIC
// vdd=0  vdd +50ms before vpp
// vdd=1  vdd before vpp
// DevID@0x2006
// Config@0x2007
// Config2@0x2008
// erase: BULK_ERASE_PROG (1001) +30ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 1ms + END_PROG2(1110)
// verify during write
{
	int errori=0;
	WORD devID=0x3fff,calib1=0x3fff;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F7x(%d,%d)\n",dim,vdd);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=1000>>8;
	bufferU[j++]=1000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(90);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms min
			bufferU[j++]=END_PROG2;
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*1.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=7;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms
			bufferU[j++]=END_PROG2;
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 1ms
	bufferU[j++]=END_PROG2;
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(dati_hex[0x2008]<0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 2 0x2008
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 1ms
		bufferU[j++]=END_PROG2;
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(dati_hex[0x2008]<0x3fff){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

void Write16F71x(int dim,int vdd)
// write 14 bit PIC
// dim=program size
// vdd=0  vdd +50ms before vpp
// vdd=1  vdd before vpp
// DevID@0x2006
// Config@0x2007
// erase: BULK_ERASE_PROG (1001) +6ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 2ms + END_PROG2(1110)
// verify during write
{
	int errori=0;
	WORD devID=0x3fff,calib1=0x3fff;
	int k=0,z=0,i,j,w;
	if(size<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Impossibile trovare la locazione CONFIG (0x2007)\r\nFine\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write16F71x(%d,%d)\n",dim,vdd);
	}
	for(i=0;i<0x2009&&i<size;i++) dati_hex[i]&=0x3FFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//enter program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
		bufferU[j++]=0xFF;				//config fasulla
		bufferU[j++]=0xFF;				//config fasulla
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//ritardo tra exit e enter prog mode
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
		bufferU[j++]=WAIT_T3;			//ritardo tra vdd e vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(30);
	if(vdd==0) msDelay(100);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	for(w=i=k=0,j=1;i<dim;i++){
		if(dati_hex[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms min
			bufferU[j++]=END_PROG2;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*2.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-7;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+5]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+6]<<8)+bufferI[z+7]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError2],k,dati_hex[k],(bufferI[z+6]<<8)+bufferI[z+7]);	//"Errore in scrittura all'indirizzo %3X: scritto %04X, letto %04X\r\n"
						errori++;
						if(max_errori&&errori>max_errori){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim;
							z=DIMBUF;
						}
					}
					k++;
					z+=8;
				}
			}
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
				WriteLogIO();
			}
		}
	}
	errori+=i-k;
	PrintMessage("\b\b\b");
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Scrittura area CONFIG ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//contatore a 0x2000
	bufferU[j++]=0xFF;				//config fasulla
	bufferU[j++]=0xFF;				//config fasulla
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms
			bufferU[j++]=END_PROG2;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=3;
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=7;
	}
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=dati_hex[0x2007]>>8;		//MSB
	bufferU[j++]=dati_hex[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 1ms
	bufferU[j++]=END_PROG2;
	bufferU[j++]=WAIT_T2;				//Tdischarge 100us
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(dati_hex[0x2008]<0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 2 0x2008
		bufferU[j++]=dati_hex[0x2008]>>8;		//MSB
		bufferU[j++]=dati_hex[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 1ms
		bufferU[j++]=END_PROG2;
		bufferU[j++]=WAIT_T2;				//Tdischarge 100us
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_IDErr],i,dati_hex[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura ID%d: scritto %04X, letto %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (dati_hex[0x2007]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
		err_c++;
	}
	if(dati_hex[0x2008]<0x3fff){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (dati_hex[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\n");
			PrintMessage(strings[S_ConfigWErr3],dati_hex[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config: scritto %04X, letto %04X\r\n"
			err_c++;
		}
	}
	errori+=err_c;
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori \n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log9],errori);	//"Area config. 	errori=%d\n"
		WriteLogIO();
	}
//****************** exit ********************
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}
