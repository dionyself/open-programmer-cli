/*
 * progP12.c - algorithms to program the PIC12 (12 bit word) family of microcontrollers
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

#include "common.h"

void Read12F5xx(int dim,int dim2)
// read 12 bit PIC
// dim=program size 	dim2=config size
// vdd before vpp
// CONFIG @ 0x7FF (upon entering in program mode)
// OSCCAL in last memory location
// 4 ID + reserved area beyond code memory
{
	int k=0,z=0,i,j;
	char s[256],t[256];
	if(dim2<4) dim2=4;
	size=0x1000;
	dati_hex=malloc(sizeof(WORD)*size);
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Read12F5xx(%d,%d)\n",dim,dim2);
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
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	bufferU[j++]=EN_VPP_VCC;		//entrata program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=READ_DATA_PROG;	//configuration word
	bufferU[j++]=INC_ADDR;			// 7FF->000
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(z<DIMBUF-2){
		dati_hex[0xfff]=(bufferI[z+1]<<8)+bufferI[z+2];
		PrintMessage("\r\n");
		PrintMessage(strings[S_ConfigWord],dati_hex[0xfff]);	//"\r\nConfiguration word: 0x%03X\r\n"
		switch(dati_hex[0xfff]&0x03){
			case 0:
				PrintMessage(strings[S_LPOsc]);	//"LP oscillator\r\n"
				break;
			case 1:
				PrintMessage(strings[S_XTOsc]);	//"XT oscillator\r\n"
				break;
			case 2:
				PrintMessage(strings[S_IntOsc]);	//"Internal osc.\r\n"
				break;
			case 3:
				PrintMessage(strings[S_RCOsc]);	//"RC oscillator\r\n"
				break;
		}
		if(dati_hex[0xfff]&0x04) PrintMessage(strings[S_WDTON]);	//"WDT ON\r\n"
		else PrintMessage(strings[S_WDTOFF]);	//"WDT OFF\r\n"
		if(dati_hex[0xfff]&0x08) PrintMessage(strings[S_CPOFF]);	//"Code protection OFF\r\n"
		else PrintMessage(strings[S_CPON]);	//"Code protection ON\r\n"
		if(dati_hex[0xfff]&0x10) PrintMessage(strings[S_MCLRON]);	//"Master clear ON\r\n"
		else PrintMessage(strings[S_MCLROFF]);	//"Master clear OFF\r\n"
	}
	else PrintMessage(strings[S_NoConfigW]);	//"Impossibile leggere la config word\r\n"
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//lettura codice ...
	PrintMessage("   ");
	for(i=0,j=1;i<dim+dim2;i++){
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==dim+dim2-1){		//2 istruzioni -> 4 risposte
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
			PrintMessage("\b\b\b%2d%%",i*100/(dim+dim2)); fflush(stdout);
			j=1;
			if(saveLog){
				fprintf(RegFile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	PrintMessage("\b\b\b");
	bufferU[j++]=NOP;				//uscita program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(1);
	read();
	unsigned int stop=GetTickCount();
	if(saveLog&&RegFile) fclose(RegFile);
	for(i=k;i<0xfff;i++) dati_hex[i]=0xfff;
	if(k!=dim+dim2){
		PrintMessage("\n");
		PrintMessage(strings[S_ReadErr],dim+dim2,k);	//"Errore in lettura: word richieste=%d, lette=%d\r\n"
	}
	else PrintMessage(strings[S_Compl],k);	//"completata\n"
//****************** visualize ********************
	for(i=0;i<4;i+=2){
		PrintMessage(strings[S_ChipID],i,dati_hex[dim+i],i+1,dati_hex[dim+i+1]);	//"ID%d: 0x%03X   ID%d: 0x%03X\r\n"
	}
	if(dim2>4){
		PrintMessage(strings[S_BKOsccal],dati_hex[dim+4]);	//"Backup OSCCAL: 0x%03X\r\n"
	}
	PrintMessage("\n");
	PrintMessage(strings[S_CodeMem]);	//"\r\nMemoria programma:\r\n"
	s[0]=0;
	int valid=0,empty=1;
	for(i=0;i<dim;i+=COL){
		valid=0;
		for(j=i;j<i+COL&&j<dim;j++){
			sprintf(t,"%03X ",dati_hex[j]);
			strcat(s,t);
			if(dati_hex[j]<0xfff) valid=1;
		}
		if(valid){
			PrintMessage("%04X: %s\r\n",i,s);
			empty=0;
		}
		s[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	if(dim2>5){
		PrintMessage(strings[S_ConfigResMem]);	//"\r\nMemoria configurazione e riservata:\r\n"
		empty=1;
		for(i=dim;i<dim+dim2;i+=COL){
			valid=0;
			for(j=i;j<i+COL&&j<dim+64;j++){
				sprintf(t,"%03X ",dati_hex[j]);
				strcat(s,t);
				if(dati_hex[j]<0xfff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s\r\n",i,s);
				empty=0;
			}
			s[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
	}
	PrintMessage("\n");
	PrintMessage(strings[S_End],(stop-start)/1000.0);	//"\r\nFine (%.2f s)\r\n"
}

void Write12F5xx(int dim,int OscAddr)
{
// write 12 bit PIC
// dim=program size     max~4300=10CC
// OscAddr=OSCCAL address (saved at the beginning), -1 not to use it
// vdd before vpp
// CONFIG @ 0x7FF upon entering program mode
// BACKUP OSCCAL @ dim+5 (saved at the beginning)
// erase: BULK_ERASE_PROG (1001) +10ms
// write: BEGIN_PROG (1000) + Tprogram 2ms + END_PROG2 (1110);
	int k=0,z=0,i,j,w;
	int errori=0;
	WORD osccal=0xffff,BKosccal=0xffff;
	if(OscAddr>dim) OscAddr=dim-1;
	if(OscAddr==-1) usa_BKosccal=usa_osccal=0;
	if(size<0x1000){
		PrintMessage(strings[S_NoConfigW2]);	//"Impossibile trovare la locazione CONFIG (0xFFF)\r\n"
		return;
	}
	if(saveLog){
		RegFile=fopen(LogFileName,"w");
		time_t rawtime;
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		fprintf(RegFile,"%s\n", asctime (timeinfo) );
		fprintf(RegFile,"Write12F5xx(%d,%d)\n",dim,OscAddr);
	}
	for(i=0;i<size;i++) dati_hex[i]&=0xFFF;
	unsigned int start=GetTickCount();
	bufferU[0]=0;
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T1T2;
	bufferU[j++]=1;						//T1=1u
	bufferU[j++]=100;					//T2=100u
	bufferU[j++]=EN_VPP_VCC;		//entrata program mode
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(OscAddr!=-1){
		for(i=-1;i<OscAddr-0xff;i+=0xff){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0xff;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=OscAddr-i;
		bufferU[j++]=READ_DATA_PROG;	// OSCCAL
		if(OscAddr<dim){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=dim-OscAddr;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x4;				// 400->404
		bufferU[j++]=READ_DATA_PROG;	// backup OSCCAL
	}
	bufferU[j++]=NOP;				//uscita program mode
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
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	if(OscAddr!=-1){
		for(z=4;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(z<DIMBUF-2) osccal=(bufferI[z+1]<<8)+bufferI[z+2];
		for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(z<DIMBUF-2) BKosccal=(bufferI[z+1]<<8)+bufferI[z+2];
		if(osccal==0xffff||BKosccal==0xffff){
			PrintMessage(strings[S_ErrOsccal]);	//"Errore in lettura OSCCAL e BKOSCCAL"
			PrintMessage("\n");
			return;
		}
		PrintMessage(strings[S_Osccal],osccal);	//"OSCCAL: 0x%03X\r\n"
		PrintMessage(strings[S_BKOsccal],BKosccal);	//"Backup OSCCAL: 0x%03X\r\n"
 	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Cancellazione ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;			// entrata program mode
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	if(dim>OscAddr+1){				//12F519 (Flash+EEPROM)
		bufferU[j++]=BULK_ERASE_PROG;	// Bulk erase
		bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
		for(i=-1;i<dim-0xff;i+=0xff){	// 0x43F
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0xff;
		}
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=dim-i-1;
		bufferU[j++]=BULK_ERASE_PROG;	// Bulk erase EEPROM
		bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
		if(programID){
			bufferU[j++]=INC_ADDR;
			bufferU[j++]=BULK_ERASE_PROG;	// Bulk erase
			bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
		}
	}
	else{							//12Fxxx
		if(programID){
			for(i=-1;i<dim-0xff;i+=0xff){
				bufferU[j++]=INC_ADDR_N;
				bufferU[j++]=0xff;
			}
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=dim-i;
			bufferU[j++]=BULK_ERASE_PROG;	// Bulk erase
			bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
		}
		else{
			bufferU[j++]=BULK_ERASE_PROG;	// Bulk erase
			bufferU[j++]=WAIT_T3;			// ritardo T3=10ms
		}
	}
	bufferU[j++]=EN_VPP_VCC;		// uscita program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// ritardo T3=10ms tra uscita e rientro program mode
	bufferU[j++]=EN_VPP_VCC;		// entrata program mode
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=INC_ADDR;				// 7FF->000
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;				//T3=2ms
	bufferU[j++]=2000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(30);
	if(dim>OscAddr+1) msDelay(20);
	read();
	PrintMessage(strings[S_Compl]);	//"completata\r\n"
	if(saveLog)WriteLogIO();
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Scrittura codice ... "
	PrintMessage("   ");
	int dim1=dim;
	if(programID) dim1=dim+5;
	if(usa_BKosccal) dati_hex[OscAddr]=BKosccal;
	else if(usa_osccal) dati_hex[OscAddr]=osccal;
	for(i=k=w=0,j=1;i<dim1;i++){
		if(dati_hex[i]<0xfff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=dati_hex[i]>>8;		//MSB
			bufferU[j++]=dati_hex[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;
			bufferU[j++]=WAIT_T3;				//Tprogram 2ms
			bufferU[j++]=END_PROG2;
			bufferU[j++]=WAIT_T2;				//Tdischarge
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim1-1){
			PrintMessage("\b\b\b%2d%%",i*100/dim); fflush(stdout);
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*3+3);
			w=0;
			read();
			for(z=1;z<DIMBUF-7;z++){
				if(bufferI[z]==INC_ADDR&&dati_hex[k]>=0xfff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+5]==READ_DATA_PROG){
					if (dati_hex[k]!=(bufferI[z+6]<<8)+bufferI[z+7]){
						PrintMessage("\n");
						PrintMessage(strings[S_CodeWError],k,dati_hex[k],(bufferI[z+6]<<8)+bufferI[z+7]);	//"Errore in scrittura all'indirizzo %3X: scritto %03X, letto %03X\r\n"
						errori++;
						if(max_err&&errori>max_err){
							PrintMessage(strings[S_MaxErr],errori);	//"Superato il massimo numero di errori (%d), scrittura interrotta\r\n"
							PrintMessage(strings[S_IntW]);	//"Scrittura interrotta"
							i=dim1;
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
	PrintMessage("\b\b\b");
	errori+=i-k;
	PrintMessage(strings[S_ComplErr],errori);	//"completata, %d errori\r\n"
//****************** write CONFIG ********************
	PrintMessage(strings[S_ConfigW]);	//"Scrittura CONFIG ... "
	int err_c=0;
	bufferU[j++]=NOP;				//uscita program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//10 ms tra uscita e rientro prog. mode
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//entrata program mode
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x5;
	bufferU[j++]=LOAD_DATA_PROG;	//config word
	bufferU[j++]=dati_hex[0xfff]>>8;			//MSB
	bufferU[j++]=dati_hex[0xfff]&0xff;			//LSB
	bufferU[j++]=BEGIN_PROG;
	bufferU[j++]=WAIT_T3;			//Tprogram 2ms
	bufferU[j++]=END_PROG2;
	bufferU[j++]=WAIT_T2;			//Tdischarge
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=NOP;				//uscita program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=SET_CK_D;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	read();
	unsigned int stop=GetTickCount();
	for(z=10;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if (z<(DIMBUF-2)&&dati_hex[0xfff]!=(bufferI[z+1]<<8)+bufferI[z+2]){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr],dati_hex[0xfff],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Errore in scrittura config:\r\ndato scritto %03X, letto %03X\r\n"
		err_c++;
	}
	errori+=err_c;
	if (z>DIMBUF-2){
		PrintMessage("\n");
		PrintMessage(strings[S_ConfigWErr2]);	//"Errore in scrittura CONFIG"
	}
	PrintMessage(strings[S_ComplErr],err_c);	//"completata, %d errori\r\n"
	if(saveLog){
		fprintf(RegFile,strings[S_Log8],i,i,k,k,errori);	//"i=%d, k=%d, errori=%d\n"
		WriteLogIO();
	}
	if(saveLog&&RegFile) fclose(RegFile);
	PrintMessage(strings[S_EndErr],(stop-start)/1000.0,errori,errori!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nFine (%.2f s) %d %s\r\n\r\n"
}

