/*
 * fileIO.c - file read and write
 * Copyright (C) 2010 Alberto Maccioni
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


void Save(char* dev,char* savefile){
	FILE* f=fopen(savefile,"w");
	if(!f) return;
//**************** 10-16F *******************************************
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		char str[512],str1[512]="";
		int i;
		fprintf(f,":020000040000FA\n");			//extended address=0
		int sum=0,count=0,s;
		for(i=0;i<size&&dati_hex[i]>=0xfff;i++); //toglie fff in cima
		for(;i<size;i++){
			sum+=(dati_hex[i]>>8)+dati_hex[i]&0xff;
			sprintf(str,"%02X%02X",dati_hex[i]&0xff,dati_hex[i]>>8);
			strcat(str1,str);
			count++;
			if(count==8||i==size-1){
				for(s=i;s>i-count&&dati_hex[s]>=0xfff;s--){	//toglie fff in coda
					sum-=(dati_hex[s]>>8)+dati_hex[s]&0xff;
					str1[strlen(str1)-4]=0;
				}
				count-=i-s;
				sum+=count*2+(((s-count+1)*2)&0xff)+(((s-count+1)*2)>>8);
				if(count) fprintf(f,":%02X%04X00%s%02X\n",count*2,(s-count+1)*2,str1,(-sum)&0xff);
				str1[0]=0;
				count=sum=0;
			}
		}
		fprintf(f,":00000001FF\n");
	}
//**************** 18F *******************************************
	else if(!strncmp(dev,"18F",3)){
		char str[512],str1[512]="";
		int i,ext=0,base;
		fprintf(f,":020000040000FA\n");			//extended address=0
		int sum=0,count=0,s;
		for(i=0;i<size&&memCODE[i]==0xff;i++); //remove leading 0xFF
		for(;i<size;i++){
			sum+=memCODE[i];
			sprintf(str,"%02X",memCODE[i]);
			strcat(str1,str);
			count++;
			if(count==16||i==size-1){
				base=i+1-count;
				for(s=i;s>=base&&memCODE[s]==0xff;s--){	//remove trailing 0xFF
					sum-=memCODE[s];
					str1[strlen(str1)-2]=0;
				}
				count-=i-s;
				sum+=count+(base&0xff)+(base>>8)&0xff;
				if(base>>16>ext){
					ext=base>>16;
					fprintf(f,":02000004%04X%02X\n",ext,(-6-ext)&0xff);
				}
				if(count){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		for(i=0,count=sum=0;i<8;i++){
			sum+=memID[i];
			sprintf(str,"%02X",memID[i]&0xff);
			strcat(str1,str);
			count++;
			if(count==7){
				fprintf(f,":020000040020DA\n");
				for(s=i;s>i-count&&memID[s]>=0xff;s--){	//remove trailing 0xFF
					sum-=memID[s]&0xff;
					str1[strlen(str1)-2]=0;
				}
				count-=i-s;
				sum+=count+(s-count+1)&0xff+((s-count+1)>>8);
				if(count){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		for(i=0,count=sum=0;i<14;i++){
			sum+=memCONFIG[i];
			sprintf(str,"%02X",memCONFIG[i]&0xff);
			strcat(str1,str);
			count++;
			if(count==13){
				fprintf(f,":020000040030CA\n");
				for(s=i;s>i-count&&memCONFIG[s]>=0xff;s--){	//remove trailing 0xFF
					sum-=memCONFIG[s]&0xff;
					str1[strlen(str1)-2]=0;
				}
				count-=i-s;
				sum+=count+(s-count+1)&0xff+((s-count+1)>>8);
				if(count){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		if(sizeEE){
			fprintf(f,":0200000400F00A\n");
			for(i=0,count=sum=0;i<sizeEE;i++){
				sum+=memEE[i];
				sprintf(str,"%02X",memEE[i]&0xff);
				strcat(str1,str);
				count++;
				if(count==16||i==sizeEE-1){
					for(s=i;s>i-count&&memEE[s]>=0xff;s--){	//remove trailing 0xFF
						sum-=memEE[s]&0xff;
						str1[strlen(str1)-2]=0;
					}
					count-=i-s;
					sum+=count+(s-count+1)&0xff+((s-count+1)>>8);
					if(count){
						fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
					}
					str1[0]=0;
					count=sum=0;
				}
			}
		}
		fprintf(f,":00000001FF\n");
	}
//**************** 24F *******************************************
	else if(!strncmp(dev,"24F",3)){
		char str[512],str1[512]="";
		int i,ext=0,base;
		int valid;
		fprintf(f,":020000040000FA\n");			//extended address=0
		int sum=0,count=0,s,word;
		word=memCODE[0]+(memCODE[1]<<8)+(memCODE[2]<<16)+(memCODE[3]<<24);
		for(i=0;i<size&&word==0xffffffff;i+=4) //remove leading 0xFFFFFFFF
			word=memCODE[i]+(memCODE[i+1]<<8)+(memCODE[i+2]<<16)+(memCODE[i+3]<<24);
		for(;i<size;i++){
			sum+=memCODE[i];
			sprintf(str,"%02X",memCODE[i]);
			strcat(str1,str);
			count++;
			if(count==16||i==size-1){
				base=i+1-count;
				for(s=base,valid=0;s<=i&&!valid;s+=4){	//remove empty lines
					if(memCODE[s]<0xFF||memCODE[s+1]<0xFF||+memCODE[s+2]<0xFF) valid=1;
				}
				s=i;
				sum+=count+(base&0xff)+(base>>8)&0xff;
				if(base>>16>ext){
					ext=base>>16;
					fprintf(f,":02000004%04X%02X\n",ext,(-6-ext)&0xff);
				}
				if(count&&valid){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		if(sizeCONFIG){
			fprintf(f,":0200000401F009\n");
			for(i=0,count=sum=0;i<sizeCONFIG&&i<34;i++){
				sum+=memCONFIG[i];
				sprintf(str,"%02X",memCONFIG[i]);
				strcat(str1,str);
				count++;
				if(count==4||i==sizeCONFIG-1){
					s=i;
					sum+=count+(s-count+1)&0xff+((s-count+1)>>8);
					if(count){
						fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
					}
					str1[0]=0;
					count=sum=0;
				}
			}
		}
		if(sizeEE){
			fprintf(f,":0200000400FFFB\n");
			for(i=0,count=sum=0;i<sizeEE;i++){
				sum+=memEE[i];
				sprintf(str,"%02X",memEE[i]&0xff);
				strcat(str1,str);
				count++;
				if(count==16||i==sizeEE-1){
					s=i;
					sum+=0xfc+count+((s-count+1)&0xff)+((s-count+1)>>8);
					if(count&&valid){
						fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1)+0xFC00,str1,(-sum)&0xff);
					}
					str1[0]=0;
					count=sum=0;
				}
			}
		}
		fprintf(f,":00000001FF\n");
	}
//**************** ATMEL *******************************************
	else if(!strncmp(dev,"AT",2)){
		char str[512],str1[512]="";
		int i,ext=0,base;
		fprintf(f,":020000040000FA\n");			//extended address=0
		int sum=0,count=0,s;
		for(i=0;i<size&&memCODE[i]==0xff;i++); //remove leading 0xFF
		for(;i<size;i++){
			sum+=memCODE[i];
			sprintf(str,"%02X",memCODE[i]);
			strcat(str1,str);
			count++;
			if(count==16||i==size-1){
				base=i+1-count;
				for(s=i;s>=base&&memCODE[s]==0xff;s--){	//remove trailing 0xFF
					sum-=memCODE[s];
					str1[strlen(str1)-2]=0;
				}
				count-=i-s;
				sum+=count+(base&0xff)+(base>>8)&0xff;
				if(base>>16>ext){
					ext=base>>16;
					fprintf(f,":02000004%04X%02X\n",ext,(-6-ext)&0xff);
				}
				if(count){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		fprintf(f,":00000001FF\n");
	}
//**************** 24xxx / 93xxx / 25xxx *******************************************
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"93",2)||!strncmp(dev,"25",2)){
		if(strstr(savefile,".bin")||strstr(loadfile,".BIN")){
			fwrite(memEE,1,sizeEE,f);
		}
		else{			//HEX
			char str[512],str1[512]="";
			int i,ext=0,base;
			fprintf(f,":020000040000FA\n");			//extended address=0
			int sum=0,count=0;
			for(i=0;i<sizeEE;i++){
				sum+=memEE[i];
				sprintf(str,"%02X",memEE[i]);
				strcat(str1,str);
				count++;
				if(count==16||i==sizeEE-1){
					base=i+1-count;
					sum+=count+(base&0xff)+(base>>8)&0xff;
					if(base>>16>ext){
						ext=base>>16;
						fprintf(f,":02000004%04X%02X\n",ext,(-6-ext)&0xff);
					}
					if(count){
						fprintf(f,":%02X%04X00%s%02X\n",count,(i-count+1),str1,(-sum)&0xff);
					}
					str1[0]=0;
					count=sum=0;
				}
			}
			fprintf(f,":00000001FF\n");
		}
	}
}

void SaveEE(char* dev,char* savefile){
	FILE* f=fopen(savefile,"w");
	if(!f) return;
//**************** ATMEL *******************************************
	if(!strncmp(dev,"AT",2)){
		char str[512],str1[512]="";
		int i,ext=0,base;
		fprintf(f,":020000040000FA\n");			//extended address=0
		int sum=0,count=0,s;
		for(i=0,count=sum=0;i<sizeEE;i++){
			sum+=memEE[i];
			sprintf(str,"%02X",memEE[i]&0xff);
			strcat(str1,str);
			count++;
			if(count==16||i==sizeEE-1){
				for(s=i;s>i-count&&memEE[s]>=0xff;s--){	//remove trailing 0xFF
					sum-=memEE[s]&0xff;
					str1[strlen(str1)-2]=0;
				}
				count-=i-s;
				sum+=count+(s-count+1)&0xff+((s-count+1)>>8);
				if(count){
					fprintf(f,":%02X%04X00%s%02X\n",count,(s-count+1),str1,(-sum)&0xff);
				}
				str1[0]=0;
				count=sum=0;
			}
		}
		fprintf(f,":00000001FF\n");
	}
}

int Load(char*dev,char*loadfile){
	FILE* f=fopen(loadfile,"r");
	int i,j;
	if(!f) return -1;
//**************** 10-16F *******************************************
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		char line[256];
		unsigned char buffer[0x5000];
		memset(buffer,0xFF,sizeof(buffer));
		int input_address=0;
		int end_address=0;
		for(;fgets(line,256,f);){
			if(strlen(line)>9){
				int hex_count = htoi(line+1, 2);
				if (strlen(line) - 11 < hex_count * 2) {
					PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
				}
				else{
					input_address=htoi(line+3,4);
					int sum = 0;
					for (i=1;i<=hex_count*2+9;i+=2)
						sum += htoi(line+i,2);
					if ((sum & 0xff)!=0) {
						PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
					}
					else{
						if (htoi(line+7,2)==0){
							end_address=input_address+hex_count;
							if(end_address>=0x5000) return;	//troppo lungo
							for (i=0;i<hex_count;i++){
								buffer[input_address+i]=htoi(line+9+i*2,2);
							}
						}
					}
				}
			}
		}
		size=end_address/2;
		dati_hex=malloc(sizeof(WORD)*size);
		for(i=0;i<size;i++){		//Swap bytes
			dati_hex[i]=(buffer[i*2+1]<<8)+buffer[i*2];
		}
		for(i=0x2100;i<size;i++)dati_hex[i]&=0xFF;
		char s[256],t[256],v[256];
		PrintMessage("%s :\n",loadfile);
		PrintMessage("\n");
		PrintMessage(strings[S_CodeMem]);	//"\r\nMemoria programma:\r\n"
		PrintMessage("\n");
		s[0]=0;
		for(i=0;i<size&&i<0x2100;i+=COL){
			int valid=0;
			for(j=i;j<i+COL&&j<size&&i<0x2100;j++){
				sprintf(t,"%04X ",dati_hex[j]);
				strcat(s,t);
				if(dati_hex[j]<0x3fff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s\n",i,s);
			}
			s[0]=0;
		}
		PrintMessage("\n\n");
		if(size>=0x2100) PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
		v[0]=0;
		for(i=0x2100;i<0x2800&&i<size;i+=COL){
			int valid=0;
			for(j=i;j<i+COL&&i<0x2800&&i<size;j++){
				sprintf(t,"%02X ",dati_hex[j]);
				strcat(s,t);
				sprintf(t,"%c",isprint(dati_hex[j])?dati_hex[j]:'.');
				strcat(v,t);
				if(dati_hex[j]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s %s\r\n",i,s,v);
			}
			s[0]=0;
			v[0]=0;
		}
		PrintMessage("\n");
	}
//**************** 18F *******************************************
	else if(!strncmp(dev,"18F",3)){
		char line[256];
		unsigned char buffer[0x30000],bufferEE[0x1000];
		int input_address=0,ext_addr=0;
		int i;
		memset(buffer,0xFF,sizeof(buffer));
		memset(bufferEE,0xFF,sizeof(bufferEE));
		memset(memID,0xFF,sizeof(memID));
		memset(memCONFIG,0xFF,sizeof(memCONFIG));
		for(;fgets(line,256,f);){
			if(strlen(line)>9){
				int hex_count = htoi(line+1, 2);
				if (strlen(line) - 11 < hex_count * 2) {
					PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
				}
				else{
					input_address=htoi(line+3,4);
					int sum = 0;
					for (i=1;i<=hex_count*2+9;i+=2)
						sum += htoi(line+i,2);
					if ((sum & 0xff)!=0) {
						PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
					}
					else{
						switch(htoi(line+7,2)){
							case 0:		//Data record
								if(ext_addr<0x20){		//Code
									size=input_address+hex_count;
									for (i=0;i<hex_count;i++){
										buffer[(ext_addr<<16)+input_address+i]=htoi(line+9+i*2,2);
									}
								}
								else if(ext_addr==0x20&&input_address<8){	//ID
									for (i=0;i<hex_count;i++){
										memID[input_address+i]=htoi(line+9+i*2,2);
									}
								}
								else if(ext_addr==0x30&&input_address<14){	//CONFIG
									for (i=0;i<hex_count;i++){
										memCONFIG[input_address+i]=htoi(line+9+i*2,2);
									}
								}
								else if(ext_addr==0xF0&&input_address<0x1000){	//EEPROM
									for (i=0;i<hex_count;i++){
										bufferEE[input_address+i]=htoi(line+9+i*2,2);
									}
									sizeEE=input_address+hex_count;
								}
								break;
							case 4:		//extended linear address record
								if(strlen(line)>14)	ext_addr=htoi(line+9,4);
								break;
							default:
								break;
						}
					}
				}
			}
		}
		memCODE=malloc(size);
		memcpy(memCODE,buffer,size);
		memEE=malloc(sizeEE);
		memcpy(memEE,bufferEE,sizeEE);
		char s[256]="",t[256],v[256]="";
		PrintMessage("%s :\n",loadfile);
		PrintMessage(strings[S_CodeMem]);	//"\r\nmemoria CODICE:\r\n"
		for(i=0;i<size;i+=COL*2){
			int valid=0;
			for(j=i;j<i+COL*2&&j<size;j++){
				sprintf(t,"%02X ",memCODE[j]);
				strcat(s,t);
				if(memCODE[j]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s\n",i,s);
			}
			s[0]=0;
		}
		PrintMessage("\n");
		PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
		for(i=0;i<sizeEE;i+=COL){
			int valid=0;
			for(j=i;j<i+COL&&j<size;j++){
				sprintf(t,"%02X ",memEE[j]);
				strcat(s,t);
				sprintf(t,"%c",isprint(memEE[j])?memEE[j]:'.');
				strcat(v,t);
				if(memEE[j]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s %s\n",i,s,v);
			}
			s[0]=0;
			v[0]=0;
		}
		PrintMessage("\n");
		PrintMessage(strings[S_IDMem]);	//"memoria ID:\r\n"
		for(i=0;i<8;i+=2) PrintMessage("ID%d: 0x%02X   ID%d: 0x%02X\n",i,memID[i],i+1,memID[i+1]);
		PrintMessage(strings[S_ConfigMem]);	//"memoria CONFIG:\r\n"
		for( i=0;i<7;i++){
			PrintMessage("CONFIG%dH: 0x%02X\t",i+1,memCONFIG[i*2+1]);
			PrintMessage("CONFIG%dL: 0x%02X\n",i+1,memCONFIG[i*2]);
		}
		PrintMessage("\n");
	}
//**************** 24F *******************************************
	else if(!strncmp(dev,"24F",3)){
		char line[256];
		unsigned char *buffer,bufferEE[0x2000];
		int input_address=0,ext_addr=0;
		int end_address=0,aa,i;
		buffer=malloc(0x100000);
		memset(buffer,0xFF,0x100000);
		memset(bufferEE,0xFF,sizeof(bufferEE));
		memset(memCONFIG,0xFF,sizeof(memCONFIG));
		for(;fgets(line,256,f);){
			if(strlen(line)>9){
				int hex_count = htoi(line+1, 2);
				if (strlen(line) - 11 < hex_count * 2) {
					PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
				}
				else{
					input_address=htoi(line+3,4);
					int sum = 0;
					for (i=1;i<=hex_count*2+9;i+=2)
						sum += htoi(line+i,2);
					if ((sum & 0xff)!=0) {
						PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
					}
					else{
						switch(htoi(line+7,2)){
							case 0:		//Data record
								if(ext_addr<0x20){		//Code
									int end1=(ext_addr<<16)+input_address+hex_count;
									int end0=size;
									if(end0<end1){			//grow array and fill with 0xFF
										size=end1;
										for(i=end0;i<(ext_addr<<16)+input_address;i++) buffer[i]=0xff;
									}
									end_address=(ext_addr<<16)+input_address+hex_count;
									for (i=0;i<hex_count;i++){
										buffer[(ext_addr<<16)+input_address+i]=htoi(line+9+i*2,2);
									}
								}
								else if(ext_addr==0x1F0&&input_address<0x22){	//CONFIG
									aa=sizeCONFIG;
									sizeCONFIG=input_address+hex_count;
									for(i=aa;i<input_address+hex_count;i++) memCONFIG[i]=0xff;	//fill with 0xFF
									for (i=0;i<hex_count;i++){
										memCONFIG[input_address+i]=htoi(line+9+i*2,2);
									}
								}
								else if(ext_addr==0xFF&&input_address>=0xFC00){	//EEPROM
									for (i=0;i<hex_count;i++){
										bufferEE[input_address-0xFC00+i]=htoi(line+9+i*2,2);
									}
									sizeEE=input_address-0xFC00+hex_count;
								}
								break;
							case 4:		//extended linear address record
								if(strlen(line)>14)	ext_addr=htoi(line+9,4);
								break;
							default:
								break;
						}
					}
				}
			}
		}
		memCODE=malloc(size);
		memcpy(memCODE,buffer,size);
		memEE=malloc(sizeEE);
		memcpy(memEE,bufferEE,sizeEE);
		free(buffer);
		char s[256]="",t[256],v[256]="";
		PrintMessage("%s :\n",loadfile);
		if(sizeCONFIG){
			sizeCONFIG=0x22;
			//for(int i=aa;i<0x22;i++) memCONFIG[i]=0xff;
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
		PrintMessage(strings[S_CodeMem]);	//"\r\nmemoria CODICE:\r\n"
		for(i=0;i<size;i+=COL*2){
			int valid=0,d;
			for(j=i;j<i+COL*2&&j<size;j+=4){
				d=(memCODE[j+3]<<24)+(memCODE[j+2]<<16)+(memCODE[j+1]<<8)+memCODE[j];
				sprintf(t,"%08X ",d);
				strcat(s,t);
				if(d!=0xffffffff) valid=1;
			}
			if(valid){
				PrintMessage("%06X: %s\n",i/2,s);
			}
			s[0]=0;
		}
		PrintMessage("\n");
		if(sizeEE){
			PrintMessage(strings[S_EEMem]);	//"\r\nmemoria EEPROM:\r\n"
			for(i=0;i<sizeEE;i+=COL*2){
				int valid=0;
				for(j=i;j<i+COL*2&&j<size;j+=4){	//skip high word
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
					PrintMessage("%04X: %s %s\n",i/2,s,v);
				}
				s[0]=0;
				v[0]=0;
			}
		}
		PrintMessage("\n");
	}
//**************** ATMEL *******************************************
	else if(!strncmp(dev,"AT",2)){
		char line[256];
		unsigned char buffer[0x30000],bufferEE[0x1000];
		int input_address=0,ext_addr=0;
		int i;
		memset(buffer,0xFF,sizeof(buffer));
		memset(bufferEE,0xFF,sizeof(bufferEE));
		for(;fgets(line,256,f);){
			if(strlen(line)>9){
				int hex_count = htoi(line+1, 2);
				if (strlen(line) - 11 < hex_count * 2) {
					PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
				}
				else{
					input_address=htoi(line+3,4);
					int sum = 0;
					for (i=1;i<=hex_count*2+9;i+=2)
						sum += htoi(line+i,2);
					if ((sum & 0xff)!=0) {
						PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
					}
					else{
						switch(htoi(line+7,2)){
							case 0:		//Data record
								if(ext_addr<0x20){		//Code
									size=input_address+hex_count;
									for (i=0;i<hex_count;i++){
										buffer[(ext_addr<<16)+input_address+i]=htoi(line+9+i*2,2);
									}
								}
								break;
							case 4:		//extended linear address record
								if(strlen(line)>14)	ext_addr=htoi(line+9,4);
								break;
							default:
								break;
						}
					}
				}
			}
		}
		memCODE=malloc(size);
		memcpy(memCODE,buffer,size);
		char s[256]="",t[256],v[256]="";
		PrintMessage("%s :\n",loadfile);
		PrintMessage(strings[S_CodeMem]);	//"\r\nmemoria CODICE:\r\n"
		for(i=0;i<size;i+=COL*2){
			int valid=0;
			for(j=i;j<i+COL*2&&j<size;j++){
				sprintf(t,"%02X ",memCODE[j]);
				strcat(s,t);
				if(memCODE[j]<0xff) valid=1;
			}
			if(valid){
				PrintMessage("%04X: %s\n",i,s);
			}
			s[0]=0;
		}
		PrintMessage("\n");
	}
//**************** 24xxx / 93xxx / 25xxx **************************************
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"93",2)||!strncmp(dev,"25",2)){

		if(strstr(loadfile,".bin")||strstr(loadfile,".BIN")){
			fseek(f, 0L, SEEK_END);
			sizeEE=ftell(f);
			fseek(f, 0L, SEEK_SET);
			if(sizeEE>0x100000) sizeEE=0x100000;
			memEE=malloc(sizeEE);
			sizeEE=fread(memEE,1,sizeEE,f);
		}
		else{			//Hex file
			char line[256];
			unsigned char buffer[0x30000];
			int input_address=0,ext_addr=0;
			int i;
			memset(buffer,0xFF,sizeof(buffer));
			for(;fgets(line,256,f);){
				if(strlen(line)>9){
					int hex_count = htoi(line+1, 2);
					if (strlen(line) - 11 < hex_count * 2) {
						PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
					}
					else{
						input_address=htoi(line+3,4);
						int sum = 0;
						int end0,end1;
						for (i=1;i<=hex_count*2+9;i+=2)
							sum += htoi(line+i,2);
						if ((sum & 0xff)!=0) {
							PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
						}
						else{
							switch(htoi(line+7,2)){
								case 0:		//Data record
									end1=(ext_addr<<16)+input_address+hex_count;
									end0=sizeEE;
									if(end0<end1){			//grow array and fill with 0xFF
										sizeEE=end1;
										for(i=end0;i<(ext_addr<<16)+input_address;i++) buffer[i]=0xff;
									}
									//end_address=(ext_addr<<16)+input_address+hex_count;
									for (i=0;i<hex_count;i++){
										buffer[(ext_addr<<16)+input_address+i]=htoi(line+9+i*2,2);
									}
									break;
								case 4:		//extended linear address record
									if(strlen(line)>14)	ext_addr=htoi(line+9,4);
									break;
								default:
									break;
							}
						}
					}
				}
			}
			memEE=malloc(sizeEE);
			memcpy(memEE,buffer,sizeEE);
		}
		DisplayEE();	//visualize
		PrintMessage("\n");
	}
	return 0;
}

void LoadEE(char*dev,char*loadfile){
	FILE* f=fopen(loadfile,"r");
	int i,j;
	if(!f) return;
//**************** ATMEL *******************************************
	if(!strncmp(dev,"AT",2)){
		char line[256];
		unsigned char bufferEE[0x1000];
		int input_address=0,ext_addr=0;
		int i;
		memset(bufferEE,0xFF,sizeof(bufferEE));
		for(;fgets(line,256,f);){
			if(strlen(line)>9){
				int hex_count = htoi(line+1, 2);
				if (strlen(line) - 11 < hex_count * 2) {
					PrintMessage(strings[S_IhexShort],line);	//"Intel hex8 line too short:\r\n%s\r\n"
				}
				else{
					input_address=htoi(line+3,4);
					int sum = 0;
					for (i=1;i<=hex_count*2+9;i+=2)
						sum += htoi(line+i,2);
					if ((sum & 0xff)!=0) {
						PrintMessage(strings[S_IhexChecksum],line);	//"Intel hex8 checksum error in line:\r\n%s\r\n"
					}
					else{
						switch(htoi(line+7,2)){
							case 0:		//Data record
								if(ext_addr==0&&input_address<0x1000){	//EEPROM
									for (i=0;i<hex_count;i++){
										bufferEE[input_address+i]=htoi(line+9+i*2,2);
									}
									sizeEE=input_address+hex_count;
								}
								break;
							case 4:		//extended linear address record
								if(strlen(line)>14)	ext_addr=htoi(line+9,4);
								break;
							default:
								break;
						}
					}
				}
			}
		}
		memEE=malloc(sizeEE);
		memcpy(memEE,bufferEE,sizeEE);
		DisplayEE();	//visualize
		PrintMessage("\n");
	}
}
