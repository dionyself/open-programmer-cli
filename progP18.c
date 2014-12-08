/**
 * \file progP18F.c - algorithms to program the PIC18 family of microcontrollers
 * Copyright (C) 2009-2014 Alberto Maccioni
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

//This cannot be executed conditionally on MSVC
//#include "stdafx.h"


//configure for GUI or command-line
#ifdef _MSC_VER 
	#define _GUI
	#include "msvc_common.h"
#else 
	#include "common.h"
#endif

struct ID18{
	int id;
	char *device;
	int revmask;
} PIC18LIST[]={
	{0x0400,"18F252/2539 rev%d\r\n",0x1F},
	{0x0420,"18F452/4539 rev%d\r\n",0x1F},
	{0x0480,"18F242/2439 rev%d\r\n",0x1F},
	{0x04A0,"18F442/4439 rev%d\r\n",0x1F},
	{0x0500,"18F2320 rev%d\r\n",0x1F},
	{0x0520,"18F4320 rev%d\r\n",0x1F},
	{0x0580,"18F2220 rev%d\r\n",0x1F},
	{0x05A0,"18F4220 rev%d\r\n",0x1F},
	{0x0600,"18F8720 rev%d\r\n",0x1F},
	{0x0620,"18F6720 rev%d\r\n",0x1F},
	{0x0640,"18F8620 rev%d\r\n",0x1F},
	{0x0660,"18F6620 rev%d\r\n",0x1F},
	{0x07C0,"18F1320 rev%d\r\n",0x1F},
	{0x07E0,"18F1220 rev%d\r\n",0x1F},
	{0x0800,"18F248 rev%d\r\n",0x1F},
	{0x0820,"18F448 rev%d\r\n",0x1F},
	{0x0840,"18F258 rev%d\r\n",0x1F},
	{0x0860,"18F458 rev%d\r\n",0x1F},
	{0x0880,"18F4431 rev%d\r\n",0x1F},
	{0x08A0,"18F4331 rev%d\r\n",0x1F},
	{0x08C0,"18F2431 rev%d\r\n",0x1F},
	{0x08E0,"18F2331 rev%d\r\n",0x1F},
	{0x0B20,"18F6520 rev%d\r\n",0x1F},
	{0x0B00,"18F8520 rev%d\r\n",0x1F},
	{0x0C00,"18F4620 rev%d\r\n",0x1F},
	{0x0C20,"18F4610 rev%d\r\n",0x1F},
	{0x0C40,"18F4525 rev%d\r\n",0x1F},
	{0x0C60,"18F4515 rev%d\r\n",0x1F},
	{0x0C80,"18F2620 rev%d\r\n",0x1F},
	{0x0CA0,"18F2610 rev%d\r\n",0x1F},
	{0x0CC0,"18F2525 rev%d\r\n",0x1F},
	{0x0CE0,"18F2515 rev%d\r\n",0x1F},
	{0x0E80,"18F4680 rev%d\r\n",0x1F},
	{0x0EA0,"18F4585 rev%d\r\n",0x1F},
	{0x0EC0,"18F2680 rev%d\r\n",0x1F},
	{0x0EE0,"18F2585 rev%d\r\n",0x1F},
	{0x1081,"18F4523 rev%d\r\n",0x0F},
	{0x1080,"18F4520 rev%d\r\n",0x0F},
	{0x10A0,"18F4510 rev%d\r\n",0x1F},
	{0x10C1,"18F4423 rev%d\r\n",0x0F},
	{0x10C0,"18F4420 rev%d\r\n",0x0F},
	{0x10E0,"18F4410 rev%d\r\n",0x1F},
	{0x1100,"18F2523 rev%d\r\n",0x0F},
	{0x1100,"18F2520 rev%d\r\n",0x0F},
	{0x1120,"18F2510 rev%d\r\n",0x1F},
	{0x1140,"18F2423 rev%d\r\n",0x0F},
	{0x1140,"18F2420 rev%d\r\n",0x0F},
	{0x1160,"18F2410 rev%d\r\n",0x1F},
	{0x1200,"18F4550 rev%d\r\n",0x1F},
	{0x1220,"18F4455 rev%d\r\n",0x1F},
	{0x1240,"18F2550 rev%d\r\n",0x1F},
	{0x1260,"18F2455 rev%d\r\n",0x1F},
	{0x1340,"18F6527 rev%d\r\n",0x1F},
	{0x1360,"18F8527 rev%d\r\n",0x1F},
	{0x1380,"18F6622 rev%d\r\n",0x1F},
	{0x13A0,"18F8622 rev%d\r\n",0x1F},
	{0x13C0,"18F6627 rev%d\r\n",0x1F},
	{0x13E0,"18F8627 rev%d\r\n",0x1F},
	{0x1400,"18F6722 rev%d\r\n",0x1F},
	{0x1420,"18F8722 rev%d\r\n",0x1F},
	{0x1520,"18F65J10 rev%d\r\n",0x1F},
	{0x1540,"18F65J15 rev%d\r\n",0x1F},
	{0x1560,"18F66J10 rev%d\r\n",0x1F},
	{0x1580,"18F66J15 rev%d\r\n",0x1F},
	{0x15A0,"18F67J10 rev%d\r\n",0x1F},
	{0x15E0,"18F85J10 rev%d\r\n",0x1F},
	{0x1700,"18F85J15 rev%d\r\n",0x1F},
	{0x1720,"18F86J10 rev%d\r\n",0x1F},
	{0x1740,"18F86J15 rev%d\r\n",0x1F},
	{0x1760,"18F87J10 rev%d\r\n",0x1F},
	{0x1800,"18F66J60 rev%d\r\n",0x1F},
	{0x1820,"18F86J60 rev%d\r\n",0x1F},
	{0x1840,"18F96J60 rev%d\r\n",0x1F},
	{0x1A80,"18F4580 rev%d\r\n",0x1F},
	{0x1AA0,"18F4480 rev%d\r\n",0x1F},
	{0x1AC0,"18F2580 rev%d\r\n",0x1F},
	{0x1AE0,"18F2480 rev%d\r\n",0x1F},
	{0x1C00,"18F25J10 rev%d\r\n",0x1F},
	{0x1C20,"18F45J10 rev%d\r\n",0x1F},
	{0x1C40,"18LF25J10 rev%d\r\n",0x1F},
	{0x1C60,"18LF45J10 rev%d\r\n",0x1F},
	{0x1D00,"18F24J10 rev%d\r\n",0x1F},
	{0x1D20,"18F44J10 rev%d\r\n",0x1F},
	{0x1D40,"18LF24J10 rev%d\r\n",0x1F},
	{0x1D60,"18LF44J10 rev%d\r\n",0x1F},
	{0x1E00,"18F1230 rev%d\r\n",0x1F},
	{0x1E20,"18F1330 rev%d\r\n",0x1F},
	{0x1F00,"18F66J65 rev%d\r\n",0x1F},
	{0x1F20,"18F67J60 rev%d\r\n",0x1F},
	{0x1F40,"18F86J65 rev%d\r\n",0x1F},
	{0x1F60,"18F87J60 rev%d\r\n",0x1F},
	{0x1F80,"18F96J65 rev%d\r\n",0x1F},
	{0x1FA0,"18F97J60 rev%d\r\n",0x1F},
	{0x1FE0,"18F1330-ICD rev%d\r\n",0x1F},
	{0x2000,"18F46K20 rev%d\r\n",0x1F},
	{0x2020,"18F26K20 rev%d\r\n",0x1F},
	{0x2040,"18F45K20 rev%d\r\n",0x1F},
	{0x2060,"18F25K20 rev%d\r\n",0x1F},
	{0x2080,"18F44K20 rev%d\r\n",0x1F},
	{0x20A0,"18F24K20 rev%d\r\n",0x1F},
	{0x20C0,"18F43K20 rev%d\r\n",0x1F},
	{0x20E0,"18F23K20 rev%d\r\n",0x1F},
	{0x2100,"18F4321 rev%d\r\n",0x1F},
	{0x2120,"18F2321 rev%d\r\n",0x1F},
	{0x2140,"18F4221 rev%d\r\n",0x1F},
	{0x2160,"18F2221 rev%d\r\n",0x1F},
	{0x2400,"18F4450 rev%d\r\n",0x1F},
	{0x2420,"18F2450 rev%d\r\n",0x1F},
	{0x2700,"18F2682 rev%d\r\n",0x1F},
	{0x2720,"18F2685 rev%d\r\n",0x1F},
	{0x2740,"18F4682 rev%d\r\n",0x1F},
	{0x2760,"18F4685 rev%d\r\n",0x1F},
	{0x2A00,"18F4553 rev%d\r\n",0x1F},
	{0x2A20,"18F4458 rev%d\r\n",0x1F},
	{0x2A40,"18F2553 rev%d\r\n",0x1F},
	{0x2A60,"18F2458 rev%d\r\n",0x1F},
	{0x3800,"18F63J90 rev%d\r\n",0x1F},
	{0x3820,"18F64J90 rev%d\r\n",0x1F},
	{0x3860,"18F65J90 rev%d\r\n",0x1F},
	{0x3880,"18F83J90 rev%d\r\n",0x1F},
	{0x38A0,"18F84J90 rev%d\r\n",0x1F},
	{0x38E0,"18F85J90 rev%d\r\n",0x1F},
	{0x3900,"18F63J11 rev%d\r\n",0x1F},
	{0x3920,"18F64J11 rev%d\r\n",0x1F},
	{0x3960,"18F65J11 rev%d\r\n",0x1F},
	{0x3980,"18F83J11 rev%d\r\n",0x1F},
	{0x39A0,"18F84J11 rev%d\r\n",0x1F},
	{0x39E0,"18F85J11 rev%d\r\n",0x1F},
	{0x4100,"18F65J50 rev%d\r\n",0x1F},
	{0x4140,"18F66J50 rev%d\r\n",0x1F},
	{0x4160,"18F66J55 rev%d\r\n",0x1F},
	{0x4180,"18F67J50 rev%d\r\n",0x1F},
	{0x41A0,"18F85J50 rev%d\r\n",0x1F},
	{0x41E0,"18F86J50 rev%d\r\n",0x1F},
	{0x4200,"18F86J55 rev%d\r\n",0x1F},
	{0x4220,"18F87J50 rev%d\r\n",0x1F},
	{0x4440,"18F66J11 rev%d\r\n",0x1F},
	{0x4460,"18F66J16 rev%d\r\n",0x1F},
	{0x4480,"18F67J11 rev%d\r\n",0x1F},
	{0x44E0,"18F86J11 rev%d\r\n",0x1F},
	{0x4500,"18F86J16 rev%d\r\n",0x1F},
	{0x4520,"18F87J11 rev%d\r\n",0x1F},
	{0x4700,"18LF13K50 rev%d\r\n",0x1F},
	{0x4720,"18LF14K50 rev%d\r\n",0x1F},
	{0x4740,"18F13K50 rev%d\r\n",0x1F},
	{0x4760,"18F14K50 rev%d\r\n",0x1F},
	{0x49C0,"18F6628 rev%d\r\n",0x1F},
	{0x49E0,"18F8628 rev%d\r\n",0x1F},
	{0x4A00,"18F6723 rev%d\r\n",0x1F},
	{0x4A20,"18F8723 rev%d\r\n",0x1F},
	{0x4C00,"18F24J50 rev%d\r\n",0x1F},
	{0x4C20,"18F25J50 rev%d\r\n",0x1F},
	{0x4C40,"18F26J50 rev%d\r\n",0x1F},
	{0x4C60,"18F44J50 rev%d\r\n",0x1F},
	{0x4C80,"18F45J50 rev%d\r\n",0x1F},
	{0x4D80,"18F24J11 rev%d\r\n",0x1F},
	{0x4DA0,"18F25J11 rev%d\r\n",0x1F},
	{0x4DC0,"18F26J11 rev%d\r\n",0x1F},
	{0x4DE0,"18F44J11 rev%d\r\n",0x1F},
	{0x4E00,"18F45J11 rev%d\r\n",0x1F},
	{0x4E20,"18F46J11 rev%d\r\n",0x1F},
	{0x4E60,"18LF25J11 rev%d\r\n",0x1F},
	{0x4E80,"18LF26J11 rev%d\r\n",0x1F},
	{0x4EA0,"18LF44J11 rev%d\r\n",0x1F},
	{0x4EC0,"18LF45J11 rev%d\r\n",0x1F},
	{0x4EE0,"18LF46J11 rev%d\r\n",0x1F},
	{0x4F20,"18F14K22 rev%d\r\n",0x1F},
	{0x4F40,"18F13K22 rev%d\r\n",0x1F},
	{0x4F60,"18LF14K22 rev%d\r\n",0x1F},
	{0x4F80,"18LF13K22 rev%d\r\n",0x1F},
	{0x5000,"18F66J90 rev%d\r\n",0x1F},
	{0x5020,"18F67J90 rev%d\r\n",0x1F},
	{0x5040,"18F66J93/86J72 rev%d\r\n",0x1F},
	{0x5060,"18F67J93/87J72 rev%d\r\n",0x1F},
	{0x5080,"18F86J90 rev%d\r\n",0x1F},
	{0x50A0,"18F87J90 rev%d\r\n",0x1F},
	{0x50C0,"18F86J93 rev%d\r\n",0x1F},
	{0x50E0,"18F87J93 rev%d\r\n",0x1F},
	{0x5400,"18F46K22 rev%d\r\n",0x1F},
	{0x5420,"18LF46K22 rev%d\r\n",0x1F},
	{0x5440,"18F26K22 rev%d\r\n",0x1F},
	{0x5460,"18LF26K22 rev%d\r\n",0x1F},
	{0x5500,"18F45K22 rev%d\r\n",0x1F},
	{0x5520,"18LF45K22 rev%d\r\n",0x1F},
	{0x5540,"18F25K22 rev%d\r\n",0x1F},
	{0x5560,"18LF25K22 rev%d\r\n",0x1F},
	{0x5600,"18F44K22 rev%d\r\n",0x1F},
	{0x5620,"18LF44K22 rev%d\r\n",0x1F},
	{0x5640,"18F24K22 rev%d\r\n",0x1F},
	{0x5660,"18LF24K22 rev%d\r\n",0x1F},
	{0x5700,"18F43K22 rev%d\r\n",0x1F},
	{0x5720,"18LF43K22 rev%d\r\n",0x1F},
	{0x5740,"18F23K22 rev%d\r\n",0x1F},
	{0x5760,"18LF23K22 rev%d\r\n",0x1F},
	{0x5820,"18F26J53 rev%d\r\n",0x1F},
	{0x5860,"18F27J53 rev%d\r\n",0x1F},
	{0x58A0,"18F46J53 rev%d\r\n",0x1F},
	{0x58E0,"18F47J53 rev%d\r\n",0x1F},
	{0x5920,"18F26J13 rev%d\r\n",0x1F},
	{0x5960,"18F27J13 rev%d\r\n",0x1F},
	{0x59A0,"18F46J13 rev%d\r\n",0x1F},
	{0x59E0,"18F47J13 rev%d\r\n",0x1F},
	{0x5A20,"18LF26J53 rev%d\r\n",0x1F},
	{0x5A60,"18LF27J53 rev%d\r\n",0x1F},
	{0x5AA0,"18LF46J53 rev%d\r\n",0x1F},
	{0x5AE0,"18LF47J53 rev%d\r\n",0x1F},
	{0x5B20,"18LF26J13 rev%d\r\n",0x1F},
	{0x5B60,"18LF27J13 rev%d\r\n",0x1F},
	{0x5BA0,"18LF46J13 rev%d\r\n",0x1F},
	{0x5BE0,"18LF47J13 rev%d\r\n",0x1F},
	{0x5C00,"18F45K50 rev%d\r\n",0x1F},
	{0x5C20,"18F25K50 rev%d\r\n",0x1F},
	{0x5C60,"18F24K50 rev%d\r\n",0x1F},
	{0x5C80,"18LF45K50 rev%d\r\n",0x1F},
	{0x5CA0,"18LF25K50 rev%d\r\n",0x1F},
	{0x5CE0,"18LF24K50 rev%d\r\n",0x1F},
	{0x5D00,"18F46K50 rev%d\r\n",0x1F},
	{0x5D20,"18F26K50 rev%d\r\n",0x1F},
	{0x5D40,"18LF46K50 rev%d\r\n",0x1F},
	{0x5D60,"18LF26K50 rev%d\r\n",0x1F},
	{0x60E0,"18F66K80 rev%d\r\n",0x1F},
	{0x6100,"18F46K80 rev%d\r\n",0x1F},
	{0x6120,"18F26K80 rev%d\r\n",0x1F},
	{0x6140,"18F65K80 rev%d\r\n",0x1F},
	{0x6160,"18F45K80 rev%d\r\n",0x1F},
	{0x6180,"18F25K80 rev%d\r\n",0x1F},
	{0x61C0,"18LF66K80 rev%d\r\n",0x1F},
	{0x61E0,"18LF46K80 rev%d\r\n",0x1F},
	{0x6200,"18LF26K80 rev%d\r\n",0x1F},
	{0x6220,"18LF65K80 rev%d\r\n",0x1F},
	{0x6240,"18LF45K80 rev%d\r\n",0x1F},
	{0x6260,"18LF25K80 rev%d\r\n",0x1F},
};

#ifdef _MSC_VER
	void COpenProgDlg::PIC18_ID(int id)
#else
void PIC18_ID(int id)
#endif
{
	char s[64];
	int i;
	for(i=0;i<sizeof(PIC18LIST)/sizeof(PIC18LIST[0]);i++){
		if(PIC18LIST[i].id==(id&~PIC18LIST[i].revmask)){
			sprintf(s,PIC18LIST[i].device,id&PIC18LIST[i].revmask);
			PrintMessage(s);
			return;
		}
	}
			sprintf(s,"%s",strings[S_nodev]); //"Unknown device\r\n");
	PrintMessage(s);
}

#ifdef _MSC_VER
void COpenProgDlg::DisplayCODE18F(int dim){
#else
void DisplayCODE18F(int dim){
#endif
// display  PIC18F CODE memory
	char s[256]="",t[256]="";
	char* aux=(char*)malloc((dim/COL+1)*(16+COL*6));
	aux[0]=0;
	int valid=0,empty=1,i,j,lines=0;
	for(i=0;i<dim&&i<size;i+=COL*2){
		valid=0;
		for(j=i;j<i+COL*2&&j<dim;j++){
			sprintf(t,"%02X ",memCODE[j]);
			strcat(s,t);
			if(memCODE[j]<0xff) valid=1;
	}
		if(valid){
			sprintf(t,"%04X: %s\r\n",i,s);
			strcat(aux,t);
			empty=0;
			lines++;
			if(lines>500){	//limit number of lines printed
				strcat(aux,"(...)\r\n");
				i=(dim<size?dim:size)-COL*4;
				lines=490;
			}
		}
		s[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	else PrintMessage(aux);
	free(aux);
}

#ifdef _MSC_VER
void COpenProgDlg::Read18Fx(int dim,int dim2,int options){
#else
void Read18Fx(int dim,int dim2,int options){
#endif
// read 16 bit PIC 18Fxxxx
// dim=program size 	dim2=eeprom size
// options:
//	bit [3:0]
//   0 = vdd before vpp (12V)
//   1 = vdd before vpp (9V)
//   2 = low voltage entry with 32 bit key
//   3 = vdd before vpp (9V) with 32 bit key
//	bit [7:4]
//     0 = normal control registers address
//     1 = new control registers address (18FxxK80)
	int k=0,k2=0,z=0,i,j;
	int entry=options&0xF;
	int newAddr=(options>>4)&0xF;
	if(dim>0x1FFFFF||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Code size out of limits\r\n"
		return;
	}
	if(dim2>0x800||dim2<0){
		PrintMessage(strings[S_EELim]);	//"EEPROM size out of limits\r\n"
		return;
	}
	if(entry>0&&!CheckV33Regulator()){
		PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
		return;
	}
	double vpp=-1;
	if(entry==0) vpp=12;
	else if(entry==1||entry==3) vpp=8.5;
	if(!StartHVReg(vpp)){
		PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
		return;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Read18F(%d,%d,%d)    (0x%X,0x%X,0x%X)\n",dim,dim2,options,dim,dim2,options);
	}
	size=dim;
	sizeEE=dim2;
	if(memCODE) free(memCODE);
	memCODE=(unsigned char*)malloc(size);		//CODE
	if(memEE) free(memEE);
	memEE=(unsigned char*)malloc(sizeEE);			//EEPROM
	for(j=0;j<8;j++) memID[j]=0xFF;
	for(j=0;j<14;j++) memCONFIG[j]=0xFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	if(entry==2){					//LV entry with key
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
	}
	if(entry==2||entry==3){			//entry with key
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=TX16;
		bufferU[j++]=2;
		bufferU[j++]=0x4D;
		bufferU[j++]=0x43;
		bufferU[j++]=0x48;
		bufferU[j++]=0x50;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
		bufferU[j++]=WAIT_T3;
	}
	bufferU[j++]=WAIT_T3;
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
	bufferU[j++]=0x6A;			//CLRF TBLPTRU
	bufferU[j++]=0xF8;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//CLRF TBLPTRH
	bufferU[j++]=0xF7;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//CLRF TBLPTRL
	bufferU[j++]=0xF6;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(4);
	if(entry==2) msDelay(7);
	read();
	if(saveLog)WriteLogIO();
	for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
	if(z<DIMBUF-3){
		PrintMessage2(strings[S_DevID2],bufferI[z+3],bufferI[z+2]);	//"DevID: 0x%02X%02X\r\n"
		PIC18_ID(bufferI[z+2]+(bufferI[z+3]<<8));
	}
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//code read ...
	PrintStatusSetup();
	if(saveLog)	fprintf(logfile,"Read code\n");
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
		PrintStatus(strings[S_CodeReading2],i*100/(dim+dim2),i);	//"Read: %d%%, addr. %05X"
		if(RWstop) i=dim;
		j=1;
		if(saveLog){
			fprintf(logfile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
			WriteLogIO();
		}
	}
	PrintStatusEnd();
	if(k!=dim){
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ReadCodeErr2],dim,k);	//"Error reading code area, requested %d bytes, read %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
//****************** read config area ********************
	if(saveLog)	fprintf(logfile,"Read config\n");
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
		fprintf(logfile,strings[S_Log7],i,i,k2,k2);	//"i=%d(0x%X), k=%d(0x%X)\n"
		WriteLogIO();
	}
	if(k2!=22){
		PrintMessage2(strings[S_ReadConfigErr],22,k2);	//"Error reading config area, requested %d bytes, read %d\r\n"
	}
//****************** read eeprom ********************
	if(dim2){
		PrintMessage(strings[S_ReadEE]);		//read eeprom ...
		PrintStatusSetup();
		if(saveLog)	fprintf(logfile,"Read EEPROM\n");
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9E;				//EEPGD=0
		bufferU[j++]=newAddr==0?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;				//CFGS=0
		bufferU[j++]=newAddr==0?0xA6:0x7F;
		for(k2=0,i=0;i<dim2;i++){
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;			//MOVLW
			bufferU[j++]=i&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;					//MOVWF
			bufferU[j++]=newAddr==0?0xA9:0x74;	//ADDR
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;		//MOVLW
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;					//MOVWF
			bufferU[j++]=newAddr==0?0xAA:0x75;	//ADDRH
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x80;			//RD=1
			bufferU[j++]=newAddr==0?0xA6:0x7F;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x50;			//MOVF EEDATA,w
			bufferU[j++]=newAddr==0?0xA8:0x73;
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
				PrintStatus(strings[S_CodeReading],(i+dim)*100/(dim+dim2),i);	//"Read: %d%%, addr. %03X"
				if(RWstop) i=dim2;
				j=1;
				if(saveLog){
					fprintf(logfile,strings[S_Log7],i,i,k2,k2);	//"i=%d(0x%X), k=%d(0x%X)\n"
					WriteLogIO();
				}
			}
		}
		PrintStatusEnd();
		if(k2!=dim2){
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ReadEEErr],dim2,k2);	//"Error reading EEPROM area, requested %d bytes, read %d\r\n"
		}
		else PrintMessage(strings[S_Compl]);
	}
	PrintMessage("\r\n");
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
	PrintStatusClear();
//****************** visualize ********************
	for(i=0;i<8;i+=2){
		PrintMessage4(strings[S_ChipID2],i,memID[i],i+1,memID[i+1]);	//"ID%d: 0x%02X   ID%d: 0x%02X\r\n"
	}
	for(i=0;i<7;i++){
		PrintMessage2(strings[S_ConfigWordH],i+1,memCONFIG[i*2+1]);	//"CONFIG%dH: 0x%02X\t"
		PrintMessage2(strings[S_ConfigWordL],i+1,memCONFIG[i*2]);	//"CONFIG%dL: 0x%02X\r\n"
	}
	PrintMessage(strings[S_CodeMem]);	//"\r\nCode memory:\r\n"
	DisplayCODE18F(dim);
	if(dim2){
		DisplayEE();	//visualize
	}
	PrintMessage1(strings[S_End],(stop-start)/1000.0);	//"\r\nEnd (%.2f s)\r\n"
	if(saveLog) CloseLogFile();
}

#ifdef _MSC_VER
void COpenProgDlg::Write18Fx(int dim,int dim2,int wbuf=8,int eraseW1=0x10000,int eraseW2=0x10000,int options=0)
#else
void Write18Fx(int dim,int dim2,int wbuf,int eraseW1,int eraseW2,int options)
#endif
// write 16 bit PIC 18Fxxxx
// dim=program size 	dim2=eeprom size	wbuf=write buffer size {<=64}
// eraseW1=erase word @3C0005	(not used if >= 0x10000)
// eraseW2=erase word @3C0004	(not used if >= 0x10000)
// options:
//	bit [3:0]
//     0 = vdd before vpp (12V)
//     1 = vdd before vpp (9V)
//     2 = low voltage entry with 32 bit key
//     3 = vdd before vpp (9V) with 32 bit key
//	bit [7:4]
//     0 = normal eeprom write algoritm
//     1 = with unlock sequence 55 AA
//	bit [11:8]
//     0 = 15ms erase delay, 1ms code write time, 5ms EE write, 5ms config write
//     1 = 550ms erase delay, 1.2ms code write time, no config or EEPROM
//     2 = 550ms erase delay, 3.4ms code write time, no config or EEPROM
//     3 = separate block erase with 5ms delay, 1ms code write, 5ms EE write, 5ms config write
//         + new control registers address (18FxxK80)
{
	int k=0,k2,z=0,i,j;
	int err=0;
	int EEalgo=(options>>4)&0xF,entry=options&0xF,optWrite=(options>>8)&0xF;
	if(dim>0x1FFFFF||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Code size out of limits\r\n"
		return;
	}
	if(dim2>0x800||dim2<0){
		PrintMessage(strings[S_EELim]);	//"EEPROM size out of limits\r\n"
		return;
	}
	if(wbuf>64){
		PrintMessage(strings[S_WbufLim]);	//"Write buffer size out of limits\r\n"
		return;
	}
	if(entry>0&&!CheckV33Regulator()){
		PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
		return;
	}
	double vpp=-1;
	if(entry==0) vpp=12;
	else if(entry==1||entry==3) vpp=8.5;
	if(!StartHVReg(vpp)){
		PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
		return;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write18F(%d,%d,%d,%d)    (0x%X,0x%X,0x%X,0x%X)\n",dim,dim2,wbuf,options,dim,dim2,wbuf,options);
	}
	if(dim>size) dim=size;
	if(dim%wbuf){			//grow to an integer number of rows
		dim+=wbuf-dim%wbuf;
		j=size;
		if(j<dim){
			size=dim;
			memCODE=(unsigned char*)realloc(memCODE,size);
		for(;j<dim;j++) memCODE[j]=0xFF;
	}
	}
	if(dim2>sizeEE) dim2=sizeEE;
	if(dim<1){
		PrintMessage(strings[S_NoCode]);	//"Empty data area\r\n"
		return;
	}
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
	if(entry==2){					//LV entry with key
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
	}
	if(entry==2||entry==3){			//entry with key
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=TX16;
		bufferU[j++]=2;
		bufferU[j++]=0x4D;
		bufferU[j++]=0x43;
		bufferU[j++]=0x48;
		bufferU[j++]=0x50;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
		bufferU[j++]=WAIT_T3;
	}
	bufferU[j++]=WAIT_T3;
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
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=5100>>8;
	bufferU[j++]=5100&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(3);
	if(entry==2) msDelay(7);
	read();
	if(saveLog)WriteLogIO();
	for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
	if(z<DIMBUF-3){
		PrintMessage2(strings[S_DevID2],bufferI[z+3],bufferI[z+2]);	//"DevID: 0x%02X%02X\r\n"
		PIC18_ID(bufferI[z+2]+(bufferI[z+3]<<8));
	}
	j=1;
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	if(optWrite!=3){	//chip erase
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
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
	if(optWrite==0) msDelay(16);	//bulk erase delay
	else msDelay(550);
		read();
		j=1;
		if(saveLog)WriteLogIO();
	}
	else{	//separate block erase
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 3C
		bufferU[j++]=0x3C;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRU
		bufferU[j++]=0xF8;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRH
		bufferU[j++]=0xF7;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x04 @ 0x3C0004
		bufferU[j++]=0x04;
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x01 @ 0x3C0005
		bufferU[j++]=0x01;
		bufferU[j++]=0x01;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE BLOCK0\n");
			WriteLogIO();
		}
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x04 @ 0x3C0004
		bufferU[j++]=0x04;
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x02 @ 0x3C0005
		bufferU[j++]=0x02;
		bufferU[j++]=0x02;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE BLOCK1\n");
			WriteLogIO();
		}
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x04 @ 0x3C0004
		bufferU[j++]=0x04;
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x04 @ 0x3C0005
		bufferU[j++]=0x04;
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE BLOCK2\n");
			WriteLogIO();
		}
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x04 @ 0x3C0004
		bufferU[j++]=0x04;
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x08 @ 0x3C0005
		bufferU[j++]=0x08;
		bufferU[j++]=0x08;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE BLOCK3\n");
			WriteLogIO();
		}
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x05 @ 0x3C0004
		bufferU[j++]=0x05;
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x00 @ 0x3C0005
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE BOOT BLOCK\n");
			WriteLogIO();
		}
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 4
		bufferU[j++]=0x04;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x02 @ 0x3C0004
		bufferU[j++]=0x02;
		bufferU[j++]=0x02;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 5
		bufferU[j++]=0x05;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x00 @ 0x3C0005
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//MOVLW 6
		bufferU[j++]=0x06;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRL
		bufferU[j++]=0xF6;
		bufferU[j++]=TABLE_WRITE;	//0x80 @ 0x3C0006
		bufferU[j++]=0x80;
		bufferU[j++]=0x80;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=CORE_INS;		//NOP
		bufferU[j++]=0x00;
		bufferU[j++]=0x00;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);	//block erase delay
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,"ERASE CONFIG\n");
			WriteLogIO();
		}
		if(programID){
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x8E;			//EEPGD=1
			bufferU[j++]=0x7F;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x9C;			//CFGS=0
			bufferU[j++]=0x7F;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x84;			//WREN=1
			bufferU[j++]=0x7F;
			bufferU[j++]=CORE_INS;		//ID 0x200000
			bufferU[j++]=0x0E;			//MOVLW
			bufferU[j++]=0x20;			//
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//MOVWF TBLPTRU
			bufferU[j++]=0xF8;			//
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6A;			//CLRF TBLPTRH
			bufferU[j++]=0xF7;			//CLRF TBLPTRH
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6A;			//CLRF TBLPTRL
			bufferU[j++]=0xF6;			//CLRF TBLPTRL
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x88;			//FREE=1
			bufferU[j++]=0x7F;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x82;			//WR=1
			bufferU[j++]=0x7F;
			bufferU[j++]=CORE_INS;		//NOP
			bufferU[j++]=0x00;
			bufferU[j++]=0x00;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(7);	//row erase delay
			read();
			j=1;
			if(saveLog){
				fprintf(logfile,"ERASE ID\n");
				WriteLogIO();
			}
		}
	}
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** prepare write ********************
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x8E;			//EEPGD=1
	bufferU[j++]=optWrite!=3?0xA6:0x7F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x9C;			//CFCGS=0
	bufferU[j++]=optWrite!=3?0xA6:0x7F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x84;			//WREN=1
	bufferU[j++]=optWrite!=3?0xA6:0x7F;
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
	msDelay(1);
	read();
	j=1;
	if(saveLog)WriteLogIO();
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"code write ... "
	PrintStatusSetup();
	int ww;
	double wdly=1.0;
	if(optWrite==1) wdly=1.2;
	if(optWrite==2) wdly=3.4;
	if(saveLog){
		fprintf(logfile,"WRITE CODE\ndim=%d(0x%X)\n",dim,dim);	//
	}
	int valid,i0;
	j=1;
	for(i=k=0;i<dim;){		//write xx instruction words
		if(k==0){				//skip row if empty
			i0=i;
			for(valid=0;!valid&&i<dim;i+=valid?0:wbuf){
				for(k2=0;k2<wbuf&&!valid;k2++) if(memCODE[i+k2]<0xFF) valid=1;
			}
			if(i>=dim) break;
			if(i>i0){				//some rows were skipped; update current address
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;
				bufferU[j++]=(i>>16)&0xFF;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;			//MOVWF TBLPTRU
				bufferU[j++]=0xF8;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;
				bufferU[j++]=(i>>8)&0xFF;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;			//MOVWF TBLPTRH
				bufferU[j++]=0xF7;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;
				bufferU[j++]=i&0xFF;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;			//MOVWF TBLPTRL
				bufferU[j++]=0xF6;
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(2);
				read();
				j=1;
				if(saveLog) WriteLogIO();
			}
		}
		if(DIMBUF-4-j<wbuf-2-k)	ww=(DIMBUF-4-j)/2;	//split data for a total of wbuf-2
		else ww=(wbuf-2-k)/2;
		bufferU[j++]=TBLW_INC_N;
		bufferU[j++]=ww;
		for(z=0;z<ww;z++){
			bufferU[j++]=memCODE[i+1];
			bufferU[j++]=memCODE[i];
			i+=2;
		}
		k+=ww*2;
		if(k==wbuf-2){	//Write row
			if(j>DIMBUF-8){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(3);
				read();
				j=1;
				if(saveLog) WriteLogIO();
			}
			bufferU[j++]=TBLW_PROG_INC;
			bufferU[j++]=memCODE[i+1];
			bufferU[j++]=memCODE[i];
			if(optWrite==1){	//1.2ms
				bufferU[j++]=1200>>8;
				bufferU[j++]=1200&0xFF;
			}
			else if(optWrite==2){	//3.4ms
				bufferU[j++]=3400>>8;
				bufferU[j++]=3400&0xFF;
			}
			else{				//1ms
				bufferU[j++]=1000>>8;
				bufferU[j++]=1000&0xFF;
			}
			i+=2;
			k=0;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		if(k==0) msDelay(wdly);
		read();
		j=1;
		PrintStatus(strings[S_CodeWriting2],i*100/(dim+dim2),i/2);	//"Write: %d%%,addr. %04X"
		if(RWstop) i=dim;
		if(saveLog){
			fprintf(logfile,strings[S_Log7],i,i,k,k);	//"i=%d, k=%d 0=%d\n"
			WriteLogIO();
		}
	}
	PrintStatusEnd();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write ID ********************
	if(programID&&(optWrite==0||optWrite==3)){
		PrintMessage(strings[S_IDW]);	//"Write ID ... "
		if(saveLog)	fprintf(logfile,"Write ID\n");
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8E;			//EEPGD=1
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;			//CFCGS=0
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	ID 0x200000
		bufferU[j++]=0x20;			//TBLPTRU	ID 0x200000
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRH
		bufferU[j++]=0xF7;			//CLRF TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRL
		bufferU[j++]=0xF6;			//CLRF TBLPTRL
		bufferU[j++]=TBLW_INC_N;
		bufferU[j++]=3;
		for(i=0;i<3;i++){
			bufferU[j++]=memID[i*2+1];
			bufferU[j++]=memID[i*2];
		}
		bufferU[j++]=TBLW_PROG;
		bufferU[j++]=memID[i*2+1];
		bufferU[j++]=memID[i*2];
		bufferU[j++]=1000>>8;		//only one write so no need to specify different delay
		bufferU[j++]=1000&0xFF;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(5);
		read();
		j=1;
		if(saveLog){
			fprintf(logfile,strings[S_Log7],i,i,0,0);	//"i=%d, k=%d 0=%d\n"
			WriteLogIO();
		}
		PrintMessage(strings[S_Compl]);	//"completed\r\n"
	}
//****************** write and verify EEPROM ********************
	if(dim2&&(optWrite==0||optWrite==3)){
		PrintMessage(strings[S_EEAreaW]);	//"Write EEPROM ... "
		PrintStatusSetup();
		if(saveLog)	fprintf(logfile,"Write EEPROM\n");
		int errEE=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9E;			//EEPGD=0
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x9C;			//CFGS=0
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x84;			//WREN=1
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		for(i=0;i<dim2&&err<=max_err;i++){
			if(memEE[i]!=0xFF){
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;			//MOVLW
				bufferU[j++]=i&0xFF;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;					//MOVWF
				bufferU[j++]=optWrite!=3?0xA9:0x74;	//ADDR
				if(EEalgo==0){
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;		//MOVLW
					bufferU[j++]=(i>>8)&0xFF;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;					//MOVWF
					bufferU[j++]=optWrite!=3?0xAA:0x75;	//ADDRH
				}
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x0E;			//MOVLW
				bufferU[j++]=memEE[i];
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x6E;					//MOVWF
				bufferU[j++]=optWrite!=3?0xA8:0x73;	//EEDATA
				if(EEalgo==1){				//memory unlock
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;		//MOVLW
					bufferU[j++]=0x55;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;		//MOVWF
					bufferU[j++]=0xA7;			//EECON2
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x0E;		//MOVLW
					bufferU[j++]=0xAA;
					bufferU[j++]=CORE_INS;
					bufferU[j++]=0x6E;		//MOVWF
					bufferU[j++]=0xA7;			//EECON2
				}
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x82;
				bufferU[j++]=optWrite!=3?0xA6:0x7F;	//WR=1
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x00;
				bufferU[j++]=0x00;			//NOP
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x00;
				bufferU[j++]=0x00;			//NOP
				bufferU[j++]=WAIT_T3;		//write delay
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x80;			//RD=1
				bufferU[j++]=optWrite!=3?0xA6:0x7F;
				bufferU[j++]=CORE_INS;
				bufferU[j++]=0x50;			//MOVF EEDATA,w
				bufferU[j++]=optWrite!=3?0xA8:0x73;
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
				PrintStatus(strings[S_CodeWriting],(i+dim)*100/(dim+dim2),i);	//"Scrittura: %d%%, ind. %03X"
				if(RWstop) i=dim2;
				j=1;
				for(z=DIMBUF-1;z&&bufferI[z]!=SHIFT_TABLAT;z--);
				if(z&&memEE[i]!=bufferI[z+1]) errEE++;
				if(saveLog){
					fprintf(logfile,strings[S_Log8],i,i,k,k,errEE);	//"i=%d, k=%d, errors=%d\n"
					WriteLogIO();
				}
			}
		}
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],errEE);	//"completed: %d errors \r\n"
		err+=errEE;
	}
//****************** verify code ********************
	PrintMessage(strings[S_CodeV]);	//"Verify code ... "
	PrintStatusSetup();
	if(saveLog)fprintf(logfile,"VERIFY CODE\n");
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x8E;			//EEPGD=1
	bufferU[j++]=optWrite!=3?0xA6:0x7F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x9C;			//CFCGS=0
	bufferU[j++]=optWrite!=3?0xA6:0x7F;
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//CLRF TBLPTRU
	bufferU[j++]=0xF8;			//CLRF TBLPTRU
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//CLRF TBLPTRH
	bufferU[j++]=0xF7;			//CLRF TBLPTRH
	bufferU[j++]=CORE_INS;
	bufferU[j++]=0x6A;			//CLRF TBLPTRL
	bufferU[j++]=0xF6;			//CLRF TBLPTRL
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog){
		WriteLogIO();
		fprintf(logfile,"\n\n");
	}
	for(i=0,j=1,k=0;i<dim;i+=DIMBUF-4){
		i0=i;
		for(valid=0;!valid&&i<dim;i+=valid?0:DIMBUF-4){		//skip verification if 0xFF
			for(k2=0;k2<DIMBUF-4&&!valid&&i+k2<dim;k2++) if(memCODE[i+k2]<0xFF) valid=1;
		}
		if(i>=dim) break;
		if(i>i0){				//some data was skipped; update current address
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;
			bufferU[j++]=(i>>16)&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//MOVWF TBLPTRU
			bufferU[j++]=0xF8;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;
			bufferU[j++]=(i>>8)&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//MOVWF TBLPTRH
			bufferU[j++]=0xF7;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x0E;
			bufferU[j++]=i&0xFF;
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x6E;			//MOVWF TBLPTRL
			bufferU[j++]=0xF6;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(2);
			read();
			j=1;
			if(saveLog) WriteLogIO();
		}
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=i<dim-(DIMBUF-4)?DIMBUF-4:dim-i;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		if(bufferI[1]==TBLR_INC_N){
			for(z=0;z<bufferI[2]&&z<DIMBUF;z++){
				if(memCODE[i+z]!=bufferI[z+3]){
					PrintMessage4(strings[S_CodeVError],i+z,i+z,memCODE[i+z],bufferI[z+3]);	//"Error writing address %4X: written %02X, read %02X\r\n"
					err++;
				}
				//k++;
			}
		}
		PrintStatus(strings[S_CodeV2],i*100/(dim+dim2),i);	//"Verifica: %d%%, ind. %04X"
		if(RWstop) i=dim;
		j=1;
		if(saveLog){
			fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d\n"
			WriteLogIO();
		}
		if(err>=max_err) break;
	}
	PrintStatusEnd();
	if(i<dim){
		PrintMessage2(strings[S_CodeVError2],dim,i);	//"Error verifying code area, requested %d bytes, read %d\r\n"
	}
	PrintMessage1(strings[S_ComplErr],err);	//"completed: %d errors\r\n"
	if(err>=max_err){
		PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
	}
//****************** verify ID ********************
	if(programID&&err<max_err&&optWrite==0){
		PrintMessage(strings[S_IDV]);	//"Verify ID ... "
		int errID=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	ID 0x200000
		bufferU[j++]=0x20;			//TBLPTRU	ID 0x200000
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//TBLPTRU
		bufferU[j++]=0xF8;			//TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRH
		bufferU[j++]=0xF7;			//CLRF TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRL
		bufferU[j++]=0xF6;			//CLRF TBLPTRL
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=8;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		for(z=0;bufferI[z]!=TBLR_INC_N&&z<DIMBUF;z++);
		for(i=0;i<8;i++) if(memID[i]!=0xFF&&memID[i]!=bufferI[z+i+2]) errID++;
		PrintMessage1(strings[S_ComplErr],errID);	//"completed: %d errors\r\n"
		err+=errID;
		if(err>=max_err){
			PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
		}
		j=1;
		if(saveLog){
			fprintf(logfile,strings[S_Log8],i,i,0,0,err);	//"i=%d, k=%d, errors=%d\n"
			WriteLogIO();
		}
	}
//****************** write CONFIG ********************
	if(err<max_err&&optWrite==0){
		PrintMessage(strings[S_ConfigW]);	//"Write CONFIG ..."
		if(saveLog)	fprintf(logfile,"Write CONFIG\n");
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8E;			//EEPGD=1
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x8C;			//CFCGS=1
		bufferU[j++]=optWrite!=3?0xA6:0x7F;
		bufferU[j++]=CORE_INS;		//CONFIG 0x300000
		bufferU[j++]=0x0E;			//MOVLW
		bufferU[j++]=0x30;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRU
		bufferU[j++]=0xF8;			//MOVWF TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRH
		bufferU[j++]=0xF7;			//CLRF TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRL
		bufferU[j++]=0xF6;			//CLRF TBLPTRL
		for(i=0;i<14;i++){
			if(memCONFIG[i]<0xFF){
				bufferU[j++]=TBLW_PROG;
				bufferU[j++]=0;
				bufferU[j++]=memCONFIG[i];
				bufferU[j++]=5000>>8;
				bufferU[j++]=5000&0xFF;
			}
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x2A;			//INCF
			bufferU[j++]=0xF6;			//TBLPTRL
			i++;
			if(memCONFIG[i]<0xFF){
				bufferU[j++]=TBLW_PROG;
				bufferU[j++]=memCONFIG[i];
				bufferU[j++]=0;
				bufferU[j++]=5000>>8;
				bufferU[j++]=5000&0xFF;
			}
			bufferU[j++]=CORE_INS;
			bufferU[j++]=0x2A;			//INCF
			bufferU[j++]=0xF6;			//TBLPTRL
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(12);
			read();
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,0,0);	//"i=%d, k=%d\n"
				WriteLogIO();
			}
		}
		PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** verify CONFIG ********************
		PrintMessage(strings[S_ConfigV]);	//"Verify CONFIG ... "
		int errC=0;
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x0E;			//TBLPTRU	CONFIG 0x300000
		bufferU[j++]=0x30;			//
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6E;			//MOVWF TBLPTRU
		bufferU[j++]=0xF8;			//MOVWF TBLPTRU
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRH
		bufferU[j++]=0xF7;			//CLRF TBLPTRH
		bufferU[j++]=CORE_INS;
		bufferU[j++]=0x6A;			//CLRF TBLPTRL
		bufferU[j++]=0xF6;			//CLRF TBLPTRL
		bufferU[j++]=TBLR_INC_N;
		bufferU[j++]=14;
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(2);
		read();
		for(z=1;bufferI[z]!=TBLR_INC_N&&z<DIMBUF-16;z++);
		if(z<DIMBUF-16){
			for(i=0;i<14;i++) if(~memCONFIG[i]&bufferI[z+i+2]) errC++;	//error if written 0 and read 1 (~W&R)
		}
		PrintMessage1(strings[S_ComplErr],errC);	//"completed: %d errors\r\n"
		err+=errC;
		if(err>=max_err){
			PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
		}
		j=1;
		if(saveLog){
			fprintf(logfile,strings[S_Log8],i,i,0,0,err);	//"i=%d, k=%d, errors=%d\n"
			WriteLogIO();
		}
	}
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
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
	PrintStatusClear();
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog) CloseLogFile();
}
