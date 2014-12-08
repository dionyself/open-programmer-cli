/**
 * \file progP16.c - algorithms to program the PIC16 (14 bit word) family of microcontrollers
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

struct ID16{
	int id;
	char *device;
	int revmask;
} PIC16LIST[]={
	{0x00A>>1,"16F72 rev%d\r\n",0x1F},			//00 0000 101x xxxx
	{0x046>>1,"12F683 rev%d\r\n",0x1F},			//00 0100 011x xxxx
	{0x04A>>1,"16F685 rev%d\r\n",0x1F},			//00 0100 101x xxxx
	{0x04C>>1,"16F818 rev%d\r\n",0x1F},			//00 0100 110x xxxx
	{0x04E>>1,"16F819 rev%d\r\n",0x1F},			//00 0100 111x xxxx
	{0x056>>1,"16F84A rev%d\r\n",0x1F}, 		//00 0101 011x xxxx
	{0x060>>1,"16F73 rev%d\r\n",0x1F},  		//00 0110 000x xxxx
	{0x062>>1,"16F74 rev%d\r\n",0x1F}, 	 		//00 0110 001x xxxx
	{0x064>>1,"16F76 rev%d\r\n",0x1F},			//00 0110 010x xxxx
	{0x066>>1,"16F77 rev%d\r\n",0x1F},			//00 0110 011x xxxx
	{0x072>>1,"16F87 rev%d\r\n",0x1F},			//00 0111 001x xxxx
	{0x076>>1,"16F88 rev%d\r\n",0x1F},			//00 0111 011x xxxx
	{0x07A>>1,"16F627 rev%d\r\n",0x1F},			//00 0111 101x xxxx
	{0x07C>>1,"16F628 rev%d\r\n",0x1F},			//00 0111 110x xxxx
	{0x08E>>1,"16F872 rev%d\r\n",0x1F},			//00 1000 111x xxxx
	{0x092>>1,"16F874 rev%d\r\n",0x1F},			//00 1001 001x xxxx
	{0x096>>1,"16F873 rev%d\r\n",0x1F},			//00 1001 011x xxxx
	{0x09A>>1,"16F877 rev%d\r\n",0x1F},			//00 1001 101x xxxx
	{0x09E>>1,"16F876 rev%d\r\n",0x1F},			//00 1001 111x xxxx
	{0x0BA>>1,"16F737 rev%d\r\n",0x1F},			//00 1011 101x xxxx
	{0x0BE>>1,"16F747 rev%d\r\n",0x1F},			//00 1011 111x xxxx
	{0x0D0>>1,"16F870 rev%d\r\n",0x1F},			//00 1101 000x xxxx
	{0x0D2>>1,"16F871 rev%d\r\n",0x1F},			//00 1101 001x xxxx
	{0x0DE>>1,"16F777 rev%d\r\n",0x1F},			//00 1101 111x xxxx
	{0x0E0>>1,"16F876A rev%d\r\n",0xF},			//00 1110 0000 xxxx
	{0x0E2>>1,"16F877A rev%d\r\n",0xF},			//00 1110 0010 xxxx
	{0x0E4>>1,"16F873A rev%d\r\n",0xF},			//00 1110 0100 xxxx
	{0x0E6>>1,"16F874A rev%d\r\n",0xF},			//00 1110 0110 xxxx
	{0x0EA>>1,"16F767 rev%d\r\n",0x1F},			//00 1110 101x xxxx
	{0x0F8>>1,"12F629 rev%d\r\n",0x1F},			//00 1111 100x xxxx
	{0x0FA>>1,"12F635 rev%d\r\n",0x1F},			//00 1111 101x xxxx
	{0x0FC>>1,"12F675 rev%d\r\n",0x1F},			//00 1111 110x xxxx
	{0x104>>1,"16F627A rev%d\r\n",0x1F},		//01 0000 010x xxxx
	{0x106>>1,"16F628A rev%d\r\n",0x1F},		//01 0000 011x xxxx
	{0x108>>1,"16F684 rev%d\r\n",0x1F},			//01 0000 100x xxxx
	{0x110>>1,"16F648A rev%d\r\n",0x1F},		//01 0001 000x xxxx
	{0x10A>>1,"16F636-639 rev%d\r\n",0x1F},		//01 0000 101x xxxx
	{0x10C>>1,"16F630 rev%d\r\n",0x1F},			//01 0000 110x xxxx
	{0x10E>>1,"16F676 rev%d\r\n",0x1F},			//01 0000 111x xxxx
	{0x114>>1,"16F716 rev%d\r\n",0x1F},			//01 0001 010x xxxx
	{0x118>>1,"16F688 rev%d\r\n",0x1F},			//01 0001 100x xxxx
	{0x120>>1,"16F785 rev%d\r\n",0x1F},			//01 0010 000x xxxx
	{0x122>>1,"16HV785 rev%d\r\n",0x1F},		//01 0010 001x xxxx
	{0x124>>1,"16F616 rev%d\r\n",0x1F},			//01 0010 010x xxxx
	{0x126>>1,"16HV616 rev%d\r\n",0x1F},		//01 0010 011x xxxx
	{0x132>>1,"16F687 rev%d\r\n",0x1F},			//01 0011 001x xxxx
	{0x134>>1,"16F689 rev%d\r\n",0x1F},			//01 0011 010x xxxx
	{0x138>>1,"16F917 rev%d\r\n",0xF},			//01 0011 1000 xxxx
	{0x13A>>1,"16F916 rev%d\r\n",0xF},			//01 0011 1010 xxxx
	{0x13C>>1,"16F914 rev%d\r\n",0xF},			//01 0011 1100 xxxx
	{0x13E>>1,"16F913 rev%d\r\n",0xF},			//01 0011 1110 xxxx
	{0x140>>1,"16F690 rev%d\r\n",0x1F},			//01 0100 000x xxxx
	{0x142>>1,"16F631 rev%d\r\n",0x1F},			//01 0100 001x xxxx
	{0x144>>1,"16F677 rev%d\r\n",0x1F},			//01 0100 010x xxxx
	{0x146>>1,"16F946 rev%d\r\n",0xF},			//01 0100 0110 xxxx
	{0x148>>1,"16F1847 rev%d\r\n",0x1F},		//01 0100 100x xxxx
	{0x14A>>1,"16LF1847 rev%d\r\n",0x1F},		//01 0100 101x xxxx
	{0x150>>1,"12F752 rev%d\r\n",0x1F},			//01 0101 000x xxxx
	{0x152>>1,"12HV752 rev%d\r\n",0x1F},		//01 0101 001x xxxx
	{0x158>>1,"16F1526 rev%d\r\n",0x1F},		//01 0101 100x xxxx
	{0x15A>>1,"16F1527 rev%d\r\n",0x1F},		//01 0101 101x xxxx
	{0x15C>>1,"16LF1526 rev%d\r\n",0x1F},		//01 0101 110x xxxx
	{0x15E>>1,"16LF1527 rev%d\r\n",0x1F},		//01 0101 111x xxxx
	{0x164>>1,"16F1513 rev%d\r\n",0x1F},		//01 0110 010x xxxx
	{0x168>>1,"16F1516 rev%d\r\n",0x1F},		//01 0110 100x xxxx
	{0x16A>>1,"16F1517 rev%d\r\n",0x1F},		//01 0110 101x xxxx
	{0x16C>>1,"16F1518 rev%d\r\n",0x1F},		//01 0110 110x xxxx
	{0x16E>>1,"16F1519 rev%d\r\n",0x1F},		//01 0110 111x xxxx
	{0x170>>1,"16F1512 rev%d\r\n",0x1F},		//01 0111 000x xxxx
	{0x172>>1,"16LF1512 rev%d\r\n",0x1F},		//01 0111 001x xxxx
	{0x174>>1,"16LF1513 rev%d\r\n",0x1F},		//01 0111 010x xxxx
	{0x178>>1,"16LF1516 rev%d\r\n",0x1F},		//01 0111 100x xxxx
	{0x17A>>1,"16LF1517 rev%d\r\n",0x1F},		//01 0111 101x xxxx
	{0x17C>>1,"16LF1518 rev%d\r\n",0x1F},		//01 0111 110x xxxx
	{0x17E>>1,"16LF1519 rev%d\r\n",0x1F},		//01 0111 111x xxxx
	{0x180>>1,"16F727 rev%d\r\n",0x1F},			//01 1000 000x xxxx
	{0x182>>1,"16F726 rev%d\r\n",0x1F},			//01 1000 001x xxxx
	{0x184>>1,"16F724 rev%d\r\n",0x1F},			//01 1000 010x xxxx
	{0x186>>1,"16F723 rev%d\r\n",0x1F},			//01 1000 011x xxxx
	{0x188>>1,"16F722 rev%d\r\n",0x1F},			//01 1000 100x xxxx
	{0x190>>1,"16LF727 rev%d\r\n",0x1F},		//01 1001 000x xxxx
	{0x192>>1,"16LF726 rev%d\r\n",0x1F},		//01 1001 001x xxxx
	{0x194>>1,"16LF724 rev%d\r\n",0x1F},		//01 1001 010x xxxx
	{0x196>>1,"16LF723 rev%d\r\n",0x1F},		//01 1001 011x xxxx
	{0x198>>1,"16LF722 rev%d\r\n",0x1F},		//01 1001 100x xxxx
	{0x1AC>>1,"16F707 rev%d\r\n",0x1F},			//01 1010 110x xxxx
	{0x1AE>>1,"16LF707 rev%d\r\n",0x1F},		//01 1010 111x xxxx
	{0x1B0>>1,"16F723A rev%d\r\n",0x1F},		//01 1011 000x xxxx
	{0x1B2>>1,"16F722A rev%d\r\n",0x1F},		//01 1011 001x xxxx
	{0x1B4>>1,"16LF723A rev%d\r\n",0x1F},		//01 1011 010x xxxx
	{0x1B6>>1,"16LF722A rev%d\r\n",0x1F},		//01 1011 011x xxxx
	{0x1B8>>1,"12F1840 rev%d\r\n",0x1F},		//01 1011 100x xxxx
	{0x1BA>>1,"12LF1840 rev%d\r\n",0x1F},		//01 1011 101x xxxx
	{0x1C0>>1,"16F720 rev%d\r\n",0x1F},			//01 1100 000x xxxx
	{0x1C2>>1,"16F721 rev%d\r\n",0x1F},			//01 1100 001x xxxx
	{0x1C4>>1,"16LF720 rev%d\r\n",0x1F},		//01 1100 010x xxxx
	{0x1C6>>1,"16LF721 rev%d\r\n",0x1F},		//01 1100 011x xxxx
	{0x200>>1,"16F882 rev%d\r\n",0x1F},			//10 0000 000x xxxx
	{0x202>>1,"16F883 rev%d\r\n",0x1F},			//10 0000 001x xxxx
	{0x204>>1,"16F884 rev%d\r\n",0x1F},			//10 0000 010x xxxx
	{0x206>>1,"16F886 rev%d\r\n",0x1F},			//10 0000 011x xxxx
	{0x208>>1,"16F887 rev%d\r\n",0x1F},			//10 0000 100x xxxx
	{0x218>>1,"12F615 rev%d\r\n",0x1F},			//10 0001 100x xxxx
	{0x21A>>1,"12HV615 rev%d\r\n",0x1F},		//10 0001 101x xxxx
	{0x224>>1,"12F609 rev%d\r\n",0x1F},			//10 0010 010x xxxx
	{0x226>>1,"16F610 rev%d\r\n",0x1F},			//10 0010 011x xxxx
	{0x228>>1,"12HV609 rev%d\r\n",0x1F},		//10 0010 100x xxxx
	{0x22A>>1,"16HV610 rev%d\r\n",0x1F},		//10 0010 101x xxxx
	{0x232>>1,"16F1933 rev%d\r\n",0x1F},		//10 0011 001x xxxx
	{0x234>>1,"16F1934 rev%d\r\n",0x1F},		//10 0011 010x xxxx
	{0x236>>1,"16F1936 rev%d\r\n",0x1F},		//10 0011 011x xxxx
	{0x238>>1,"16F1937 rev%d\r\n",0x1F},		//10 0011 100x xxxx
	{0x23A>>1,"16F1938 rev%d\r\n",0x1F},		//10 0011 101x xxxx
	{0x23C>>1,"16F1939 rev%d\r\n",0x1F},		//10 0011 110x xxxx
	{0x242>>1,"16LF1933 rev%d\r\n",0x1F},		//10 0100 001x xxxx
	{0x244>>1,"16LF1934 rev%d\r\n",0x1F},		//10 0100 010x xxxx
	{0x246>>1,"16LF1936 rev%d\r\n",0x1F},		//10 0100 011x xxxx
	{0x248>>1,"16LF1937 rev%d\r\n",0x1F},		//10 0100 100x xxxx
	{0x24A>>1,"16LF1938 rev%d\r\n",0x1F},		//10 0100 101x xxxx
	{0x24C>>1,"16LF1939 rev%d\r\n",0x1F},		//10 0100 110x xxxx
	{0x250>>1,"16F1946 rev%d\r\n",0x1F},		//10 0101 000x xxxx
	{0x252>>1,"16F1947 rev%d\r\n",0x1F},		//10 0101 001x xxxx
	{0x258>>1,"16LF1946 rev%d\r\n",0x1F},		//10 0101 100x xxxx
	{0x25A>>1,"16LF1947 rev%d\r\n",0x1F},		//10 0101 101x xxxx
	{0x270>>1,"16F1822 rev%d\r\n",0x1F},		//10 0111 000x xxxx
	{0x272>>1,"16F1823 rev%d\r\n",0x1F},		//10 0111 001x xxxx
	{0x274>>1,"16F1824 rev%d\r\n",0x1F},		//10 0111 010x xxxx
	{0x276>>1,"16F1825 rev%d\r\n",0x1F},		//10 0111 011x xxxx
	{0x278>>1,"16F1826 rev%d\r\n",0x1F},		//10 0111 100x xxxx
	{0x27A>>1,"16F1827 rev%d\r\n",0x1F},		//10 0111 101x xxxx
	{0x280>>1,"16LF1822 rev%d\r\n",0x1F},		//10 1000 000x xxxx
	{0x282>>1,"16LF1823 rev%d\r\n",0x1F},		//10 1000 001x xxxx
	{0x284>>1,"16LF1824 rev%d\r\n",0x1F},		//10 1000 010x xxxx
	{0x286>>1,"16LF1825 rev%d\r\n",0x1F},		//10 1000 011x xxxx
	{0x288>>1,"16LF1826 rev%d\r\n",0x1F},		//10 1000 100x xxxx
	{0x28A>>1,"16LF1827 rev%d\r\n",0x1F},		//10 1000 101x xxxx
	{0x298>>1,"10F322 rev%d\r\n",0x1F},			//10 1001 100x xxxx
	{0x29A>>1,"10F320 rev%d\r\n",0x1F},			//10 1001 101x xxxx
	{0x29C>>1,"10LF322 rev%d\r\n",0x1F},		//10 1001 110x xxxx
	{0x29E>>1,"10LF320 rev%d\r\n",0x1F},		//10 1001 111x xxxx
	{0x2A0>>1,"16F1782 rev%d\r\n",0x1F},		//10 1010 000x xxxx
	{0x2A2>>1,"16F1783 rev%d\r\n",0x1F},		//10 1010 001x xxxx
	{0x2A4>>1,"16F1784 rev%d\r\n",0x1F},		//10 1010 010x xxxx
	{0x2A6>>1,"16F1786 rev%d\r\n",0x1F},		//10 1010 011x xxxx
	{0x2A8>>1,"16F1787 rev%d\r\n",0x1F},		//10 1010 100x xxxx
	{0x2AA>>1,"16LF1782 rev%d\r\n",0x1F},		//10 1010 101x xxxx
	{0x2AC>>1,"16LF1783 rev%d\r\n",0x1F},		//10 1010 110x xxxx
	{0x2AE>>1,"16LF1784 rev%d\r\n",0x1F},		//10 1010 111x xxxx
	{0x2B0>>1,"16LF1786 rev%d\r\n",0x1F},		//10 1011 000x xxxx
	{0x2B2>>1,"16LF1787 rev%d\r\n",0x1F},		//10 1011 001x xxxx
	{0x2C0>>1,"16LF1903 rev%d\r\n",0x1F},		//10 1100 000x xxxx
	{0x2C2>>1,"16LF1902 rev%d\r\n",0x1F},		//10 1100 001x xxxx
	{0x2C4>>1,"16LF1907 rev%d\r\n",0x1F},		//10 1100 010x xxxx
	{0x2C6>>1,"16LF1906 rev%d\r\n",0x1F},		//10 1100 011x xxxx
	{0x2C8>>1,"16LF1904 rev%d\r\n",0x1F},		//10 1100 100x xxxx
	{0x2CC>>1,"12F1501 rev%d\r\n",0x1F},		//10 1100 110x xxxx
	{0x2CE>>1,"16F1503 rev%d\r\n",0x1F},		//10 1100 111x xxxx
	{0x2D0>>1,"16F1507 rev%d\r\n",0x1F},		//10 1101 000x xxxx
	{0x2D2>>1,"16F1508 rev%d\r\n",0x1F},		//10 1101 001x xxxx
	{0x2D4>>1,"16F1509 rev%d\r\n",0x1F},		//10 1101 010x xxxx
	{0x2D8>>1,"12LF1501 rev%d\r\n",0x1F},		//10 1101 100x xxxx
	{0x2DA>>1,"16LF1503 rev%d\r\n",0x1F},		//10 1101 101x xxxx
	{0x2DC>>1,"16LF1507 rev%d\r\n",0x1F},		//10 1101 110x xxxx
	{0x2DE>>1,"16LF1508 rev%d\r\n",0x1F},		//10 1101 111x xxxx
	{0x2E0>>1,"16LF1509 rev%d\r\n",0x1F},		//10 1110 000x xxxx
	{0x2F0>>1,"16LF1554 rev%d\r\n",0x1F},		//10 1111 000x xxxx
	{0x2F2>>1,"16LF1559 rev%d\r\n",0x1F},		//10 1111 001x xxxx
	{0x3000,"16F1574\r\n",0},
	{0x3001,"16F1575\r\n",0},
	{0x3002,"16F1578\r\n",0},
	{0x3003,"16F1579\r\n",0},
	{0x3004,"16LF1574\r\n",0},
	{0x3005,"16LF1575\r\n",0},
	{0x3006,"16LF1578\r\n",0},
	{0x3007,"16LF1579\r\n",0},
	{0x3020,"16F1454\r\n",0},
	{0x3021,"16F1455\r\n",0},
	{0x3023,"16F1459\r\n",0},
	{0x3024,"16LF1454\r\n",0},
	{0x3025,"16LF1455\r\n",0},
	{0x3027,"16LF1459\r\n",0},
	{0x302A,"16F1789\r\n",0},
	{0x302B,"16F1788\r\n",0},
	{0x302C,"16LF1789\r\n",0},
	{0x302D,"16LF1788\r\n",0},
	{0x3030,"16F753\r\n",0},
	{0x3031,"16HV753\r\n",0},
	{0x3042,"16F1708\r\n",0},
	{0x3043,"16F1704\r\n",0},
	{0x3044,"16LF1708\r\n",0},
	{0x3045,"16LF1704\r\n",0},
	{0x3048,"16F1716\r\n",0},
	{0x3049,"16F1713\r\n",0},
	{0x304A,"16LF1716\r\n",0},
	{0x304B,"16LF1713\r\n",0},
	{0x304C,"16F1613\r\n",0},
	{0x304D,"16LF1613\r\n",0},
	{0x3050,"12F1572\r\n",0},
	{0x3051,"12F1571\r\n",0},
	{0x3052,"12LF1572\r\n",0},
	{0x3053,"12LF1571\r\n",0},
	{0x3054,"16F1709\r\n",0},
	{0x3055,"16F1705\r\n",0},
	{0x3056,"16LF1709\r\n",0},
	{0x3057,"16LF1705\r\n",0},
	{0x3058,"12F1612\r\n",0},
	{0x3059,"12LF1612\r\n",0},
	{0x305A,"16F1719\r\n",0},
	{0x305B,"16F1718\r\n",0},
	{0x305C,"16F1717\r\n",0},
	{0x305D,"16LF1719\r\n",0},
	{0x305E,"16LF1718\r\n",0},
	{0x305F,"16LF1717\r\n",0},
	{0x3060,"16F1707\r\n",0},
	{0x3061,"16F1703\r\n",0},
	{0x3062,"16LF1707\r\n",0},
	{0x3063,"16LF1703\r\n",0},
	{0x3078,"16F1614\r\n",0},
	{0x3079,"16F1618\r\n",0},
	{0x307A,"16LF1614\r\n",0},
	{0x307B,"16LF1618\r\n",0},
	{0x307C,"16F1615\r\n",0},
	{0x307D,"16F1619\r\n",0},
	{0x307E,"16LF1615\r\n",0},
	{0x307F,"16LF1619\r\n",0},
};

#ifdef _MSC_VER
	void COpenProgDlg::PIC16_ID(int id)
#else
	void PIC16_ID(int id)
#endif
{
	char s[64];
	int i;
	for(i=0;i<sizeof(PIC16LIST)/sizeof(PIC16LIST[0]);i++){
		if(PIC16LIST[i].revmask&&(id>>5)==PIC16LIST[i].id){		//id + rev in same location
			sprintf(s,PIC16LIST[i].device,id&PIC16LIST[i].revmask);
			PrintMessage(s);
			return;
		}
		else if(!PIC16LIST[i].revmask&&id==PIC16LIST[i].id){		//id separate from rev
			sprintf(s,PIC16LIST[i].device);
			PrintMessage(s);
			return;
		}
	}
			sprintf(s,"%s",strings[S_nodev]); //"Unknown device\r\n");
	PrintMessage(s);
}

#ifdef _MSC_VER
void COpenProgDlg::DisplayCODE16F(int size){
#else
void DisplayCODE16F(int size){
#endif
// display 14 bit PIC CODE memory
	char s[256]="",t[256]="";
	char* aux=(char*)malloc((size/COL+1)*(16+COL*5));
	aux[0]=0;
	int valid=0,empty=1,i,j,lines=0;
	for(i=0;i<size&&i<sizeW;i+=COL){
		valid=0;
		for(j=i;j<i+COL&&j<sizeW&&i<size;j++){
			sprintf(t,"%04X ",memCODE_W[j]);
			strcat(s,t);
			if(memCODE_W[j]<0x3fff) valid=1;
	}
		if(valid){
			sprintf(t,"%04X: %s\r\n",i,s);
			empty=0;
			strcat(aux,t);
			lines++;
			if(lines>500){	//limit number of lines printed
				strcat(aux,"(...)\r\n");
				i=(sizeW<size?sizeW:size)-COL*2;
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
void COpenProgDlg::DisplayEE16F(int size){
#else
void DisplayEE16F(int size){
#endif
// display 14 bit PIC EEPROM memory
	int valid=0,empty=1,i,j;
	char s[256]="",t[256]="",v[256]="";
	char* aux=(char*)malloc((size/COL+1)*(16+COL*5));
	aux[0]=0;
#ifdef __GTK_H__
	char *g;
#endif
	empty=1;
	v[0]=0;
	for(i=0x2100;i<0x2100+size&&i<sizeW;i+=COL){
		valid=0;
		for(j=i;j<i+COL&&j<0x2100+size&&j<sizeW;j++){
			sprintf(t,"%02X ",memCODE_W[j]&0xff);
			strcat(s,t);
			sprintf(t,"%c",isprint(memCODE_W[j]&0xFF)&&(memCODE_W[j]&0xFF)<0xFF?memCODE_W[j]&0xFF:'.');
#ifdef __GTK_H__
			g=g_locale_to_utf8(t,-1,NULL,NULL,NULL);
			if(g) strcat(v,g);
			g_free(g);
#else
			strcat(v,t);
#endif
			if(memCODE_W[j]<0xff) valid=1;/**/
		}
		if(valid){
			sprintf(t,"%04X: %s %s\r\n",i,s,v);
			empty=0;
			strcat(aux,t);
		}
		s[0]=0;
		v[0]=0;
	}
	if(empty) PrintMessage(strings[S_Empty]);	//empty
	else PrintMessage(aux);
	free(aux);
}

#ifdef _MSC_VER
void COpenProgDlg::Read16Fxxx(int dim,int dim2,int dim3,int vdd){
#else
void Read16Fxxx(int dim,int dim2,int dim3,int vdd){
#endif
// read 14 bit PIC
// dim=program size 	dim2=eeprom size   dim3=config size
// dim2<0 -> eeprom @ 0x2200
// vdd=0 -> vpp before vdd
// vdd=1 -> vdd (+50ms) before vpp
// vdd=2 -> vdd before vpp
// DevREV@0x2005
// DevID@0x2006
// Config@0x2007
// Calib1/Config2@0x2008
// Calib2/Calib1@0x2009
// eeprom@0x2100
	int k=0,k2=0,z=0,i,j,ee2200=0;
	char s[512],t[256],*aux;
	if(dim2<0){
		dim2=-dim2;
		ee2200=1;
	}
	if(dim>0x2000||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Code size exceeds limits\r\n"
		return;
	}
	if(dim2>0x400||dim2<0){		//Max 1K
		PrintMessage(strings[S_EELim]);	//"EEPROM size exceeds limits\r\n"
		return;
	}
	if(dim3>0x100||dim3<0){
		PrintMessage(strings[S_ConfigLim]);	//"Config area size exceeds limits\r\n"
		return;
	}
	if(dim3<8)dim3=8;
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Read16Fxxx(%d,%d,%d,%d)\n",dim,dim2,dim3,vdd);
	}
	sizeW=0x2100+dim2;
	if(memCODE_W) free(memCODE_W);
	memCODE_W=(WORD*)malloc(sizeof(WORD)*sizeW);
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
	PrintMessage(strings[S_CodeReading1]);		//read code ...
	PrintStatusSetup();
	for(i=0,j=1;i<dim;i++){
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==dim-1){		//2B cmd -> 4B data
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					memCODE_W[k++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			PrintStatus(strings[S_CodeReading],i*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr. %03X"
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	PrintStatusEnd();
	if(k!=dim){
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ReadCodeErr],dim,k);	//"Error reading code area, requested %d words, read %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=k;i<0x2000;i++) memCODE_W[i]=0x3fff;
//****************** read config area ********************
	PrintMessage(strings[S_Read_CONFIG_A]);		//read config ...
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	for(i=0x2000;i<0x2000+dim3;i++){		//Config
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==0x2000+dim3-1){		//2B cmd -> 4B data
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					memCODE_W[0x2000+k2++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			PrintStatus(strings[S_CodeReading],(i-0x2000+dim)*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr. %03X"
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,k2,k2);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	if(k2!=dim3){
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigErr],dim3,k2);	//"Error reading config area, requested %d words, read %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=0x2000+k2;i<0x2000+dim3;i++) memCODE_W[i]=0x3fff;
//****************** read eeprom ********************
	if(dim2){
		PrintMessage(strings[S_ReadEE]);		//Read EEPROM ...
		PrintStatusSetup();
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
			if(j>DIMBUF*2/4-2||i==0x2100+dim2-1){		//2B cmd -> 4B data
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(10);
				read();
				for(z=1;z<DIMBUF-1;z++){
					if(bufferI[z]==READ_DATA_DATA){
						memCODE_W[0x2100+k2++]=bufferI[z+1];
						z++;
					}
				}
				PrintStatus(strings[S_CodeReading],(i-0x2100+dim)*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr. %03X"
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
			PrintMessage2(strings[S_ReadEEErr],dim2,k2);	//"Error reading EE area, ..."
			for(i=0x2100+k2;i<0x2100+dim2;i++) memCODE_W[i]=0x3fff;
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
	PrintStatusClear();			//clear status report
//****************** visualize ********************
	for(i=0;i<4;i+=2){
		PrintMessage4("ID%d: 0x%04X\tID%d: 0x%04X\r\n",i,memCODE_W[0x2000+i],i+1,memCODE_W[0x2000+i+1]);
	}
	PrintMessage1(strings[S_DevID],memCODE_W[0x2006]);	//"DevID: 0x%04X\r\n"
	if(memCODE_W[0x2005]<0x3FFF) PrintMessage1(strings[S_DevREV],memCODE_W[0x2005]);	//"DevREV: 0x%04X\r\n"
	PIC16_ID(memCODE_W[0x2006]);
	PrintMessage1(strings[S_ConfigWord],memCODE_W[0x2007]);	//"Configuration word: 0x%04X\r\n"
	if(dim3>8){
		PrintMessage1(strings[S_Config2Cal1],memCODE_W[0x2008]);	//"Config2 or Cal1: 0x%04X\r\n"
	}
	if(dim3>9){
		PrintMessage1(strings[S_Calib1_2],memCODE_W[0x2009]);	//"Calibration word 1 or 2: 0x%04X\r\n"
	}
	PrintMessage(strings[S_CodeMem2]);	//"\r\nCode memory\r\n"
	DisplayCODE16F(dim);
	s[0]=0;
	int valid=0,empty=1;
	if(dim3>8){
		aux=(char*)malloc((dim3/COL+1)*(16+COL*5));
		aux[0]=0;
		empty=1;
		PrintMessage(strings[S_ConfigResMem]);	//"\r\nConfig and reserved memory:\r\n"
		for(i=0x2000;i<0x2000+dim3;i+=COL){
			valid=0;
			for(j=i;j<i+COL&&j<0x2000+dim3;j++){
				sprintf(t,"%04X ",memCODE_W[j]);
				strcat(s,t);
				if(memCODE_W[j]<0x3fff) valid=1;
			}
			if(valid){
				sprintf(t,"%04X: %s\r\n",i,s);
				empty=0;
				strcat(aux,t);
			}
			s[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
		else PrintMessage(aux);
		free(aux);
	}
	if(dim2){
		PrintMessage(strings[S_EEMem]);	//"\r\nEEPROM memory:\r\n"
		DisplayEE16F(dim2);
	}
	PrintMessage1(strings[S_End],(stop-start)/1000.0);	//"\r\nEnd (%.2f s)\r\n"
	if(saveLog) CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Read16F1xxx(int dim,int dim2,int dim3,int options){
#else
void Read16F1xxx(int dim,int dim2,int dim3,int options){
#endif
// read 14 bit enhanced PIC
// dim=program size
// dim2=eeprom size
// dim3=config size
// options:
//		bit0=0 -> vpp before vdd
//		bit0=1 -> vdd before vpp
//		bit1=1 -> LVP programming
//		bit2=1 -> Config3@0x8009
// DevREV@0x8005
// DevID@0x8006
// Config1@0x8007
// Config2@0x8008
// Config3@0x8009
// Calib1@0x8009/A
// Calib2@0x800A/B
// Calib3@0x800B/C
// eeprom@0x0
	int k=0,k2=0,z=0,i,j;
	if(!CheckV33Regulator()){
		PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
		return;
	}
	char s[256],t[256],*aux;
	if(dim>0x8000||dim<0){
		PrintMessage(strings[S_CodeLim]);	//"Code size exceeds limits\r\n"
		return;
	}
	if(dim2>0x400||dim2<0){		//Max 1K
		PrintMessage(strings[S_EELim]);	//"EEPROM size exceeds limits\r\n"
		return;
	}
	if(dim3>0x200||dim3<0){
		PrintMessage(strings[S_ConfigLim]);	//"Config area size exceeds limits\r\n"
		return;
	}
	if(dim3<11)dim3=11;		//at least config1-2 + calib1-2
	if(saveLog){
		OpenLogFile();	//"Log.txt"
		fprintf(logfile,"Read16F1xxx(%d,%d,%d,%d)\n",dim,dim2,dim3,options);
	}
	if((options&2)==0){				//HV entry
		if(!StartHVReg(8.5)){
			PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
			return;
		}
		else hvreg=8.5;
	}
	else StartHVReg(-1);			//LVP mode, turn off HV
	sizeW=0x8000+dim3;
	sizeEE=dim2;
	if(memCODE_W) free(memCODE_W);
	memCODE_W=(WORD*)malloc(sizeof(WORD)*sizeW);
	if(memEE) free(memEE);
	memEE=(unsigned char*)malloc(sizeEE);			//EEPROM
	for(i=0;i<sizeEE;i++) memEE[i]=0xFF;
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
	if((options&2)==0){				//HV entry
		if((options&1)==0){				//VPP before VDD
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=4;				//VPP
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x5;			//VDD+VPP
	}
		else{							//VDD before VPP without delay
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=1;				//VDD
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x5;			//VDD+VPP
	}
	}
	else{			//Low voltage programming
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=4;				//VPP
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=TX16;			//0000 1010 0001 0010 1100 0010 1011 0010 = 0A12C2B2
		bufferU[j++]=2;
		bufferU[j++]=0x0A;
		bufferU[j++]=0x12;
		bufferU[j++]=0xC2;
		bufferU[j++]=0xB2;
		bufferU[j++]=SET_CK_D;		//Clock pulse
		bufferU[j++]=0x4;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)WriteLogIO();
//****************** read code ********************
	PrintMessage(strings[S_CodeReading1]);		//read code ...
	PrintStatusSetup();
	for(i=0,j=1;i<dim;i++){
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==dim-1){		//2B cmd -> 4B data
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					memCODE_W[k++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			PrintStatus(strings[S_CodeReading],i*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr. %03X"
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	PrintStatusEnd();
	if(k!=dim){
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ReadCodeErr],dim,k);	//"Error reading code area, requested %d words, read %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=k;i<0x8000;i++) memCODE_W[i]=0x3fff;
//****************** read config area ********************
	PrintMessage(strings[S_Read_CONFIG_A]);		//read config ...
	bufferU[j++]=LOAD_CONF;			//counter at 0x8000
	bufferU[j++]=0xFF;
	bufferU[j++]=0xFF;
	for(i=0x8000;i<0x8000+dim3;i++){		//Config
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==0x8000+dim3-1){		//2B cmd -> 4B data
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==READ_DATA_PROG){
					memCODE_W[0x8000+k2++]=(bufferI[z+1]<<8)+bufferI[z+2];
					z+=2;
				}
			}
			PrintStatus(strings[S_CodeReading],(i-0x8000+dim)*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr %03X"
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,k2,k2);	//"i=%d(0x%X), k=%d(0x%X)\n"
				WriteLogIO();
			}
		}
	}
	if(k2!=dim3){
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigErr],dim3,k2);	//"Error reading config area, requested %d words, read %d\r\n"
	}
	else PrintMessage(strings[S_Compl]);
	for(i=0x8000+k2;i<0x8000+dim3;i++) memCODE_W[i]=0x3fff;
//****************** read eeprom ********************
	if(dim2){
		PrintMessage(strings[S_ReadEE]);		//Read EEPROM ...
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x16;		//Reset address
		for(i=k=0;i<dim2;i++){
			bufferU[j++]=READ_DATA_DATA;
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF*2/4-2||i==dim2-1){		//2B cmd -> 4B data
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(5);
				read();
				for(z=1;z<DIMBUF-1;z++){
					if(bufferI[z]==READ_DATA_DATA){
						memEE[k++]=bufferI[z+1];
						z++;
					}
				}
				PrintStatus(strings[S_CodeReading],i*100/(dim+dim2+dim3),i);	//"Read: %d%%, addr %03X"
				j=1;
				if(saveLog){
					fprintf(logfile,strings[S_Log7],i,i,k,k);	//"i=%d(0x%X), k=%d(0x%X)\n"
					WriteLogIO();
				}
			}
		}
		PrintStatusEnd();
		if(i!=dim2){
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ReadEEErr],dim2,i);	//"Error reading EE area, ..."
			for(;i<dim2;i++) memEE[i]=0xff;
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
	PrintStatusClear();			//clear status report
//****************** visualize ********************
	for(i=0;i<4;i+=2){
		PrintMessage4("ID%d: 0x%04X\tID%d: 0x%04X\r\n",i,memCODE_W[0x8000+i],i+1,memCODE_W[0x8000+i+1]);
	}
	PrintMessage1(strings[S_DevID],memCODE_W[0x8006]);	//"DevID: 0x%04X\r\n"
	if(memCODE_W[0x8005]<0x3FFF) PrintMessage1(strings[S_DevREV],memCODE_W[0x8005]);	//"DevREV: 0x%04X\r\n"
	PIC16_ID(memCODE_W[0x8006]);
	PrintMessage2(strings[S_ConfigWordX],1,memCODE_W[0x8007]);	//"Configuration word %d: 0x%04X\r\n"
	PrintMessage2(strings[S_ConfigWordX],2,memCODE_W[0x8008]);	//"Configuration word %d: 0x%04X\r\n"
	if((options&4)==0){	//2 config + 2/3 calib words
	PrintMessage2(strings[S_CalibWordX],1,memCODE_W[0x8009]);	//"Calibration word %d: 0x%04X\r\n"
	PrintMessage2(strings[S_CalibWordX],2,memCODE_W[0x800A]);	//"Calibration word %d: 0x%04X\r\n"
	if(memCODE_W[0x800B]<0x3FFF) PrintMessage2(strings[S_CalibWordX],3,memCODE_W[0x800B]);	//"Calibration word %d: 0x%04X\r\n"
	}
	else{	//3 config + 3 calib words
		PrintMessage2(strings[S_ConfigWordX],3,memCODE_W[0x8009]);	//"Configuration word %d: 0x%04X\r\n"
		PrintMessage2(strings[S_CalibWordX],1,memCODE_W[0x800A]);	//"Calibration word %d: 0x%04X\r\n"
		PrintMessage2(strings[S_CalibWordX],2,memCODE_W[0x800B]);	//"Calibration word %d: 0x%04X\r\n"
		PrintMessage2(strings[S_CalibWordX],3,memCODE_W[0x800C]);	//"Calibration word %d: 0x%04X\r\n"
	}
	PrintMessage(strings[S_CodeMem2]);	//"\r\nCode memory:\r\n"
	DisplayCODE16F(dim);
	if(dim3>15){
	int valid=0,empty=1;
	s[0]=0;
		aux=(char*)malloc(dim3/COL*(16+COL*5));
		aux[0]=0;
		empty=1;
		PrintMessage(strings[S_ConfigResMem]);	//"\r\nConfig and reserved memory:\r\n"
		for(i=0x8000;i<0x8000+dim3;i+=COL){
			valid=0;
			for(j=i;j<i+COL&&j<0x8000+dim3;j++){
				sprintf(t,"%04X ",memCODE_W[j]);
				strcat(s,t);
				if(memCODE_W[j]<0x3fff) valid=1;
			}
			if(valid){
				sprintf(t,"%04X: %s\r\n",i,s);
				empty=0;
				strcat(aux,t);
			}
			s[0]=0;
		}
		if(empty) PrintMessage(strings[S_Empty]);	//empty
		else PrintMessage(aux);
		free(aux);
	}
	if(dim2) DisplayEE();	//visualize
	PrintMessage1(strings[S_End],(stop-start)/1000.0);	//"\r\nEnd (%.2f s)\r\n"
	if(saveLog) CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write12F6xx(int dim,int dim2)
#else
void Write12F6xx(int dim,int dim2)
#endif
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
	int err=0;
	WORD devID=0x3fff,calib1=0x3fff,calib2=0x3fff;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(load_calibword){
		if(sizeW>0x2009) load_calibword=2;
		else if(sizeW>0x2008) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Can't find calibration data\r\n"
			load_calibword=0;
		}
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write12F6xx(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff){
		PrintMessage1(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	}
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib2=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib2<0x3fff){
		PrintMessage1(strings[S_CalibWord2],calib2);	//"Calib2: 0x%04X\r\n"
	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=NOP;
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=WAIT_T3;		//necessary when erasing fully written 16F62xA
								//not mentioned in the prog spec!
	if(programID||load_calibword||ICDenable){
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			if(load_calibword==2) bufferU[j++]=0x09;
			else bufferU[j++]=0x08;
		}
	}
	bufferU[j++]=LOAD_DATA_PROG;
	bufferU[j++]=0xFF;
	bufferU[j++]=0xFF;
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	if(dim2){
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// delay T3=10ms
	}
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
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
	msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=8000>>8;
		bufferU[j++]=8000&0xff;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// delay T3=8ms
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=1;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2100-0x2001;		//EEPROM area: counter at 0x2100
		for(w=2,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms min
				bufferU[j++]=WAIT_T3;				//Tprogram 8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-12||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*9+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err_e++;
							if(max_err&&err+err_e>max_err){
								PrintMessage1(strings[S_MaxErr],err+err_e);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		err+=err_e;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],err_e);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=4000>>8;
	bufferU[j++]=4000&0xff;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;		//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 4ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 1
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 4ms
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 2
		bufferU[j++]=memCODE_W[0x2009]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2009]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 4ms
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(35);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage2(strings[S_Calib1Err],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing Calib1: written %04X, read %04X\r\n"
			err_c++;
		}
		if(load_calibword==2){
			for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
			if (memCODE_W[0x2009]!=(bufferI[z+1]<<8)+bufferI[z+2]){
				PrintMessage("\r\n");
				PrintMessage2(strings[S_Calib2Err],memCODE_W[0x2009],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing Calib2: written %04X, read %04X\r\n"
				err_c++;
			}
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F8x (int dim,int dim2)
#else
void Write16F8x (int dim,int dim2)
#endif
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
// erase if not protected and DevID!=16F84A:
// LOAD_DATA_PROG (0010)(0x3FFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 10ms + CUST_CMD (0001) + CUST_CMD (0111)
// LOAD_DATA_DATA (0011)(0xFF) + CUST_CMD (0001) + CUST_CMD (0111)
// + BEGIN_PROG (1000) + 10ms + CUST_CMD (0001) + CUST_CMD (0111)
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 20ms or 8ms(16F84A)
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 20ms or 8ms(16F84A)
// verify during write
{
	int err=0;
	WORD devID,config;
	int k=0,z=0,i,j,w,r;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F8x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=10000>>8;
	bufferU[j++]=10000&0xff;
	if(config<0x3FF0){
		PrintMessage(strings[S_ProtErase]);	//"Override write protection\r\n"
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0x3F;				//fake config
		bufferU[j++]=0xFF;				//fake config
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
			bufferU[j++]=LOAD_DATA_DATA;	//EEPROM:  spec error?
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
	if(!programID){					//back to program memory
		bufferU[j++]=NOP;				//exit program mode
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x0;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
		bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*r+4);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;
		bufferU[j++]=INC_ADDR_N;	//EEPROM: counter at 0x2100
		bufferU[j++]=1;
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*r+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err+=i-k;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],i-k);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
		WriteLogIO();
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F62x (int dim,int dim2)
#else
void Write16F62x (int dim,int dim2)
#endif
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
	int err=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F62x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=EN_VPP_VCC;		//VPP
	bufferU[j++]=0x4;
	bufferU[j++]=EN_VPP_VCC;		//0
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//delay exit-enter prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=5000>>8;
	bufferU[j++]=5000&0xff;
	if(config<0x3C00){
		PrintMessage(strings[S_ProtErase]);	//"Override write protection\r\n"
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0x3F;				//fake config	spec ERROR!!! is written data=0!!
		bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//program only, internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram=8ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*9+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;		//20FF
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0xFF;		//21FE
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=0x2;		//EEPROM: counter at 0x2200
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram=8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*14+1);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err+=i-k;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],i-k);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d\n"
		WriteLogIO();
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write12F62x(int dim,int dim2)
#else
void Write12F62x(int dim,int dim2)
#endif
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// vpp before vdd
// save OSCCAL @ dim-1
// CONFIG@0x2007 includes 2  calibration bits
// DevID@0x2006
// eeprom@0x2100
// erase: BULK_ERASE_PROG (1001) +10ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 3ms
// eeprom: BULK_ERASE_DATA (1011) + 9ms
// LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 6ms
// verify during write
{
	int err=0;
	WORD devID,config,osccal;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write12F62x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
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
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms after exiting program mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	if(!load_osccal) memCODE_W[dim-1]=osccal;	//backup osccal
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*6.5);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	PrintStatusEnd();
	err+=i-k;
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=6000>>8;
		bufferU[j++]=6000&0xff;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;
		bufferU[j++]=INC_ADDR_N;	//EEPROM: counter at 0x2100
		bufferU[j++]=1;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// delay=12ms
		bufferU[j++]=WAIT_T3;
		for(w=3,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms
				bufferU[j++]=WAIT_T3;				//Tprogram 6ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*7+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err+=i-k;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],i-k);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	if(!load_calibword)	memCODE_W[0x2007]=(memCODE_W[0x2007]&0xfff)+(config&0x3000);
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
		WriteLogIO();
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F87x (int dim,int dim2)
#else
void Write16F87x (int dim,int dim2)
#endif
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
	int err=0;
	WORD devID,config;
	int k=0,z=0,i,j,w,ee2200=0;
	if(dim2<0){
		dim2=-dim2;
		ee2200=1;
	}
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F87x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	if((config&0x3130)!=0x3130){
		PrintMessage(strings[S_ProtErase]);	//"override write protection\r\n"
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0x3F;				//fake config
		bufferU[j++]=0xFF;				//fake config
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
		bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
		bufferU[j++]=EN_VPP_VCC;		//VDD
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
		bufferU[j++]=0x5;
	}
	else{
		if(ICDenable||programID){			//erase 0x2000-2004 also
			bufferU[j++]=LOAD_CONF;			//counter at 0x2000
			bufferU[j++]=0x3F;				//fake config
			bufferU[j++]=0xFF;				//fake config
		}
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
			bufferU[j++]=LOAD_DATA_DATA;	//EEPROM:  spec error?
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
		if(ICDenable||programID){			//back to addr 0
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=0x1;
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=0x0;
			bufferU[j++]=SET_CK_D;
			bufferU[j++]=0x0;
			bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
			bufferU[j++]=EN_VPP_VCC;		//VDD
			bufferU[j++]=0x1;
			bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
			bufferU[j++]=0x5;
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=1;					//EEPROM: counter at 0x2100
		if(ee2200){		//eeprom at 0x2200
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0xFF;
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=1;
		}
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//internally timed ?????
				bufferU[j++]=WAIT_T3;				//Tprogram         ?????
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*8+5);
				w=0;
				read();
				for(z=1;z<DIMBUF;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err_e++;
							if(max_err&&err+err_e>max_err){
								PrintMessage1(strings[S_MaxErr],err+err_e);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		err+=err_e;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],err_e);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=8000>>8;
	bufferU[j++]=8000&0xff;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;		//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG2;			//internally timed
		bufferU[j++]=WAIT_T3;				//Tprogram
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F87xA (int dim,int dim2,int seq)
#else
void Write16F87xA (int dim,int dim2,int seq)
#endif
// write 14 bit PIC
// dim=program size 	dim2=eeprom size
// seq=0: vdd + (50ms) + vdd&vpp
// seq=1: vdd + (50us) + vdd&vpp
// DevID@0x2006
// Config@0x2007
// write CONFIG2@0x2008 if different from 3FFF
// eeprom@0x2100
// erase:
// CHIP ERASE (11111) + 15ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 1.2ms + END_PROGX (10111)
// write eeprom: LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 12ms
// verify during write
{
	int err=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F87xA(%d,%d,%d)\n",dim,dim2,seq);
	}
	for(i=0;i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Config
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=15000>>8;
	bufferU[j++]=15000&0xff;
	if(ICDenable||programID){			//erase 0x2000-2004 also
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0x3F;				//fake config
		bufferU[j++]=0xFF;				//fake config
	}
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x1F;					// CHIP_ERASE (11111)
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*1.5+(6-w)*0.2+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=12000>>8;
		bufferU[j++]=12000&0xff;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;
		bufferU[j++]=INC_ADDR_N;		//EEPROM: counter at 0x2100
		bufferU[j++]=1;
		for(w=0,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed
				bufferU[j++]=WAIT_T3;				//Tprogram 8ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*12.5+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err_e++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		err+=err_e;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],err_e);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=12000>>8;
	bufferU[j++]=12000&0xff;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;		//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed
		bufferU[j++]=WAIT_T3;				//Tprogram
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed
	bufferU[j++]=WAIT_T3;					//Tprogram
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(sizeW>0x2008&&memCODE_W[0x2008]!=0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;				//internally timed
		bufferU[j++]=WAIT_T3;					//Tprogram
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	if(programID) msDelay(50);
	if(ICDenable) msDelay(13);
	msDelay(28);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(sizeW>0x2008&&memCODE_W[0x2008]!=0x3fff){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F81x (int dim,int dim2)
#else
void Write16F81x (int dim,int dim2)
#endif
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
	int err=0;
	WORD devID,config;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F81x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	config=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_ConfigWord],config);	//"Config word: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2000>>8;
	bufferU[j++]=2000&0xff;
	if(programID||ICDenable){
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0x3F;				//fake config
		bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*2.5+3);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		int err_e=0;
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x1;
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=0x0;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
		bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
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
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG2;			//externally timed
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=CUST_CMD;
				bufferU[j++]=0x17;					//END_PROGX (10111)
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-10||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*2.5+5);
				w=0;
				read();
				for(z=1;z<DIMBUF-5;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+4]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+5]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+5]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err_e++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err_e+=i-k;
		err+=err_e;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],err_e);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed
			bufferU[j++]=WAIT_T3;				//Tprogram
			bufferU[j++]=CUST_CMD;
			bufferU[j++]=0x17;					//END_PROGX (10111)
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;		//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG2;				//externally timed
		bufferU[j++]=WAIT_T3;					//Tprogram
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x17;						//END_PROGX (10111)
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;				//externally timed
	bufferU[j++]=WAIT_T3;					//Tprogram
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x17;						//END_PROGX (10111)
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(sizeW>0x2008&&memCODE_W[0x2008]!=0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=7;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=7;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(sizeW>0x2008&&memCODE_W[0x2008]!=0x3fff){
		for(z+=7;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write12F61x(int dim, int d, int d2)
#else
void Write12F61x(int dim, int d, int d2)
#endif
// write 14 bit PIC
// dim=program size
// d not used
// vpp before vdd
// DevREV@0x2005
// DevID@0x2006
// Config@0x2007
// Calib1@0x2008 (save)
// Calib2@0x2009
// erase: BULK_ERASE_PROG (1001) +10ms
// write: LOAD_DATA_PROG (0010) + BEGIN_PROG2 (11000) + 4ms + END_PROG (1010)
// verify during write
{
	int err=0;
	WORD devID=0x3fff,devREV=0x3fff,calib1=0x3fff,calib2=0x3fff;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(load_calibword){
		if(sizeW>0x2008) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Can't find calibration data\r\n"
			load_calibword=0;
		}
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write12F61x(%d)\n",dim);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x05;
	bufferU[j++]=READ_DATA_PROG;	//DevREV
	bufferU[j++]=INC_ADDR;
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devREV=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	if(devREV<0x3FFF) PrintMessage1(strings[S_DevREV],devREV);	//"DevREV: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib2=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff) PrintMessage1(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	if(calib2<0x3fff) PrintMessage1(strings[S_CalibWord2],calib2);	//"Calib2: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
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
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=0x08;
		}
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms after exiting program mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 4ms
			bufferU[j++]=END_PROG;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*5+2.5);
			w=0;
			read();
			for(z=1;z<DIMBUF-7;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+5]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+6]<<8)+bufferI[z+7]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;			//externally timed, T=3ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 4ms
	bufferU[j++]=END_PROG;
	bufferU[j++]=WAIT_T2;				//Tdischarge 100us
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;			//Calib word 1
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=8;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=8;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2008]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage2(strings[S_Calib1Err],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing Calib1: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F88x(int dim,int dim2)
#else
void Write16F88x(int dim,int dim2)
#endif
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
	int err=0;
	WORD devID=0x3fff,calib1=0x3fff;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2009){
		PrintMessage(strings[S_NoConfigW4]);	//"Can't find CONFIG (0x2008)\r\nEnd\r\n"
		return;
	}
	if(load_calibword){
		if(sizeW>0x200A) load_calibword=1;
		else{
			PrintMessage(strings[S_NoCalibW]);	//"Can't find calibration data\r\n"
			load_calibword=0;
		}
	}
	if(dim2){
		if(sizeW<0x2100){
			dim2=0;
			PrintMessage(strings[S_NoEEMem]);	//"Can't find EEPROM data\r\n"
		}
		else if(dim2>sizeW-0x2100) dim2=sizeW-0x2100;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F88x(%d,%d)\n",dim,dim2);
	}
	for(i=0;i<0x200A&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff){
		PrintMessage1(strings[S_CalibWord1],calib1);	//"Calib1: 0x%04X\r\n"
	}
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
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
	if(programID||load_calibword||ICDenable){
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		if(load_calibword){
			bufferU[j++]=INC_ADDR_N;
			bufferU[j++]=9;
		}
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=6ms
	bufferU[j++]=EN_VPP_VCC;		//exit program mode
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			// delay T3=6ms after exiting program mode
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*3+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write eeprom ********************
	if(dim2){
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=6000>>8;
		bufferU[j++]=6000&0xff;
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// delay T3=6ms
		bufferU[j++]=INC_ADDR_N;		//use only INC_ADDR_N so verification does not look at it
		bufferU[j++]=0xFF;
		bufferU[j++]=INC_ADDR_N;		//EEPROM: counter at 0x2100
		bufferU[j++]=1;
		for(w=2,i=k=0x2100;i<0x2100+dim2;i++){
			if(memCODE_W[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memCODE_W[i]&0xff;
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=6ms min
				bufferU[j++]=WAIT_T3;				//Tprogram 6ms
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-12||i==0x2100+dim2-1){
				PrintStatus(strings[S_CodeWriting],(i-0x2100+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*6.5+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memCODE_W[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memCODE_W[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							err++;
							if(max_err&&err>max_err){
								PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
					fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d \n"
					WriteLogIO();
				}
			}
		}
		err+=i-k;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],i-k);	//"completed, %d errors\r\n"
	}
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=3000>>8;
	bufferU[j++]=3000&0xff;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;			//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 3ms
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word2 0x2008
	bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=3ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(load_calibword){
		bufferU[j++]=LOAD_DATA_PROG;		//Calib word 1
		bufferU[j++]=memCODE_W[0x2009]>>8;	//MSB
		bufferU[j++]=memCODE_W[0x2009]&0xff;	//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"\r\n"
		err_c++;
	}
	if(load_calibword){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2009]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage2(strings[S_Calib1Err],memCODE_W[0x2009],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing Calib1: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
		WriteLogIO();
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F7x(int dim,int vdd)
#else
void Write16F7x(int dim,int vdd)
#endif
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
	int err=0;
	WORD devID=0x3fff;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F7x(%d,%d)\n",dim,vdd);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(50);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
	j=1;
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=1000>>8;
	bufferU[j++]=1000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms min
			bufferU[j++]=END_PROG2;
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-10||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*1.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-6;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+4]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+5]<<8)+bufferI[z+6]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+5]<<8)+bufferI[z+6]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 1ms
	bufferU[j++]=END_PROG2;
	bufferU[j++]=WAIT_T3;				//Tprogram 1ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(sizeW>0x2008&&memCODE_W[0x2008]<0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 2 0x2008
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//externally timed, T=1ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 1ms
		bufferU[j++]=END_PROG2;
		bufferU[j++]=WAIT_T3;				//Tprogram 1ms
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(15);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(sizeW>0x2008&&memCODE_W[0x2008]<0x3fff){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F71x(int dim,int vdd)
#else
void Write16F71x(int dim,int vdd)
#endif
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
	int err=0;
	WORD devID=0x3fff;
	int k=0,z=0,i,j,w;
	if(sizeW<0x2007){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F71x(%d,%d)\n",dim,vdd);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
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
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(20);
	if(vdd==0) msDelay(50);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
	}
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=NOP;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=10ms
	bufferU[j++]=NOP;				//exit program mode
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x1;
	bufferU[j++]=EN_VPP_VCC;
	bufferU[j++]=0x0;
	bufferU[j++]=WAIT_T3;			//delay after exiting prog mode
	//enter program mode
	bufferU[j++]=EN_VPP_VCC;		//VDD
	bufferU[j++]=0x1;
	if(vdd==0){
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
		bufferU[j++]=WAIT_T3;			//delay between vdd and vpp
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
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 1ms min
			bufferU[j++]=END_PROG2;
			bufferU[j++]=WAIT_T2;				//Tdischarge 100us
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-11||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*2.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-7;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+5]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+6]<<8)+bufferI[z+7]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+6]<<8)+bufferI[z+7]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
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
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG2;			//externally timed, T=1ms min
	bufferU[j++]=WAIT_T3;				//Tprogram 1ms
	bufferU[j++]=END_PROG2;
	bufferU[j++]=WAIT_T2;				//Tdischarge 100us
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(sizeW>0x2008&&memCODE_W[0x2008]<0x3fff){
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 2 0x2008
		bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
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
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(sizeW>0x2008&&memCODE_W[0x2008]<0x3fff){
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F72x(int dim, int d, int d2)
#else
void Write16F72x(int dim, int d, int d2)
#endif
// write 14 bit PIC
// dim=program size
// d not used
// vpp before vdd
// DevID@0x2006
// Config@0x2007
// Config2@0x2008 (not used on LF devices)
// erase: BULK_ERASE_PROG (1001) +6ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 2.5ms
// verify during write
{
	int err=0;
	WORD devID=0x3fff;
	int k=0,z=0,i,j,w;
	if(!CheckV33Regulator()){
		PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
		return;
	}
	if(!StartHVReg(8.5)){
		PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
		return;
	}
	if(sizeW<0x2008){
		PrintMessage(strings[S_NoConfigW3]);	//"Can't find CONFIG (0x2007)\r\nEnd\r\n"
		return;
	}
	if(saveLog){
		OpenLogFile();	//"log.txt"
		fprintf(logfile,"Write16F72x(%d)\n",dim);
	}
	for(i=0;i<0x2009&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	bufferU[j++]=EN_VPP_VCC;		//VDD+VPP
	bufferU[j++]=0x5;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x06;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x16;		//Reset address
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=6000>>8;
	bufferU[j++]=6000&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(3);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	PIC16_ID(devID);
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erase ... "
	j=1;
	if(programID||ICDenable){
		bufferU[j++]=LOAD_CONF;			//counter at 0x2000
		bufferU[j++]=0xFF;				//fake config
		bufferU[j++]=0xFF;				//fake config
	}
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// delay T3=6ms
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x16;		//Reset address
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2500>>8;
	bufferU[j++]=2500&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(18);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(w=i=k=0,j=1;i<dim;i++){
		if(memCODE_W[i]<0x3fff){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			w++;
		}
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF-12||i==dim-1){
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, add. %03X"
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(w*2.5+2);
			w=0;
			read();
			for(z=1;z<DIMBUF-5;z++){
				if(bufferI[z]==INC_ADDR&&memCODE_W[k]>=0x3fff) k++;
				else if(bufferI[z]==LOAD_DATA_PROG&&bufferI[z+3]==READ_DATA_PROG){
					if (memCODE_W[k]!=(bufferI[z+4]<<8)+bufferI[z+5]){
						PrintMessage("\r\n");
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+4]<<8)+bufferI[z+5]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
						if(max_err&&err>max_err){
							PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
							PrintMessage(strings[S_IntW]);	//"Write interrupted"
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
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d,\n"
				WriteLogIO();
			}
		}
	}
	err+=i-k;
	PrintStatusEnd();
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
//****************** write ID, CONFIG, CALIB ********************
	PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
	int err_c=0;
	int ICDgoto=0x2800+(ICDaddr&0x7FF);		//GOTO ICD routine (0x28xx)
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=6000>>8;
	bufferU[j++]=6000&0xff;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;				//fake config
	bufferU[j++]=0xFF;				//fake config
	if(programID){
		for(i=0x2000;i<0x2004;i++){
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i]>>8;		//MSB
			bufferU[j++]=memCODE_W[i]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
			bufferU[j++]=WAIT_T3;				//Tprogram 3ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
	}
	else{
		bufferU[j++]=INC_ADDR_N;
		bufferU[j++]=4;
	}
	if(ICDenable){		//write a GOTO ICD routine (0x28xx)
		bufferU[j++]=LOAD_DATA_PROG;
		bufferU[j++]=ICDgoto>>8;		//MSB
		bufferU[j++]=ICDgoto&0xFF;			//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed, T=3ms min
		bufferU[j++]=WAIT_T3;				//Tprogram 3ms
		bufferU[j++]=READ_DATA_PROG;
	}
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=3;
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x2007
	bufferU[j++]=memCODE_W[0x2007]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2007]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=5ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=INC_ADDR;
	if(sizeW>0x2008){						//only if Config2 is present
	bufferU[j++]=LOAD_DATA_PROG;			//Config word 2 0x2008
	bufferU[j++]=memCODE_W[0x2008]>>8;		//MSB
	bufferU[j++]=memCODE_W[0x2008]&0xff;		//LSB
	bufferU[j++]=BEGIN_PROG;				//internally timed, T=5ms min
	bufferU[j++]=WAIT_T3;					//Tprogram 3ms
	bufferU[j++]=READ_DATA_PROG;
	bufferU[j++]=FLUSH;
	}
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(45);
	read();
	for(i=0,z=0;programID&&i<4;i++){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (memCODE_W[0x2000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage("\r\n");
			PrintMessage3(strings[S_IDErr],i,memCODE_W[0x2000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	if(ICDenable){
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if (ICDgoto!=(bufferI[z+1]<<8)+bufferI[z+2]){
			PrintMessage4(strings[S_ICDErr],0x2004,i,ICDgoto,(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ICD (0x%X): written %04X, read %04X\r\n"
			err_c++;
		}
		z+=6;
	}
	for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	if(sizeW>0x2008){						//only if Config2 is present
	for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	if(~memCODE_W[0x2008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
		PrintMessage("\r\n");
		PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x2008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
		err_c++;
	}
	}
	err+=err_c;
	PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
	if(saveLog){
		fprintf(logfile,strings[S_Log9],err);	//"Area config. 	errors=%d \n"
		WriteLogIO();
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}

#ifdef _MSC_VER
void COpenProgDlg::Write16F1xxx(int dim,int dim2,int options)
#else
void Write16F1xxx(int dim,int dim2,int options)
#endif
// write 14 bit enhanced PIC
// dim=program size
// dim2=eeprom size
// options:
//		bit0=0 -> vpp before vdd
//		bit0=1 -> vdd before vpp
//		bit1=1 -> LVP programming
//		bit2=1 -> Config3@0x8009
// DevREV@0x8005
// DevID@0x8006
// Config1@0x8007
// Config2@0x8008
// Config3@0x8009
// Calib1@0x8009/A
// Calib2@0x800A/B
// Calib3@0x800B/C
// eeprom@0x0
// erase: BULK_ERASE_PROG (1001) +5ms
// write:LOAD_DATA_PROG (0010) + BEGIN_PROG (1000) + 2.5ms (8 word algorithm)
// config write time 5ms
// eeprom:	BULK_ERASE_DATA (1011) + 5ms
//			LOAD_DATA_DATA (0011) + BEGIN_PROG (1000) + 2.5ms
// verify after write
{
	int err=0;
	WORD devID=0x3fff,devREV=0x3fff,calib1=0x3fff,calib2=0x3fff,calib3=0x3fff;
	int k=0,k2=0,z=0,i,j,w;
	if(!CheckV33Regulator()){
		PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
		return;
	}
	if(sizeW<0x8009){
		PrintMessage(strings[S_NoConfigW5]);	//"Can't find CONFIG (0x8007-0x8008)\r\n"
		PrintMessage(strings[S_End]);
		return;
	}
	if((options&4)&&sizeW<0x800A){		//Config3 defaults to 0x3FFF
		sizeW=0x800A;
		memCODE_W=(WORD*)realloc(memCODE_W,sizeof(WORD)*sizeW);
		memCODE_W[0x8009]=0x3FFF;
	}
	if(saveLog){
		OpenLogFile();	//"Log.txt"
		fprintf(logfile,"Write16F1xxx(%d,%d,%d)\n",dim,dim2,options);
	}
	if(dim2>sizeEE) dim2=sizeEE;
	if((options&2)==0){				//HV entry
			if(!StartHVReg(8.5)){
			PrintMessage(strings[S_HVregErr]); //"HV regulator error\r\n"
			return;
		}
	}
	else StartHVReg(-1);			//LVP mode, turn off HV
	for(i=0;i<0x800C&&i<sizeW;i++) memCODE_W[i]&=0x3FFF;
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
	if((options&2)==0){				//HV entry
		if((options&1)==0){				//VPP before VDD
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=4;				//VPP
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=0x5;			//VDD+VPP
		}
		else{							//VDD before VPP without delay
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=1;				//VDD
			bufferU[j++]=EN_VPP_VCC;
			bufferU[j++]=0x5;			//VDD+VPP
		}
	}
	else{			//Low voltage programming
		bufferU[j++]=EN_VPP_VCC;
		bufferU[j++]=4;				//VPP
		bufferU[j++]=WAIT_T3;
		bufferU[j++]=TX16;			//0000 1010 0001 0010 1100 0010 1011 0010 = 0A12C2B2
		bufferU[j++]=2;
		bufferU[j++]=0x0A;
		bufferU[j++]=0x12;
		bufferU[j++]=0xC2;
		bufferU[j++]=0xB2;
		bufferU[j++]=SET_CK_D;		//Clock pulse
	bufferU[j++]=0x4;
		bufferU[j++]=SET_CK_D;
		bufferU[j++]=0x0;
	}
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=WAIT_T2;
	bufferU[j++]=LOAD_CONF;			//counter at 0x2000
	bufferU[j++]=0xFF;
	bufferU[j++]=0xFF;
	bufferU[j++]=INC_ADDR_N;
	bufferU[j++]=0x05;
	bufferU[j++]=READ_DATA_PROG;	//DevREV
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//DevID
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=INC_ADDR;
	if(options&4) bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib1
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib2
	bufferU[j++]=INC_ADDR;
	bufferU[j++]=READ_DATA_PROG;	//Calib3
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x16;		//Reset address
	bufferU[j++]=SET_PARAMETER;
	bufferU[j++]=SET_T3;
	bufferU[j++]=2500>>8;
	bufferU[j++]=2500&0xff;
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(3);
	read();
	if(saveLog)WriteLogIO();
	for(z=0;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devREV=(bufferI[z+1]<<8)+bufferI[z+2];
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	devID=(bufferI[z+1]<<8)+bufferI[z+2];
	PrintMessage1(strings[S_DevID],devID);	//"DevID: 0x%04X\r\n"
	if(devREV<0x3FFF) PrintMessage1(strings[S_DevREV],devREV);	//"DevREV: 0x%04X\r\n"
	PIC16_ID(devID);
	if(memCODE_W[0x8006]<0x3FFF&&devID!=memCODE_W[0x8006]) PrintMessage(strings[S_DevMismatch]);	//"Warning: the device is different from what specified in source data"
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib1=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib1<0x3fff) PrintMessage2(strings[S_CalibWordX],1,calib1);	//"Calibration word %d: 0x%04X\r\n"
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib2=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib2<0x3fff) PrintMessage2(strings[S_CalibWordX],2,calib2);	//"Calibration word %d: 0x%04X\r\n"
	for(z+=3;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
	calib3=(bufferI[z+1]<<8)+bufferI[z+2];
	if(calib3<0x3fff) PrintMessage2(strings[S_CalibWordX],3,calib3);	//"Calibration word %d: 0x%04X\r\n"
//****************** erase memory ********************
	PrintMessage(strings[S_StartErase]);	//"Erasing ... "
	j=1;
	if(programID){
		bufferU[j++]=LOAD_CONF;			//PC @ 0x8000
		bufferU[j++]=0xFF;
		bufferU[j++]=0xFF;
	}
	bufferU[j++]=BULK_ERASE_PROG;
	bufferU[j++]=WAIT_T3;			// wait 5ms
	bufferU[j++]=WAIT_T3;
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x16;		//Reset address
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(8);
	read();
	if(saveLog)WriteLogIO();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** write code ********************
	PrintMessage(strings[S_StartCodeProg]);	//"Write code ... "
	PrintStatusSetup();
	for(;dim>0&&memCODE_W[dim]>=0x3fff;dim--); //skip empty space at end
	dim+=dim%8;		//grow to 8 word multiple
	int valid,inc;
	for(i=k=0,j=1;i<dim;i+=8){
		valid=inc=0;
		for(;i<dim&&!valid;){	//skip empty locations (8 words)
			valid=0;
			for(k=0;k<8;k++) if(memCODE_W[i+k]<0x3fff) valid=1;
			if(!valid){
				inc+=8;
				i+=8;
			}
			if(inc&&(valid||inc==248)){	//increase address to skip empty words
				bufferU[j++]=INC_ADDR_N;
				bufferU[j++]=k=inc;
				inc=0;
			}
			if(j>DIMBUF-4||valid){
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(3);
				msDelay(1*k/2);	//wait for long INC_ADDR_N
				read();
				j=1;
				if(saveLog)	WriteLogIO();
			}
		}
		if(valid){
			k=0;
			bufferU[j++]=LOAD_DATA_PROG;
			bufferU[j++]=memCODE_W[i+k]>>8;  		//MSB
			bufferU[j++]=memCODE_W[i+k]&0xff;		//LSB
			for(k=1;k<8;k++){
				bufferU[j++]=INC_ADDR;
				bufferU[j++]=LOAD_DATA_PROG;
				bufferU[j++]=memCODE_W[i+k]>>8;  		//MSB
				bufferU[j++]=memCODE_W[i+k]&0xff;		//LSB
		}
			bufferU[j++]=BEGIN_PROG;			//internally timed, T=2.5ms
			bufferU[j++]=WAIT_T3;
			bufferU[j++]=INC_ADDR;
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(3);
			read();
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log7],i,i,0,0);	//"i=%d, k=%d 0=%d\n"
				WriteLogIO();
			}
			PrintStatus(strings[S_CodeWriting],i*100/dim,i);	//"Writing: %d%%, addr. %03X"
		}
	}
	PrintStatusEnd();
	PrintMessage(strings[S_Compl]);	//"completed\r\n"
//****************** verify code ********************
	PrintMessage(strings[S_CodeV]);	//"Verifying code ... "
	PrintStatusSetup();
	if(saveLog)	fprintf(logfile,"VERIFY CODE\n");
	j=1;
	bufferU[j++]=CUST_CMD;
	bufferU[j++]=0x16;		//Reset address
	bufferU[j++]=FLUSH;
	for(;j<DIMBUF;j++) bufferU[j]=0x0;
	write();
	msDelay(2);
	read();
	if(saveLog)	WriteLogIO();
	j=1;
	for(i=k=0;i<dim;i++){
		if(j==1){				//skip empty locations (only after a write)
			for(valid=0;i<dim&&!valid;){
				for(k2=0;k2<255&&!valid;k2++) if(memCODE_W[i+k2]<0x3fff) valid=1;
				if(k2>16){			//increase address to skip empty words, if enough is found
					bufferU[j++]=INC_ADDR_N;
					bufferU[j++]=k2;
					i+=k2;
					k+=k2;
				}
				if(j>DIMBUF-4||(valid&&j>1)){		//if buffer is full or last skip
					bufferU[j++]=FLUSH;
					for(;j<DIMBUF;j++) bufferU[j]=0x0;
					write();
					msDelay(3);
					msDelay(1*j/2);	//wait for long INC_ADDR_N
					read();
					j=1;
					if(saveLog)	WriteLogIO();
				}
			}
		}
		if(memCODE_W[i]<0x3FFF) bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(j>DIMBUF*2/4-2||i==dim-1){		//2B cmd -> 4B data
			bufferU[j++]=FLUSH;
			for(;j<DIMBUF;j++) bufferU[j]=0x0;
			write();
			msDelay(5);
			read();
			for(z=1;z<DIMBUF-2;z++){
				if(bufferI[z]==INC_ADDR) k++;
				else if(bufferI[z]==READ_DATA_PROG){
					if(memCODE_W[k]<0x3FFF&&(memCODE_W[k]!=(bufferI[z+1]<<8)+bufferI[z+2])){
						PrintMessage3(strings[S_CodeWError2],k,memCODE_W[k],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing address %3X: written %04X, read %04X\r\n"
						err++;
					}
					z+=2;
			}
			}
			PrintStatus(strings[S_CodeV2],i*100/(dim+dim2),i);	//"Verify: %d%%, addr. %04X"
			j=1;
			if(saveLog){
				fprintf(logfile,strings[S_Log8],i,i,k,k,err);	//"i=%d, k=%d, errors=%d\n"
				WriteLogIO();
			}
			if(err>=max_err) i=dim;
		}
	}
	PrintStatusEnd();
	if(k<dim){
		PrintMessage2(strings[S_CodeVError3],dim,k);	//"Error verifying code area, requested %d words, read %d\r\n"
	}
	PrintMessage1(strings[S_ComplErr],err);	//"completed, %d errors\r\n"
	if(err>=max_err){
		PrintMessage1(strings[S_MaxErr],err);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
	}
//****************** write eeprom ********************
	if(dim2&&err<max_err){
		int errEE=0;
		PrintMessage(strings[S_EEAreaW]);	//"Writing EEPROM ... "
		PrintStatusSetup();
		j=1;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=5000>>8;
		bufferU[j++]=5000&0xff;
		bufferU[j++]=BULK_ERASE_DATA;
		bufferU[j++]=WAIT_T3;			// wait 5ms
		bufferU[j++]=CUST_CMD;
		bufferU[j++]=0x16;		//Reset address
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(7);
		read();
		j=1;
		if(saveLog)WriteLogIO();
		for(w=i=k=0;i<dim2;i++){
			if(memEE[i]<0xff){
				bufferU[j++]=LOAD_DATA_DATA;
				bufferU[j++]=memEE[i];
				bufferU[j++]=BEGIN_PROG;			//internally timed, T=5ms max
				bufferU[j++]=WAIT_T3;				//Tprogram
				bufferU[j++]=READ_DATA_DATA;
				w++;
			}
			bufferU[j++]=INC_ADDR;
			if(j>DIMBUF-12||i==dim2-1){
				PrintStatus(strings[S_CodeWriting],(i+dim)*100/(dim+dim2),i);	//"Writing: %d%%, add. %03X"
				bufferU[j++]=FLUSH;
				for(;j<DIMBUF;j++) bufferU[j]=0x0;
				write();
				msDelay(w*5+2);
				w=0;
				read();
				for(z=1;z<DIMBUF-4;z++){
					if(bufferI[z]==INC_ADDR&&memEE[k]>=0xff) k++;
					else if(bufferI[z]==LOAD_DATA_DATA&&bufferI[z+3]==READ_DATA_DATA){
						if (memEE[k]!=bufferI[z+4]){
							PrintMessage("\r\n");
							PrintMessage3(strings[S_CodeWError3],k,memEE[k],bufferI[z+4]);	//"Error writing address %4X: written %02X, read %02X\r\n"
							errEE++;
							if(max_err&&err+errEE>max_err){
								PrintMessage1(strings[S_MaxErr],err+errEE);	//"Exceeded maximum number of errors (%d), write interrupted\r\n"
								PrintMessage(strings[S_IntW]);	//"write interrupted"
								i=dim2;
								z=DIMBUF;
							}
						}
						k++;
						z+=5;
					}
				}
				j=1;
				if(saveLog){
					fprintf(logfile,strings[S_Log8],i,i,k,k,errEE);	//"i=%d, k=%d, errors=%d\n"
					WriteLogIO();
				}
			}
		}
		errEE+=i-k;
		PrintStatusEnd();
		PrintMessage1(strings[S_ComplErr],errEE);	//"completed, %d errors\r\n"
		err+=errEE;
	}
//****************** write ID, CONFIG, CALIB ********************
	if(max_err&&err<max_err){
		PrintMessage(strings[S_ConfigAreaW]);	//"Writing CONFIG area ... "
		int err_c=0;
		bufferU[j++]=SET_PARAMETER;
		bufferU[j++]=SET_T3;
		bufferU[j++]=5000>>8;
		bufferU[j++]=5000&0xff;
		bufferU[j++]=LOAD_CONF;			//PC @ 0x8000
		bufferU[j++]=0xFF;
		bufferU[j++]=0xFF;
		if(programID){
				for(i=0x8000;i<0x8004;i++){
				bufferU[j++]=LOAD_DATA_PROG;
				bufferU[j++]=memCODE_W[i]>>8;		//MSB
				bufferU[j++]=memCODE_W[i]&0xff;		//LSB
					bufferU[j++]=BEGIN_PROG;			//internally timed
					bufferU[j++]=WAIT_T3;				//Tprogram 5ms
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
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 0x8007
		bufferU[j++]=memCODE_W[0x8007]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x8007]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed
		bufferU[j++]=WAIT_T3;				//Tprogram 5ms
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		bufferU[j++]=LOAD_DATA_PROG;			//Config word 2
		bufferU[j++]=memCODE_W[0x8008]>>8;		//MSB
		bufferU[j++]=memCODE_W[0x8008]&0xff;		//LSB
		bufferU[j++]=BEGIN_PROG;			//internally timed
		bufferU[j++]=WAIT_T3;				//Tprogram 5ms
		bufferU[j++]=READ_DATA_PROG;
		bufferU[j++]=INC_ADDR;
		if(options&4){
			bufferU[j++]=LOAD_DATA_PROG;			//Config word 3
			bufferU[j++]=memCODE_W[0x8009]>>8;		//MSB
			bufferU[j++]=memCODE_W[0x8009]&0xff;		//LSB
			bufferU[j++]=BEGIN_PROG;			//internally timed
			bufferU[j++]=WAIT_T3;				//Tprogram 5ms
			bufferU[j++]=READ_DATA_PROG;
			bufferU[j++]=INC_ADDR;
		}
		bufferU[j++]=FLUSH;
		for(;j<DIMBUF;j++) bufferU[j]=0x0;
		write();
		msDelay(18);
		if(programID) msDelay(22);
		read();
		for(i=0,z=0;programID&&i<4;i++){
			for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
				if (memCODE_W[0x8000+i]!=(bufferI[z+1]<<8)+bufferI[z+2]){
				PrintMessage("\r\n");
				PrintMessage3(strings[S_IDErr],i,memCODE_W[0x8000+i],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing ID%d: written %04X, read %04X\r\n"
				err_c++;
			}
			z+=6;
		}
		for(;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x8007]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x8007],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
		for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
		if(~memCODE_W[0x8008]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
			PrintMessage("\r\n");
			PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x8008],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
			err_c++;
		}
		if(options&4){
			for(z+=6;z<DIMBUF-2&&bufferI[z]!=READ_DATA_PROG;z++);
			if(~memCODE_W[0x8009]&((bufferI[z+1]<<8)+bufferI[z+2])){	//error if written 0 and read 1 (~W&R)
				PrintMessage("\r\n");
				PrintMessage2(strings[S_ConfigWErr3],memCODE_W[0x8009],(bufferI[z+1]<<8)+bufferI[z+2]);	//"Error writing config area: written %04X, read %04X\r\n"
				err_c++;
			}
		}
		err+=err_c;
		PrintMessage1(strings[S_ComplErr],err_c);	//"completed, %d errors\r\n"
		if(saveLog){
			fprintf(logfile,strings[S_Log9],err);	//"Config area 	errors=%d \n"
			WriteLogIO();
		}
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
	PrintMessage3(strings[S_EndErr],(stop-start)/1000.0,err,err!=1?strings[S_ErrPlur]:strings[S_ErrSing]);	//"\r\nEnd (%.2f s) %d %s\r\n\r\n"
	if(saveLog)CloseLogFile();
	PrintStatusClear();			//clear status report
}
