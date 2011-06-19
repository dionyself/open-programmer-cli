/*
 * deviceRW.c - Read-write calls for various devices
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

//This cannot be executed conditionally on MSVC
//#include "stdafx.h"


//configure for GUI or command-line
#ifdef _MSC_VER 
	#define _GUI
	#include "msvc_common.h"
#else 
	#define _CMD
	#include "common.h"
#endif
#ifdef __GTK_H__
#define _GTKGUI
#endif

#define EQ(s) !strncmp(s,dev,64)

#ifdef _MSC_VER
#define add(l,d) m_DispoPage.m_dispo.AddString(d);
void COpenProgDlg::AddDevices(){
#elif defined _GTKGUI
extern	GtkWidget * devCombo;
#define add(l,d) gtk_combo_box_append_text(GTK_COMBO_BOX(devCombo),d);
void AddDevices(){
#else
void add(char* list, const char* dev){
	static char last[8]="";
	if(last[0]) strcat(list,", ");
	if(strncmp(dev,last,2)){
		strcat(list,"\n");
		last[0]=dev[0];
		last[1]=dev[1];
		last[2]=0;
	}
	strcat(list,dev);
}

void AddDevices(char *list){		//make sure list is long enough
#endif
	add(list,"10F200");
	add(list,"10F202");
	add(list,"10F204");
	add(list,"10F206");
	add(list,"10F220");
	add(list,"10F222");
	add(list,"12C508");
	add(list,"12C508A");
	add(list,"12C509");
	add(list,"12C509A");
	add(list,"12F508");
	add(list,"12F509");
	add(list,"12F510");
	add(list,"12F519");
	add(list,"12F609");
	add(list,"12F615");
	add(list,"12F629");
	add(list,"12F635");
	add(list,"12C671");
	add(list,"12C672");
	add(list,"12CE673");
	add(list,"12CE674");
	add(list,"12F675");
	add(list,"12F683");
	add(list,"12F1822");
	add(list,"12F1840");
	add(list,"16F505");
	add(list,"16F506");
	add(list,"16F526");
	add(list,"16F54");
	add(list,"16F57");
	add(list,"16F59");
	add(list,"16F610");
	add(list,"16F616");
	add(list,"16F627");
	add(list,"16F627A");
	add(list,"16F628");
	add(list,"16F628A");
	add(list,"16F630");
	add(list,"16F631");
	add(list,"16F636");
	add(list,"16F639");
	add(list,"16F648A");
	add(list,"16F676");
	add(list,"16F677");
	add(list,"16F684");
	add(list,"16F685");
	add(list,"16F687");
	add(list,"16F688");
	add(list,"16F689");
	add(list,"16F690");
	add(list,"16F716");
	add(list,"16F722");
	add(list,"16F722A");
	add(list,"16F723");
	add(list,"16F723A");
	add(list,"16F724");
	add(list,"16F726");
	add(list,"16F727");
	add(list,"16F73");
	add(list,"16F737");
	add(list,"16F74");
	add(list,"16F747");
	add(list,"16F76");
	add(list,"16F767");
	add(list,"16F77");
	add(list,"16F777");
	add(list,"16F785");
	add(list,"16F818");
	add(list,"16F819");
	add(list,"16C83");
	add(list,"16F83");
	add(list,"16F83A");
	add(list,"16C84");
	add(list,"16F84");
	add(list,"16F84A");
	add(list,"16F87");
	add(list,"16F870");
	add(list,"16F871");
	add(list,"16F872");
	add(list,"16F873");
	add(list,"16F873A");
	add(list,"16F874");
	add(list,"16F874A");
	add(list,"16F876");
	add(list,"16F876A");
	add(list,"16F877");
	add(list,"16F877A");
	add(list,"16F88");
	add(list,"16F882");
	add(list,"16F883");
	add(list,"16F884");
	add(list,"16F886");
	add(list,"16F887");
	add(list,"16F913");
	add(list,"16F914");
	add(list,"16F916");
	add(list,"16F917");
	add(list,"16F946");
	add(list,"16F1516");
	add(list,"16F1517");
	add(list,"16F1518");
	add(list,"16F1519");
	add(list,"16F1526");
	add(list,"16F1527");
	add(list,"16F1823");
	add(list,"16F1824");
	add(list,"16F1825");
	add(list,"16F1826");
	add(list,"16F1827");
	add(list,"16F1828");
	add(list,"16F1829");
	add(list,"16F1847");
	add(list,"16LF1902");
	add(list,"16LF1903");
	add(list,"16LF1904");
	add(list,"16LF1906");
	add(list,"16LF1907");
	add(list,"16F1933");
	add(list,"16F1934");
	add(list,"16F1936");
	add(list,"16F1937");
	add(list,"16F1938");
	add(list,"16F1939");
	add(list,"16F1946");
	add(list,"16F1947");
	add(list,"18F242");
	add(list,"18F248");
	add(list,"18F252");
	add(list,"18F258");
	add(list,"18F442");
	add(list,"18F448");
	add(list,"18F452");
	add(list,"18F458");
	add(list,"18F1220");
	add(list,"18F1230");
	add(list,"18F1320");
	add(list,"18F1330");
	add(list,"18F13K50");
	add(list,"18F14K50");
	add(list,"18F2220");
	add(list,"18F2221");
	add(list,"18F2320");
	add(list,"18F23K20");
	add(list,"18F2321");
	add(list,"18F2331");
	add(list,"18F2410");
	add(list,"18F24J10");
	add(list,"18F24J11");
	add(list,"18F2420");
	add(list,"18F24K20");
	add(list,"18F2423");
	add(list,"18F2431");
	add(list,"18F2439");
	add(list,"18F2450");
	add(list,"18F24J50");
	add(list,"18F2455");
	add(list,"18F2458");
	add(list,"18F2480");
	add(list,"18F2510");
	add(list,"18F25J10");
	add(list,"18F25J11");
	add(list,"18F2515");
	add(list,"18F25K20");
	add(list,"18F2520");
	add(list,"18F2523");
	add(list,"18F2525");
	add(list,"18F2539");
	add(list,"18F2550");
	add(list,"18F25J50");
	add(list,"18F2553");
	add(list,"18F2580");
	add(list,"18F2585");
	add(list,"18F2610");
	add(list,"18F26J11");
	add(list,"18F26J13");
	add(list,"18F2620");
	add(list,"18F26K20");
	add(list,"18F26J50");
	add(list,"18F26J53");
	add(list,"18F2680");
	add(list,"18F2682");
	add(list,"18F2685");
	add(list,"18F27J13");
	add(list,"18F27J53");
	add(list,"18F4220");
	add(list,"18F4221");
	add(list,"18F4320");
	add(list,"18F43K20");
	add(list,"18F4321");
	add(list,"18F4331");
	add(list,"18F4410");
	add(list,"18F44J10");
	add(list,"18F44J11");
	add(list,"18F4420");
	add(list,"18F44K20");
	add(list,"18F4423");
	add(list,"18F4431");
	add(list,"18F4439");
	add(list,"18F4450");
	add(list,"18F44J50");
	add(list,"18F4455");
	add(list,"18F4458");
	add(list,"18F4480");
	add(list,"18F4510");
	add(list,"18F45J10");
	add(list,"18F45J11");
	add(list,"18F4515");
	add(list,"18F4520");
	add(list,"18F45K20");
	add(list,"18F4523");
	add(list,"18F4525");
	add(list,"18F4539");
	add(list,"18F4550");
	add(list,"18F45J50");
	add(list,"18F4553");
	add(list,"18F4580");
	add(list,"18F4585");
	add(list,"18F4610");
	add(list,"18F46J11");
	add(list,"18F46J13");
	add(list,"18F4620");
	add(list,"18F46K20");
	add(list,"18F46J50");
	add(list,"18F46J53");
	add(list,"18F4680");
	add(list,"18F4682");
	add(list,"18F4685");
	add(list,"18F47J13");
	add(list,"18F47J53");
	add(list,"18F8722");
	add(list,"24F04KA200");
	add(list,"24F04KA201");
	add(list,"24F08KA101");
	add(list,"24F08KA102");
	add(list,"24F16KA101");
	add(list,"24F16KA102");
	add(list,"24FJ16GA002");
	add(list,"24FJ16GA004");
	add(list,"24FJ32GA002");
	add(list,"24FJ32GA004");
	add(list,"24FJ32GA102");
	add(list,"24FJ32GA104");
	add(list,"24FJ32GB002");
	add(list,"24FJ32GB004");
	add(list,"24FJ48GA002");
	add(list,"24FJ48GA004");
	add(list,"24FJ64GA002");
	add(list,"24FJ64GA004");
	add(list,"24FJ64GA006");
	add(list,"24FJ64GA008");
	add(list,"24FJ64GA010");
	add(list,"24FJ64GA102");
	add(list,"24FJ64GA104");
	add(list,"24FJ64GB002");
	add(list,"24FJ64GB004");
	add(list,"24FJ64GB106");
	add(list,"24FJ64GB108");
	add(list,"24FJ64GB110");
	add(list,"24FJ96GA006");
	add(list,"24FJ96GA008");
	add(list,"24FJ96GA010");
	add(list,"24FJ128GA006");
	add(list,"24FJ128GA008");
	add(list,"24FJ128GA010");
	add(list,"24FJ128GA106");
	add(list,"24FJ128GB106");
	add(list,"24FJ128GA108");
	add(list,"24FJ128GB108");
	add(list,"24FJ128GA110");
	add(list,"24FJ128GB110");
	add(list,"24FJ192GA106");
	add(list,"24FJ192GB106");
	add(list,"24FJ192GA108");
	add(list,"24FJ192GB108");
	add(list,"24FJ192GA110");
	add(list,"24FJ192GB110");
	add(list,"24FJ256GA106");
	add(list,"24FJ256GB106");
	add(list,"24FJ256GA108");
	add(list,"24FJ256GB108");
	add(list,"24FJ256GA110");
	add(list,"24FJ256GB110");
	add(list,"24HJ12GP201");
	add(list,"24HJ12GP202");
	add(list,"24HJ16GP304");
	add(list,"24HJ32GP202");
	add(list,"24HJ32GP204");
	add(list,"24HJ32GP302");
	add(list,"24HJ32GP304");
	add(list,"24HJ64GP202");
	add(list,"24HJ64GP204");
	add(list,"24HJ64GP206");
	add(list,"24HJ64GP210");
	add(list,"24HJ64GP502");
	add(list,"24HJ64GP504");
	add(list,"24HJ64GP506");
	add(list,"24HJ64GP510");
	add(list,"24HJ128GP202");
	add(list,"24HJ128GP204");
	add(list,"24HJ128GP206");
	add(list,"24HJ128GP210");
	add(list,"24HJ128GP306");
	add(list,"24HJ128GP310");
	add(list,"24HJ128GP502");
	add(list,"24HJ128GP504");
	add(list,"24HJ128GP506");
	add(list,"24HJ128GP510");
	add(list,"24HJ256GP206");
	add(list,"24HJ256GP210");
	add(list,"24HJ256GP610");
	add(list,"30F1010");
	add(list,"30F2010");
	add(list,"30F2011");
	add(list,"30F2012");
	add(list,"30F2020");
	add(list,"30F2023");
	add(list,"30F3010");
	add(list,"30F3011");
	add(list,"30F3012");
	add(list,"30F3013");
	add(list,"30F3014");
	add(list,"30F4011");
	add(list,"30F4012");
	add(list,"30F4013");
	add(list,"30F5011");
	add(list,"30F5013");
	add(list,"30F5015");
	add(list,"30F5016");
	add(list,"30F6010");
	add(list,"30F6011");
	add(list,"30F6012");
	add(list,"30F6013");
	add(list,"30F6014");
	add(list,"30F6015");
	add(list,"33FJ06GS101");
	add(list,"33FJ06GS102");
	add(list,"33FJ06GS202");
	add(list,"33FJ12GP201");
	add(list,"33FJ12GP202");
	add(list,"33FJ12MC201");
	add(list,"33FJ12MC202");
	add(list,"33FJ16GP304");
	add(list,"33FJ16GS402");
	add(list,"33FJ16GS404");
	add(list,"33FJ16GS502");
	add(list,"33FJ16GS504");
	add(list,"33FJ16MC304");
	add(list,"33FJ32GP202");
	add(list,"33FJ32GP204");
	add(list,"33FJ32GP302");
	add(list,"33FJ32GP304");
	add(list,"33FJ32GS406");
	add(list,"33FJ32GS606");
	add(list,"33FJ32GS608");
	add(list,"33FJ32GS610");
	add(list,"33FJ32MC202");
	add(list,"33FJ32MC204");
	add(list,"33FJ32MC302");
	add(list,"33FJ32MC304");
	add(list,"33FJ64GP202");
	add(list,"33FJ64GP204");
	add(list,"33FJ64GP206");
	add(list,"33FJ64GP306");
	add(list,"33FJ64GP310");
	add(list,"33FJ64GP706");
	add(list,"33FJ64GP708");
	add(list,"33FJ64GP710");
	add(list,"33FJ64GP802");
	add(list,"33FJ64GP804");
	add(list,"33FJ64GS406");
	add(list,"33FJ64GS606");
	add(list,"33FJ64GS608");
	add(list,"33FJ64GS610");
	add(list,"33FJ64MC202");
	add(list,"33FJ64MC204");
	add(list,"33FJ64MC506");
	add(list,"33FJ64MC508");
	add(list,"33FJ64MC510");
	add(list,"33FJ64MC706");
	add(list,"33FJ64MC710");
	add(list,"33FJ64MC802");
	add(list,"33FJ64MC804");
	add(list,"33FJ128GP202");
	add(list,"33FJ128GP204");
	add(list,"33FJ128GP206");
	add(list,"33FJ128GP306");
	add(list,"33FJ128GP310");
	add(list,"33FJ128GP706");
	add(list,"33FJ128GP708");
	add(list,"33FJ128GP710");
	add(list,"33FJ128GP802");
	add(list,"33FJ128GP804");
	add(list,"33FJ128MC202");
	add(list,"33FJ128MC204");
	add(list,"33FJ128MC506");
	add(list,"33FJ128MC510");
	add(list,"33FJ128MC706");
	add(list,"33FJ128MC708");
	add(list,"33FJ128MC710");
	add(list,"33FJ128MC802");
	add(list,"33FJ128MC804");
	add(list,"33FJ256GP506");
	add(list,"33FJ256GP510");
	add(list,"33FJ256GP710");
	add(list,"33FJ256MC510");
	add(list,"33FJ256MC710");
	add(list,"AT90S1200");
	add(list,"AT90S2313");
	add(list,"AT90S8515");
	add(list,"AT90S8535");
	add(list,"ATmega48");
	add(list,"ATmega8");
	add(list,"ATmega88");
	add(list,"ATmega8515");
	add(list,"ATmega8535");
	add(list,"ATmega16");
	add(list,"ATmega164A");
	add(list,"ATmega168");
	add(list,"ATmega32");
	add(list,"ATmega324A");
	add(list,"ATmega328");
	add(list,"ATmega64");
	add(list,"ATmega644A");
	add(list,"ATmega1284");
	add(list,"ATtiny12");
	add(list,"ATtiny13");
	add(list,"ATtiny24");
	add(list,"ATtiny26");
	add(list,"ATtiny261");
	add(list,"ATtiny2313");
	add(list,"ATtiny44");
	add(list,"ATtiny48");
	add(list,"ATtiny461");
	add(list,"ATtiny4313");
	add(list,"ATtiny84");
	add(list,"ATtiny88");
	add(list,"ATtiny861");
	add(list,"2400");
	add(list,"2401");
	add(list,"2402");
	add(list,"2404");
	add(list,"2408");
	add(list,"2416");
	add(list,"2432");
	add(list,"2464");
	add(list,"24128");
	add(list,"24256");
	add(list,"24512");
	add(list,"241024");
	add(list,"241025");
	add(list,"25010");
	add(list,"25020");
	add(list,"25040");
	add(list,"25080");
	add(list,"25160");
	add(list,"25320");
	add(list,"25640");
	add(list,"25128");
	add(list,"25256");
	add(list,"25512");
	add(list,"251024");
	add(list,"93S46");
	add(list,"93x46");
	add(list,"93x46A");
	add(list,"93S56");
	add(list,"93x56");
	add(list,"93x56A");
	add(list,"93S66");
	add(list,"93x66");
	add(list,"93x66A");
	add(list,"93x76");
	add(list,"93x76A");
	add(list,"93x86");
	add(list,"93x86A");
}

#ifdef _MSC_VER
	void COpenProgDlg::Write(char* dev,int ee)
#else
	void Write(char* dev,int ee)
#endif
{
	if(!strncmp(dev,"16F1",4)||!strncmp(dev,"12F1",4)||!strncmp(dev,"16F72",5));
	else if((!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2))&&hvreg!=13) hvreg=StartHVReg(13)>0?13:0;
//-------------PIC10-16---------------------------------------------------------
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		if(EQ("10F200")||EQ("10F204")||EQ("10F220")){
			Write12F5xx(0x100,0xFF);						//256
		}
		else if(EQ("12F508")||EQ("10F202")||EQ("10F206")||EQ("10F222")){
			Write12F5xx(0x200,0x1FF);						//512
		}
		else if(EQ("16F54")){
			Write12F5xx(0x200,-1);							//512, no osccal
		}
		else if(EQ("16C83")||EQ("16F83")||EQ("16F83A")){
			Write16F8x(0x200,ee?0x40:0);					//512, 64
		}
		else if(EQ("12F509")||EQ("12F510")||EQ("16F505")||EQ("16F506")){
			Write12F5xx(0x400,0x3FF);						//1K
		}
		else if(EQ("12F519")||EQ("16F526")){
			Write12F5xx(0x440,0x3FF);						//1K + 64
		}
		else if(EQ("12F609")||EQ("12F615")||EQ("16F610")){
			Write12F61x(0x400);								//1K
		}
		else if(EQ("16C84")||EQ("16F84")||EQ("16F84A")){
			Write16F8x(0x400,ee?0x40:0);					//1K, 64
		}
		else if(EQ("12F629")||EQ("12F675")||EQ("16F630")||EQ("16F676")){
			Write12F62x(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("16F627")){
			Write16F62x(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("12F635")||EQ("16F631")||EQ("16F627A")||EQ("16F785")){
			Write12F6xx(0x400,ee?0x80:0);					//1K, 128
		}
		else if(EQ("16F818")){
			Write16F81x(0x400,ee?0x80:0);					//1K, 128, vdd no delay
		}
		else if(EQ("16F57")||EQ("16F59")){
			Write12F5xx(0x800,-1);							//2K, no osccal
		}
		else if(EQ("16F616")){
			Write12F61x(0x800);								//2K
		}
		else if(EQ("16F716")){
			Write16F71x(0x800,1);							//2K, vdd
		}
		else if(EQ("16F722")||EQ("16F722A")){
			Write16F72x(0x800);								//2K, vpp, 3.3V
		}
		else if(EQ("16LF1902")){
			Write16F1xxx(0x800,0,0);						//2K
		}
		else if(EQ("16F870")||EQ("16F871")||EQ("16F872")){
			Write16F87x(0x800,ee?0x40:0);					//2K, 64
		}
		else if(EQ("16F628A")){
			Write12F6xx(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("16F628")){
			Write16F62x(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("16F882")){
			Write16F88x(0x800,ee?0x80:0);					//2K, 128
		}
		else if(EQ("12F683")||EQ("16F636")||EQ("16F639")||EQ("16F677")||EQ("16F684")||EQ("16F687")||EQ("16F785")){
			Write12F6xx(0x800,ee?0x100:0);					//2K, 256
		}
		else if(EQ("16F819")){
			Write16F81x(0x800,ee?0x100:0);					//2K, 256, vdd no delay
		}
		else if(EQ("12F1822")||EQ("16F1823")||EQ("16F1826")){
			Write16F1xxx(0x800,ee?0x100:0,0);				//2K, 256
		}
		else if(EQ("16F73")||EQ("16F74")){
			Write16F7x(0x1000,0);							//4K
		}
		else if(EQ("16F723")||EQ("16F723A")||EQ("16F724")){
			Write16F72x(0x1000);							//4K, vpp, 3.3V
		}
		else if(EQ("16F737")||EQ("16F747")){
			Write16F7x(0x1000,1);							//4K, vdd no delay
		}
		else if(EQ("16LF1903")||EQ("16LF1904")){
			Write16F1xxx(0x1000,0,0);						//4K
		}
		else if(EQ("16F873")||EQ("16F874")){
			Write16F87x(0x1000,ee?-0x80:0);					//4K, 128, ee@0x2200
		}
		else if(EQ("16F648A")||EQ("16F685")||EQ("16F688")||EQ("16F689")||EQ("16F690")||EQ("16F913")||EQ("16F914")){
			Write12F6xx(0x1000,ee?0x100:0);					//4K, 256
		}
		else if(EQ("16F873A")||EQ("16F874A")){
			Write16F87xA(0x1000,ee?0x80:0,1);				//4K, 128
		}
		else if(EQ("16F883")||EQ("16F884")){
			Write16F88x(0x1000,ee?0x100:0);					//4K, 256
		}
		else if(EQ("16F87")||EQ("16F88")){
			Write16F81x(0x1000,ee?0x100:0);					//4K, 256, vdd no delay
		}
		else if(EQ("16F1933")||EQ("16F1934")||EQ("16F1824")||EQ("16F1827")||EQ("16F1828")||EQ("12F1840")){
			Write16F1xxx(0x1000,ee?0x100:0,0);				//4K, 256
		}
		else if(EQ("16F76")||EQ("16F77")){
			Write16F7x(0x2000,0);							//8K
		}
		else if(EQ("16F726")||EQ("16F727")){
			Write16F72x(0x2000);							//8K, vpp, 3.3V
		}
		else if(EQ("16F767")||EQ("16F777")){
			Write16F7x(0x2000,1);							//8K, vdd no delay
		}
		else if(EQ("16LF1906")||EQ("16LF1907")){
			Write16F1xxx(0x2000,0,0);						//8K
		}
		else if(EQ("16F916")||EQ("16F917")||EQ("16F946")){
			Write12F6xx(0x2000,ee?0x100:0);					//8K, 256
		}
		else if(EQ("16F876")||EQ("16F877")){
			Write16F87x(0x2000,ee?-0x100:0);				//8K, 256, ee@0x2200
		}
		else if(EQ("16F876A")||EQ("16F877A")){
			Write16F87xA(0x2000,ee?0x100:0,1);				//8K, 256,
		}
		else if(EQ("16F886")||EQ("16F887")){
			Write16F88x(0x2000,ee?0x100:0);					//8K, 256
		}
		else if(EQ("16F1516")||EQ("16F1517")||EQ("16F1526")){
			Write16F1xxx(0x2000,0,0);						//8K
		}
		else if(EQ("16F1936")||EQ("16F1937")||EQ("16F1946")||EQ("16F1825")||EQ("16F1829")||EQ("16F1847")){
			Write16F1xxx(0x2000,ee?0x100:0,0);				//8K, 256
		}
		else if(EQ("16F1518")||EQ("16F1519")||EQ("16F1527")){
			Write16F1xxx(0x4000,0,0);						//16K
		}
		else if(EQ("16F1938")||EQ("16F1939")||EQ("16F1947")){
			Write16F1xxx(0x4000,ee?0x100:0,0);				//16K, 256
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
		}
	}
//-------------PIC18---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = vdd before vpp (12V)
//     1 = vdd before vpp (9V)
//     2 = low voltage entry with 32 bit key
//	bit [7:4]
//     0 = normal eeprom write algoritm
//     1 = with unlock sequence 55 AA
//	bit [11:8]
//     0 = 5ms erase delay, 1ms code write time, 5ms EE write delay, 5ms config write time
//     1 = 550ms erase delay, 1.2ms code write time, no config or EEPROM
//     2 = 550ms erase delay, 3.4ms code write time, no config or EEPROM
	else if(!strncmp(dev,"18F",3)){
		if(EQ("18F1230")){
			Write18Fx(0x1000,ee?0x80:0,8,0x0F0F,0x8787,0);		//4K, 128, 8
		}
		else if(EQ("18F2221")||EQ("18F4221")){
			Write18Fx(0x1000,ee?0x100:0,8,0x3F3F,0x8F8F,0);		//4K, 256, 8
		}
		else if(EQ("18F1220")||EQ("18F2220")||EQ("18F4220")){
			Write18Fx(0x1000,ee?0x100:0,8,0x10000,0x80,0x10);		//4K, 256, 8, EE with unlock
		}
		else if(EQ("18F1330")){
			Write18Fx(0x2000,ee?0x80:0,8,0x0F0F,0x8787,0);		//8K, 128, 8
		}
		else if(EQ("18F2321")||EQ("18F4321")){
			Write18Fx(0x2000,ee?0x100:0,8,0x3F3F,0x8F8F,0);		//8K, 256, 8
		}
		else if(EQ("18F1320")||EQ("18F2320")||EQ("18F4320")||EQ("18F2331")||EQ("18F4331")){
			Write18Fx(0x2000,ee?0x100:0,8,0x10000,0x80,0x10);		//8K, 256, 8, EE with unlock
		}
		else if(EQ("18F13K50")){
			Write18Fx(0x2000,ee?0x100:0,8,0x0F0F,0x8F8F,1);		//8K, 256, 9V
		}
		else if(EQ("18F23K20")||EQ("18F43K20")){
			Write18Fx(0x2000,ee?0x100:0,16,0x0F0F,0x8F8F,1);		//8K, 256, 9V
		}
		else if(EQ("18F2439")||EQ("18F4439")){
			Write18Fx(0x3000,ee?0x100:0,8,0x10000,0x80,0x10);		//12K, 256, 8, EE with unlock
		}
		else if(EQ("18F2410")||EQ("18F4410")){
			Write18Fx(0x4000,0,32,0x3F3F,0x8F8F,0);				//16K, 0, 32
		}
		else if(EQ("18F24J10")||EQ("18F44J10")){
			Write18Fx(0x4000,0,64,0x0101,0x8080,0x202);				//16K, 0, 64, LV
		}
		else if(EQ("18F24J11")||EQ("18F24J50")||EQ("18F44J11")||EQ("18F44J50")){
			Write18Fx(0x4000,0,64,0x0101,0x8080,0x102);				//16K, 0, 64, LV
		}
		else if(EQ("18F2450")||EQ("18F4450")){
			Write18Fx(0x4000,0,16,0x3F3F,0x8F8F,0);				//16K, 0, 16
		}
		else if(EQ("18F14K50")){
			Write18Fx(0x4000,ee?0x100:0,16,0x0F0F,0x8F8F,1);	//16K, 256, 9V
		}
		else if(EQ("18F24K20")||EQ("18F44K20")){
			Write18Fx(0x4000,ee?0x100:0,32,0x0F0F,0x8F8F,1);	//16K, 256, 9V
		}
		else if(EQ("18F2431")||EQ("18F4431")||EQ("18F242")||EQ("18F248")||EQ("18F442")||EQ("18F448")){
			Write18Fx(0x4000,ee?0x100:0,8,0x10000,0x80,0x10);		//16K, 256, 8, EE with unlock
		}
		else if(EQ("18F2420")||EQ("18F2423")||EQ("18F4420")||EQ("18F4423")||EQ("18F2480")||EQ("18F4480")){
			Write18Fx(0x4000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//16K, 256, 32
		}
		else if(EQ("18F2455")||EQ("18F2458")||EQ("18F4455")||EQ("18F4458")){
			Write18Fx(0x6000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//24K, 256, 32
		}
		else if(EQ("18F2539")||EQ("18F4539")){
			Write18Fx(0x6000,ee?0x100:0,8,0x10000,0x80,0x10);	//24K, 256, 8, EE with unlock
		}
		else if(EQ("18F2510")||EQ("18F4510")){
			Write18Fx(0x8000,0,32,0x3F3F,0x8F8F,0);				//32K, 0, 32
		}
		else if(EQ("18F25J10")||EQ("18F45J10")){
			Write18Fx(0x8000,0,64,0x0101,0x8080,0x202);			//32K, 0, 64, LV
		}
		else if(EQ("18F25J11")||EQ("18F25J50")||EQ("18F45J11")||EQ("18F45J50")){
			Write18Fx(0x8000,0,64,0x0101,0x8080,0x102);			//32K, 0, 64, LV
		}
		else if(EQ("18F252")||EQ("18F258")||EQ("18F452")||EQ("18F458")){
			Write18Fx(0x8000,ee?0x100:0,8,0x10000,0x80,0x10);	//32K, 256, 8, EE with unlock
		}
		else if(EQ("18F2550")||EQ("18F2553")||EQ("18F4550")||EQ("18F4553")||EQ("18F2520")||EQ("18F2523")||EQ("18F4520")||EQ("18F4523")||EQ("18F2580")||EQ("18F4580")){
			Write18Fx(0x8000,ee?0x100:0,32,0x3F3F,0x8F8F,0);	//32K, 256, 32
		}
		else if(EQ("18F25K20")||EQ("18F45K20")){
			Write18Fx(0x8000,ee?0x100:0,32,0x0F0F,0x8F8F,1);	//32K, 256, 32, 9V
		}
		else if(EQ("18F2515")||EQ("18F4515")){
			Write18Fx(0xC000,0,64,0x3F3F,0x8F8F,0);				//48K, 0, 64
		}
		else if(EQ("18F2525")||EQ("18F2585")||EQ("18F4525")||EQ("18F4585")){
			Write18Fx(0xC000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//48K, 1K, 64
		}
		else if(EQ("18F2610")||EQ("18F4610")){
			Write18Fx(0x10000,0,64,0x3F3F,0x8F8F,0);			//64K, 0, 64
		}
		else if(EQ("18F26J11")||EQ("18F26J13")||EQ("18F26J50")||EQ("18F26J53")||EQ("18F46J11")||EQ("18F46J13")||EQ("18F46J50")||EQ("18F46J53")){
			Write18Fx(0x10000,0,64,0x0101,0x8080,0x102);		//64K, 0, 64, LV
		}
		else if(EQ("18F2620")||EQ("18F2680")||EQ("18F4620")||EQ("18F4680")){
			Write18Fx(0x10000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//64K, 1K, 64
		}
		else if(EQ("18F26K20")||EQ("18F46K20")){
			Write18Fx(0x10000,ee?0x100:0,64,0x0F0F,0x8F8F,1);	//64K, 256, 64, 9V
		}
		else if(EQ("18F2682")||EQ("18F4682")){
			Write18Fx(0x14000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//80K, 1K, 64
		}
		else if(EQ("18F2685")||EQ("18F4685")){
			Write18Fx(0x18000,ee?0x400:0,64,0x3F3F,0x8F8F,0);	//96K, 1K, 64
		}
		else if(EQ("18F27J13")||EQ("18F27J53")||EQ("18F47J13")||EQ("18F47J53")){
			Write18Fx(0x20000,0,64,0x0101,0x8080,0x102);		//128K, 0, 64, LV
		}
		else if(EQ("18F8722")){
			Write18Fx(0x20000,ee?0x400:0,64,0xFFFF,0x8787,0);	//128K, 1K, 64
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
		}
	}
//-------------PIC24---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = low voltage ICSP entry
//     1 = High voltage ICSP entry (6V)
//     2 = High voltage ICSP entry (12V) + PIC30F sequence (additional NOPs)
//     3 = low voltage ICSP entry (5V power supply)
//	bit [7:4]
//	   0 = config area in the last 2 program words
//	   1 = config area in the last 3 program words
//	   2 = config area in the last 4 program words
//	   3 = 0xF80000 to 0xF80010 except 02 (24F)
//     4 = 0xF80000 to 0xF80016 (24H-33F)
//     5 = 0xF80000 to 0xF8000C (x16 bit, 30F)
//     6 = 0xF80000 to 0xF8000E (30FSMPS)
//	bit [11:8]
//	   0 = code erase word is 0x4064, row write is 0x4004
//	   1 = code erase word is 0x404F, row write is 0x4001
//	   2 = code erase word is 0x407F, row write is 0x4001, 55AA unlock and external timing (2 ms)
//	   3 = code erase word is 0x407F, row write is 0x4001, 55AA unlock and external timing (200 ms)
//	bit [15:12]
//	   0 = eeprom erase word is 0x4050, write word is 0x4004
//	   1 = eeprom erased with bulk erase, write word is 0x4004
//	   2 = eeprom erased with special sequence, write word is 0x4004
//	bit [19:16]
//	   0 = config write is 0x4000
//	   1 = config write is 0x4003
//	   2 = config write is 0x4004
//	   3 = config write is 0x4008
	else if(!strncmp(dev,"24F",3)||!strncmp(dev,"24H",3)||!strncmp(dev,"30F",3)||!strncmp(dev,"33F",3)){
		if(EQ("24F04KA200")||EQ("24F04KA201")){
			Write24Fx(0xB00,0,0x20031,0x05BE,32,2.0);				//1.375KW, HV
		}
		else if(EQ("24F08KA101")||EQ("24F08KA102")){
			Write24Fx(0x1600,ee?0x200:0,0x20031,0x05BE,32,2.0);		//2.75KW, HV, 512
		}
		else if(EQ("24F16KA101")||EQ("24F16KA102")){
			Write24Fx(0x2C00,ee?0x200:0,0x20031,0x05BE,32,2.0);		//5.5KW, HV, 512
		}
		else if(EQ("24FJ16GA002")||EQ("24FJ16GA004")){
			Write24Fx(0x2C00,0,0x10100,0x05BE,64,2.0);				//5.5KW
		}
		else if(EQ("24FJ32GA002")||EQ("24FJ32GA004")){
			Write24Fx(0x5800,0,0x10100,0x05BE,64,2.0);				//11KW
		}
		else if(EQ("24FJ32GA102")||EQ("24FJ32GA104")||EQ("24FJ32GB002")||EQ("24FJ32GB004")){
			Write24Fx(0x5800,0,0x10120,0x07F0,64,2.0);				//11KW
		}
		else if(EQ("24FJ48GA002")||EQ("24FJ48GA004")){
			Write24Fx(0x8400,0,0x10100,0x05BE,64,2.0);				//16.5KW
		}
		else if(EQ("24FJ64GA002")||EQ("24FJ64GA004")||EQ("24FJ64GA006")||EQ("24FJ64GA008")||EQ("24FJ64GA010")){
			Write24Fx(0xAC00,0,0x10100,0x05BE,64,2.0);				//22KW
		}
		else if(EQ("24FJ64GA102")||EQ("24FJ64GA104")||EQ("24FJ64GB002")||EQ("24FJ64GB004")){
			Write24Fx(0xAC00,0,0x10120,0x07F0,64,2.0);				//22KW
		}
		else if(EQ("24FJ64GB106")||EQ("24FJ64GB108")||EQ("24FJ64GB110")){
			Write24Fx(0xAC00,0,0x10110,0x07F0,64,2.0);				//22KW
		}
		else if(EQ("24FJ96GA006")||EQ("24FJ96GA008")||EQ("24FJ96GA010")){
			Write24Fx(0x10000,0,0x10100,0x05BE,64,2.0);				//32KW
		}
		else if(EQ("24FJ128GA006")||EQ("24FJ128GA008")||EQ("24FJ128GA010")){
			Write24Fx(0x15800,0,0x10100,0x05BE,64,2.0);				//44KW
		}
		else if(EQ("24FJ128GA106")||EQ("24FJ128GA108")||EQ("24FJ128GA110")||EQ("24FJ128GB106")||EQ("24FJ128GB108")||EQ("24FJ128GB110")){
			Write24Fx(0x15800,0,0x10110,0x07F0,64,2.0);				//44KW
		}
		else if(EQ("24FJ192GA106")||EQ("24FJ192GA108")||EQ("24FJ192GA110")||EQ("24FJ192GB106")||EQ("24FJ192GB108")||EQ("24FJ192GB110")){
			Write24Fx(0x20C00,0,0x10110,0x07F0,64,2.0);				//68KW
		}
		else if(EQ("24FJ256GA106")||EQ("24FJ256GA108")||EQ("24FJ256GA110")||EQ("24FJ256GB106")||EQ("24FJ256GB108")||EQ("24FJ256GB110")){
			Write24Fx(0x2AC00,0,0x10110,0x07F0,64,2.0);				//88KW
		}
		else if(!strncmp(dev,"33FJ06",6)){
			Write24Fx(0x1000,0,0x00140,0x07F0,64,2.0);				//2KW
		}
		else if(!strncmp(dev,"24HJ12",6)||!strncmp(dev,"33FJ12",6)){
			Write24Fx(0x2000,0,0x00140,0x07F0,64,2.0);				//4KW
		}
		else if(!strncmp(dev,"24HJ16",6)||!strncmp(dev,"33FJ16",6)){
			Write24Fx(0x2C00,0,0x00140,0x07F0,64,2.0);				//5.5KW
		}
		else if(!strncmp(dev,"24HJ32",6)||!strncmp(dev,"33FJ32",6)){
			Write24Fx(0x5800,0,0x00140,0x07F0,64,2.0);				//11KW
		}
		else if(!strncmp(dev,"24HJ64",6)||!strncmp(dev,"33FJ64",6)){
			Write24Fx(0xAC00,0,0x00140,0x07F0,64,2.0);				//22KW
		}
		else if(!strncmp(dev,"24HJ128",7)||!strncmp(dev,"33FJ128",7)){
			Write24Fx(0x15800,0,0x00140,0x07F0,64,2.0);				//44KW
		}
		else if(!strncmp(dev,"24HJ256",7)||!strncmp(dev,"33FJ256",7)){
			Write24Fx(0x2AC00,0,0x00140,0x07F0,64,2.0);				//88KW
		}
		else if(EQ("30F1010")){
			Write24Fx(0x1000,0,0x30263,0x05BE,32,2.0);				//1KW, LV5V
		}
		else if(EQ("30F2020")||EQ("30F2023")){
			Write24Fx(0x2000,0,0x30263,0x05BE,32,2.0);				//2KW, LV5V
		}
		else if(EQ("30F2010")){
			Write24Fx(0x2000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//4KW, 1K, HV12
		}
		else if(EQ("30F2011")||EQ("30F2012")){
			Write24Fx(0x2000,0,0x31252,0x05BE,32,2.0);				//4KW, HV12
		}
		else if(!strncmp(dev,"30F301",6)){
			Write24Fx(0x4000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//8KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F401",6)){
			Write24Fx(0x8000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//16KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F501",6)){
			Write24Fx(0xB000,ee?0x400:0,0x31252,0x05BE,32,2.0);		//22KW, 1K, HV12
		}
		else if(EQ("30F6011")||EQ("30F6013")){
			Write24Fx(0x16000,ee?0x800:0,0x31252,0x05BE,32,2.0);	//44KW, 2K, HV12
		}
		else if(EQ("30F6010")||EQ("30F6012")||EQ("30F6014")||EQ("30F6015")){
			Write24Fx(0x18000,ee?0x1000:0,0x31252,0x05BE,32,2.0);	//49KW, 4K, HV12
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
		}
	}
//-------------ATMEL---------------------------------------------------------
	else if(!strncmp(dev,"AT",2)){
		if(EQ("AT90S1200")||EQ("ATtiny12")){	//*******aggiungere FUSE
			WriteAT(0x400,ee?0x40:0);						//1K, 64
		}
		else if(EQ("ATtiny13")){
			WriteATmega(0x400,ee?0x40:0,16,0);				//1K, 64
		}
		else if(EQ("AT90S2313")){
			WriteAT(0x800,ee?0x80:0);						//2K, 128
		}
		else if(EQ("ATtiny2313")||EQ("ATtiny26")){
			WriteATmega(0x800,ee?0x80:0,16,SLOW);			//2K, 128
		}
		else if(EQ("ATtiny24")||EQ("ATtiny261")){
			WriteATmega(0x800,ee?0x80:0,16,0);				//2K, 128
		}
		else if(EQ("ATtiny48")){
			WriteATmega(0x1000,ee?0x40:0,32,0);				//4K, 64
		}
		else if(EQ("ATtiny44")||EQ("ATtiny461")||EQ("ATtiny4313")||EQ("ATmega48")){
			WriteATmega(0x1000,ee?0x100:0,32,0);			//4K, 256
		}
		else if(EQ("ATtiny88")){
			WriteATmega(0x2000,ee?0x40:0,32,0);				//8K, 64
		}
		else if(EQ("AT90S8515")||EQ("AT90S8535")){
			WriteAT(0x2000,ee?0x100:0);						//8K, 256
		}
		else if(EQ("ATmega8")||EQ("ATmega88")||EQ("ATmega8515")||EQ("ATmega8535")||EQ("ATtiny84")||EQ("ATtiny861")){
			WriteATmega(0x2000,ee?0x200:0,32,0);				//8K, 512
		}
		else if(EQ("ATmega16")||EQ("ATmega164A")||EQ("ATmega168")){
			WriteATmega(0x4000,ee?0x200:0,64,0);				//16K, 512
		}
		else if(EQ("ATmega32")||EQ("ATmega324A")||EQ("ATmega328")){
			WriteATmega(0x8000,ee?0x400:0,64,0);				//32K, 1K
		}
		else if(EQ("ATmega64")||EQ("ATmega644A")){
			WriteATmega(0x10000,ee?0x800:0,128,0);			//64K, 2K
		}
		else if(EQ("ATmega1284")){
			WriteATmega(0x20000,ee?0x1000:0,128,0);			//128K, 4K
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
		}
	}
//-------------I2C---------------------------------------------------------
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"25",2)||!strncmp(dev,"93",2)){
		if(EQ("2400")){
			WriteI2C(0x10,0,1);			//16, 1B addr.
		}
		else if(EQ("2401")){
			WriteI2C(0x80,0,8);			//128, 1B addr.
		}
		else if(EQ("2402")){
			WriteI2C(0x100,0,8);			//256, 1B addr.
		}
		else if(EQ("2404")){
			WriteI2C(0x200,0,16);		//512, 1B addr.
		}
		else if(EQ("2408")){
			WriteI2C(0x400,0,16);		//1K, 1B addr.
		}
		else if(EQ("2416")){
			WriteI2C(0x800,0,16);		//2K, 1B addr.
		}
		else if(EQ("2432")){
			WriteI2C(0x1000,1,32);		//4K, 2B addr.
		}
		else if(EQ("2464")){
			WriteI2C(0x2000,1,32);		//8K, 2B addr.
		}
		else if(EQ("24128")){
			WriteI2C(0x4000,1,64);		//16K, 2B addr.
		}
		else if(EQ("24256")){
			WriteI2C(0x8000,1,64);		//32K, 2B addr.
		}
		else if(EQ("24512")){
			WriteI2C(0x10000,1,128);		//64K, 2B addr.
		}
		else if(EQ("241024")){
			WriteI2C(0x20000,0x201,256);	//128K, 2B addr.
		}
		else if(EQ("241025")){
			WriteI2C(0x20000,0x841,128);	//128K, 2B addr.
		}
//-------------Microwire EEPROM---------------------------------------------------------
		else if(EQ("93S46")){
			Write93Sx(0x80,6,8);						//128, 4W page
		}
		else if(EQ("93x46")){
			Write93Cx(0x80,6,0);							//128,
		}
		else if(EQ("93x46A")){
			Write93Cx(0x80,7,1);							//128, x8
		}
		else if(EQ("93S56")){
			Write93Sx(0x100,8,8);						//256, 4W page
		}
		else if(EQ("93x56")){
			Write93Cx(0x100,8,0);							//256,
		}
		else if(EQ("93x56A")){
			Write93Cx(0x100,9,1);							//256, x8
		}
		else if(EQ("93S66")){
			Write93Sx(0x200,8,8);						//512, 4W page
		}
		else if(EQ("93x66")){
			Write93Cx(0x200,8,0);						//512,
		}
		else if(EQ("93x66A")){
			Write93Cx(0x200,9,1);						//512, x8
		}
		else if(EQ("93x76")){
			Write93Cx(0x400,10,0);						//1k
		}
		else if(EQ("93x76A")){
			Write93Cx(0x400,11,1);						//1k, x8
		}
		else if(EQ("93x86")){
			Write93Cx(0x800,10,0);						//2k,
		}
		else if(EQ("93x86A")){
			Write93Cx(0x800,11,1);						//2k, x8
		}
//-------------SPI---------------------------------------------------------
		else if(EQ("25010")){
			Write25xx(0x80,16);									//128
		}
		else if(EQ("25020")){
			Write25xx(0x100,16);								//256
		}
		else if(EQ("25040")){
			Write25xx(0x200,16);								//512
		}
		else if(EQ("25080")){
			Write25xx(0x400,16);								//1K
		}
		else if(EQ("25160")){
			Write25xx(0x800,16);								//2K
		}
		else if(EQ("25320")){
			Write25xx(0x1000,32);								//4K
		}
		else if(EQ("25640")){
			Write25xx(0x2000,32);								//8K
		}
		else if(EQ("25128")){
			Write25xx(0x4000,64);								//16K
		}
		else if(EQ("25256")){
			Write25xx(0x8000,64);								//32K
		}
		else if(EQ("25512")){
			Write25xx(0x10000,128);								//64K
		}
		else if(EQ("251024")){
			Write25xx(0x20000,256);								//128K
		}
		else{
			PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
		}
	}
//-------------Unsupported device---------------------------------------------------------
	else{
		PrintMessage(strings[S_nodev_w]); //"Device not supported for writing\r\n");
	}
}


#ifdef _MSC_VER
	void COpenProgDlg::Read(char* dev,int ee,int r)
#else
	void Read(char* dev,int ee,int r)
#endif
{
	if(!strncmp(dev,"16F1",4)||!strncmp(dev,"12F1",4));
	else if(!strncmp(dev,"16F72",5)){
		if(!CheckV33Regulator()){
			PrintMessage(strings[S_noV33reg]);	//Can't find 3.3V expansion board
			return;
		}
		if(hvreg!=8.5) hvreg=StartHVReg(8.5)>0?8.5:0;
	}
	else if((!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2))&&hvreg!=13) hvreg=StartHVReg(13)>0?13:0;
//-------------PIC10-16---------------------------------------------------------
	if(!strncmp(dev,"10",2)||!strncmp(dev,"12",2)||!strncmp(dev,"16",2)){
		if(EQ("10F200")||EQ("10F204")||EQ("10F220")){
			Read12F5xx(0x100,r?0x40:5);						//256
		}
		else if(EQ("12C508")||EQ("12C508A")||EQ("16F54")){
			Read12F5xx(0x200,r?0x40:4);						//512
		}
		else if(EQ("12F508")||EQ("10F202")||EQ("10F206")||EQ("10F222")){
			Read12F5xx(0x200,r?0x40:5);						//512
		}
		else if(EQ("16C83")||EQ("16F83")||EQ("16F83A")){
			Read16Fxxx(0x200,ee?0x40:0,r?0x10:8,1);			//512, 64, vdd
		}
		else if(EQ("12C509")||EQ("12C509A")){
			Read12F5xx(0x400,r?0x40:4);						//1K
		}
		else if(EQ("12F509")||EQ("12F510")||EQ("16F505")||EQ("16F506")){
			Read12F5xx(0x400,r?0x40:5);						//1K
		}
		else if(EQ("12F519")||EQ("16F526")){
			Read12F5xx(0x440,r?0x60:8);						//1K + 64
		}
		else if(EQ("12C671")||EQ("12CE673")){
			Read16Fxxx(0x400,0,r?0x100:0,0);				//1K, vpp
		}
		else if(EQ("12F609")||EQ("12F615")||EQ("16F610")){
			Read16Fxxx(0x400,0,r?0x40:9,0);					//1K, vpp, cal1
		}
		else if(EQ("16C84")||EQ("16F84")||EQ("16F84A")){
			Read16Fxxx(0x400,ee?0x40:0,r?0x10:8,1);			//1K, 64, vdd
		}
		else if(EQ("12F635")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x40:10,0);		//1K, 128, vpp, cal1 + cal2
		}
		else if(EQ("16F631")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x80:9,0);			//1K, 128, vpp, cal1
		}
		else if(EQ("12F629")||EQ("12F675")||EQ("16F630")||EQ("16F676")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x20:8,0);			//1K, 128, vpp
		}
		else if(EQ("16F627")){
			Read16Fxxx(0x400,ee?-0x80:0,r?0x10:8,0);		//1K, 128, vpp, ee@0x2200
		}
		else if(EQ("16F627A")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x10:8,0);			//1K, 128, vpp
		}
		else if(EQ("16F818")){
			Read16Fxxx(0x400,ee?0x80:0,r?0x10:8,2);			//1K, 128, vdd short delay
		}
		else if(EQ("16F57")||EQ("16F59")){
			Read12F5xx(0x800,r?0x40:4);						//2K
		}
		else if(EQ("16F722")||EQ("16F722A")){
			Read16Fxxx(0x800,0,r?0x100:11,0);				//2K, vpp, config1-2 + cal1-2, 3.3V
		}
		else if(EQ("12C672")||EQ("12CE674")){
			Read16Fxxx(0x800,0,r?0x100:0,0);				//2K, vpp
		}
		else if(EQ("16F716")){
			Read16Fxxx(0x800,0,8,2);						//2K, vdd
		}
		else if(EQ("16F616")){
			Read16Fxxx(0x800,0,r?0x40:9,0);					//2K, vpp, cal1
		}
		else if(EQ("16LF1902")){
			Read16F1xxx(0x800,0,r?0x200:11,0);				//2K, vpp
		}
		else if(EQ("16F870")||EQ("16F871")||EQ("16F872")){
			Read16Fxxx(0x800,ee?0x40:0,r?0x100:8,1);		//2K, 64, vdd
		}
		else if(EQ("16F628")){
			Read16Fxxx(0x800,ee?-0x80:0,r?0x10:8,0);		//2K, 128, vpp, ee@0x2200
		}
		else if(EQ("16F628A")){
			Read16Fxxx(0x800,ee?0x80:0,r?0x10:8,0);			//2K, 128, vpp
		}
		else if(EQ("16F882")){
			Read16Fxxx(0x800,ee?0x80:0,r?0x80:10,0);		//2K, 128, vpp, config2 + cal1
		}
		else if(EQ("16F819")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x10:8,2);		//2K, 256, vdd short delay
		}
		else if(EQ("12F683")||EQ("16F684")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x40:9,0);		//2K, 256, vpp, cal1
		}
		else if(EQ("16F636")||EQ("16F639")||EQ("16F785")||EQ("16F785")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x40:10,0);		//2K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F677")||EQ("16F687")){
			Read16Fxxx(0x800,ee?0x100:0,r?0x80:9,0);		//2K, 256, vpp, cal1
		}
		else if(EQ("12F1822")||EQ("16F1823")||EQ("16F1826")){
			Read16F1xxx(0x800,ee?0x100:0,r?0x200:11,0);		//2K, 256, vpp
		}
		else if(EQ("16F73")||EQ("16F74")){
			Read16Fxxx(0x1000,0,r?0x20:8,1);				//4K, vdd
		}
		else if(EQ("16F737")||EQ("16F747")){
			Read16Fxxx(0x1000,0,r?0x40:9,2);				//4K, vdd short delay
		}
		else if(EQ("16F723")||EQ("16F723A")||EQ("16F724")){
			Read16Fxxx(0x1000,0,r?0x100:11,0);				//4K, vpp, config1-2 + cal1-2, 3.3V
		}
		else if(EQ("16LF1903")||EQ("16LF1904")){
			Read16F1xxx(0x1000,0,r?0x200:11,0);				//4K, vpp
		}
		else if(EQ("16F873A")||EQ("16F874A")){
			Read16Fxxx(0x1000,ee?0x80:0,r?0x100:8,1);		//4K, 128, vdd
		}
		else if(EQ("16F873")||EQ("16F874")){
			Read16Fxxx(0x1000,ee?-0x80:0,r?0x100:8,1);		//4K, 128, vdd, ee@0x2200
		}
		else if(EQ("16F685")||EQ("16F689")||EQ("16F690")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x80:9,0);		//4K, 256, vpp, cal1
		}
		else if(EQ("16F688")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x40:9,0);		//4K, 256, vpp, cal1
		}
		else if(EQ("16F883")||EQ("16F884")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x80:10,0);		//4K, 256, vpp, config2 + cal1
		}
		else if(EQ("16F648A")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x10:8,0);		//4K, 256, vpp
		}
		else if(EQ("16F87")||EQ("16F88")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x10:9,2);		//4K, 256, vdd short delay
		}
		else if(EQ("16F913")||EQ("16F914")){
			Read16Fxxx(0x1000,ee?0x100:0,r?0x40:10,0);		//4K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F1933")||EQ("16F1934")||EQ("16F1824")||EQ("16F1827")||EQ("16F1828")||EQ("12F1840")){
			Read16F1xxx(0x1000,ee?0x100:0,r?0x200:11,0);	//4K, 256, vpp
		}
		else if(EQ("16F76")||EQ("16F77")){
			Read16Fxxx(0x2000,0,r?0x20:8,1);				//8K, vdd
		}
		else if(EQ("16F767")||EQ("16F777")){
			Read16Fxxx(0x2000,0,r?0x40:9,2);				//8K, vdd short delay
		}
		else if(EQ("16F726")||EQ("16F727")){
			Read16Fxxx(0x2000,0,r?0x100:11,0);				//8K, vpp, config1-2 + cal1-2, 3.3V
		}
		else if(EQ("16LF1906")||EQ("16LF1907")){
			Read16F1xxx(0x2000,0,r?0x200:11,0);				//8K, vpp
		}
		else if(EQ("16F876A")||EQ("16F877A")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x100:8,1);		//8K, 256, vdd
		}
		else if(EQ("16F876")||EQ("16F877")){
			Read16Fxxx(0x2000,ee?-0x100:0,r?0x100:8,1);		//8K, 256, vdd, ee@0x2200
		}
		else if(EQ("16F886")||EQ("16F887")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x80:10,0);		//8K, 256, vpp, config2 + cal1
		}
		else if(EQ("16F916")||EQ("16F917")||EQ("16F946")){
			Read16Fxxx(0x2000,ee?0x100:0,r?0x40:10,0);		//8K, 256, vpp, cal1 + cal2
		}
		else if(EQ("16F1516")||EQ("16F1517")||EQ("16F1526")){
			Read16F1xxx(0x2000,0,r?0x200:11,0);				//8K, vpp
		}
		else if(EQ("16F1936")||EQ("16F1937")||EQ("16F1946")||EQ("16F1825")||EQ("16F1829")||EQ("16F1847")){
			Read16F1xxx(0x2000,ee?0x100:0,r?0x200:11,0);	//8K, 256, vpp
		}
		else if(EQ("16F1518")||EQ("16F1519")||EQ("16F1527")){
			Read16F1xxx(0x4000,0,r?0x200:11,0);				//16K, vpp
		}
		else if(EQ("16F1938")||EQ("16F1939")||EQ("16F1947")){
			Read16F1xxx(0x4000,ee?0x100:0,r?0x200:11,0);	//16K, 256, vpp
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
		}
	}
//-------------PIC18---------------------------------------------------------
	else if(!strncmp(dev,"18F",3)){
		if(EQ("18F1230")){
			Read18Fx(0x1000,ee?0x80:0,0);					//4K, 128
		}
		else if(EQ("18F2221")||EQ("18F4221")||EQ("18F1220")||EQ("18F2220")||EQ("18F4220")){
			Read18Fx(0x1000,ee?0x100:0,0);					//4K, 256
		}
		else if(EQ("18F1330")){
			Read18Fx(0x2000,ee?0x80:0,0);					//8K, 128
		}
		else if(EQ("18F2321")||EQ("18F4321")||EQ("18F1320")||EQ("18F2320")||EQ("18F4320")||EQ("18F2331")||EQ("18F4331")){
			Read18Fx(0x2000,ee?0x100:0,0);					//8K, 256
		}
		else if(EQ("18F13K50")||EQ("18F23K20")||EQ("18F43K20")){
			Read18Fx(0x2000,ee?0x100:0,1);					//8K, 256, 9V
		}
		else if(EQ("18F2439")||EQ("18F4439")){
			Read18Fx(0x3000,ee?0x100:0,0);					//12K, 256
		}
		else if(EQ("18F2410")||EQ("18F4410")||EQ("18F2450")||EQ("18F4450")){
			Read18Fx(0x4000,0,0);							//16K, 0
		}
		else if(EQ("18F24J10")||EQ("18F44J10")||EQ("18F24J11")||EQ("18F24J50")||EQ("18F44J11")||EQ("18F44J50")){
			Read18Fx(0x4000,0,2);							//16K, 0, LV
		}
		else if(EQ("18F2420")||EQ("18F2423")||EQ("18F4420")||EQ("18F4423")||EQ("18F2431")||EQ("18F4431")||EQ("18F2480")||EQ("18F4480")||EQ("18F242")||EQ("18F248")||EQ("18F442")||EQ("18F448")){
			Read18Fx(0x4000,ee?0x100:0,0);					//16K, 256
		}
		else if(EQ("18F14K50")||EQ("18F24K20")||EQ("18F44K20")){
			Read18Fx(0x4000,ee?0x100:0,1);					//16K, 256, 9V
		}
		else if(EQ("18F2455")||EQ("18F2458")||EQ("18F4455")||EQ("18F4458")||EQ("18F2539")||EQ("18F4539")){
			Read18Fx(0x6000,ee?0x100:0,0);					//24K, 256
		}
		else if(EQ("18F2510")||EQ("18F4510")){
			Read18Fx(0x8000,0,0);							//32K, 0
		}
		else if(EQ("18F25J10")||EQ("18F25J11")||EQ("18F25J50")||EQ("18F45J10")||EQ("18F45J11")||EQ("18F45J50")){
			Read18Fx(0x8000,0,2);							//32K, 0, LV
		}
		else if(EQ("18F2550")||EQ("18F2553")||EQ("18F4550")||EQ("18F4553")||EQ("18F2520")||EQ("18F2523")||EQ("18F4520")||EQ("18F4523")||EQ("18F2580")||EQ("18F4580")||EQ("18F252")||EQ("18F258")||EQ("18F452")||EQ("18F458")){
			Read18Fx(0x8000,ee?0x100:0,0);					//32K, 256
		}
		else if(EQ("18F25K20")||EQ("18F45K20")){
			Read18Fx(0x8000,ee?0x100:0,1);					//32K, 256, 9V
		}
		else if(EQ("18F2515")||EQ("18F4515")){
			Read18Fx(0xC000,0,0);							//48K, 0
		}
		else if(EQ("18F2525")||EQ("18F2585")||EQ("18F4525")||EQ("18F4585")){
			Read18Fx(0xC000,ee?0x400:0,0);					//48K, 1K
		}
		else if(EQ("18F2610")||EQ("18F4610")){
			Read18Fx(0x10000,0,0);							//64K, 0
		}
		else if(EQ("18F26J11")||EQ("18F26J13")||EQ("18F26J50")||EQ("18F26J53")||EQ("18F46J11")||EQ("18F46J13")||EQ("18F46J50")||EQ("18F46J53")){
			Read18Fx(0x10000,0,2);							//64K, 0, LV
		}
		else if(EQ("18F2620")||EQ("18F2680")||EQ("18F4620")||EQ("18F4680")){
			Read18Fx(0x10000,ee?0x400:0,0);					//64K, 1K
		}
		else if(EQ("18F26K20")||EQ("18F46K20")){
			Read18Fx(0x10000,ee?0x400:0,1);					//64K, 1K, 9V
		}
		else if(EQ("18F2682")||EQ("18F4682")){
			Read18Fx(0x14000,ee?0x400:0,0);					//80K, 1K
		}
		else if(EQ("18F2685")||EQ("18F4685")){
			Read18Fx(0x18000,ee?0x400:0,0);					//96K, 1K
		}
		else if(EQ("18F27J13")||EQ("18F27J53")||EQ("18F47J13")||EQ("18F47J53")){
			Read18Fx(0x20000,0,2);							//128K, 0, LV
		}
		else if(EQ("18F8722")){
			Read18Fx(0x20000,ee?0x400:0,0);					//128K, 1K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
		}
	}
//-------------PIC24---------------------------------------------------------
// options:
//	bit [3:0]
//     0 = low voltage ICSP entry
//     1 = High voltage ICSP entry (6V)
//     2 = High voltage ICSP entry (12V) + PIC30F sequence (additional NOPs)
//     3 = low voltage ICSP entry (5V power supply)
//	bit [7:4]
//	   0 = config area in the last 2 program words
//	   1 = config area in the last 3 program words
//	   2 = config area in the last 4 program words
//	   3 = 0xF80000 to 0xF80010 except 02 (24F)
//     4 = 0xF80000 to 0xF80016 (24H-33F)
//     5 = 0xF80000 to 0xF8000C (x16 bit, 30F)
//     6 = 0xF80000 to 0xF8000E (30FSMPS)
	else if(!strncmp(dev,"24F",3)||!strncmp(dev,"24H",3)||!strncmp(dev,"30F",3)||!strncmp(dev,"33F",3)){
		if(EQ("24F04KA200")||EQ("24F04KA201")){
			Read24Fx(0xB00,0,0x31,0x05BE,r?0x800:0);				//1.375KW, HV
		}
		else if(EQ("24F08KA101")||EQ("24F08KA102")){
			Read24Fx(0x1600,ee?0x200:0,0x31,0x05BE,r?0x800:0);		//2.75KW, HV, 512
		}
		else if(EQ("24F16KA101")||EQ("24F16KA102")){
			Read24Fx(0x2C00,ee?0x200:0,0x31,0x05BE,r?0x800:0);		//5.5KW, HV, 512
		}
		else if(EQ("24FJ16GA002")||EQ("24FJ16GA004")){
			Read24Fx(0x2C00,0,0,0x05BE,r?0x800:0);					//5.5KW
		}
		else if(EQ("24FJ32GA002")||EQ("24FJ32GA004")){
			Read24Fx(0x5800,0,0,0x05BE,r?0x800:0);					//11KW
		}
		else if(EQ("24FJ48GA002")||EQ("24FJ48GA004")){
			Read24Fx(0x8400,0,0,0x05BE,r?0x800:0);					//16.5KW
		}
		else if(EQ("24FJ64GA002")||EQ("24FJ64GA004")||EQ("24FJ64GA006")||EQ("24FJ64GA008")||EQ("24FJ64GA010")){
			Read24Fx(0xAC00,0,0,0x05BE,r?0x800:0);					//22KW
		}
		else if(EQ("24FJ64GA102")||EQ("24FJ64GA104")||EQ("24FJ64GB002")||EQ("24FJ64GB004")){
			Read24Fx(0xAC00,0,0,0x07F0,r?0x800:0);					//22KW
		}
		else if(EQ("24FJ64GB106")||EQ("24FJ64GB108")||EQ("24FJ64GB110")){
			Read24Fx(0xAC00,0,0x10,0x07F0,r?0x800:0);					//22KW
		}
		else if(EQ("24FJ96GA006")||EQ("24FJ96GA008")||EQ("24FJ96GA010")){
			Read24Fx(0x10000,0,0,0x05BE,r?0x800:0);					//32KW
		}
		else if(EQ("24FJ128GA006")||EQ("24FJ128GA008")||EQ("24FJ128GA010")){
			Read24Fx(0x15800,0,0,0x05BE,r?0x800:0);					//44KW
		}
		else if(EQ("24FJ128GA106")||EQ("24FJ128GA108")||EQ("24FJ128GA110")||EQ("24FJ128GB106")||EQ("24FJ128GB108")||EQ("24FJ128GB110")){
			Read24Fx(0x15800,0,0x10,0x07F0,r?0x800:0);					//44KW
		}
		else if(EQ("24FJ192GA106")||EQ("24FJ192GA108")||EQ("24FJ192GA110")||EQ("24FJ192GB106")||EQ("24FJ192GB108")||EQ("24FJ192GB110")){
			Read24Fx(0x20C00,0,0x10,0x07F0,r?0x800:0);					//68KW
		}
		else if(EQ("24FJ256GA106")||EQ("24FJ256GA108")||EQ("24FJ256GA110")||EQ("24FJ256GB106")||EQ("24FJ256GB108")||EQ("24FJ256GB110")){
			Read24Fx(0x2AC00,0,0x10,0x07F0,r?0x800:0);					//88KW
		}
		else if(!strncmp(dev,"33FJ06",6)){
			Read24Fx(0x1000,0,0x40,0x07F0,r?0x800:0);				//2KW
		}
		else if(!strncmp(dev,"24HJ12",6)||!strncmp(dev,"33FJ12",6)){
			Read24Fx(0x2000,0,0x40,0x07F0,r?0x800:0);				//4KW
		}
		else if(!strncmp(dev,"24HJ16",6)||!strncmp(dev,"33FJ16",6)){
			Read24Fx(0x2C00,0,0x40,0x07F0,r?0x800:0);				//5.5KW
		}
		else if(!strncmp(dev,"24HJ32",6)||!strncmp(dev,"33FJ32",6)){
			Read24Fx(0x5800,0,0x40,0x07F0,r?0x1000:0);				//11KW
		}
		else if(!strncmp(dev,"24HJ64",6)||!strncmp(dev,"33FJ64",6)){
			Read24Fx(0xAC00,0,0x40,0x07F0,r?0x1000:0);				//22KW
		}
		else if(!strncmp(dev,"24HJ128",7)||!strncmp(dev,"33FJ128",7)){
			Read24Fx(0x15800,0,0x40,0x07F0,r?0x1000:0);				//44KW
		}
		else if(!strncmp(dev,"24HJ256",7)||!strncmp(dev,"33FJ256",7)){
			Read24Fx(0x2AC00,0,0x40,0x07F0,r?0x1000:0);				//88KW
		}
		else if(EQ("30F1010")){
			Read24Fx(0x1000,0,0x63,0x05BE,r?0x600:0);				//1KW, LV5V
		}
		else if(EQ("30F2020")||EQ("30F2023")){
			Read24Fx(0x2000,0,0x63,0x05BE,r?0x600:0);				//2KW, LV5V
		}
		else if(EQ("30F2010")){
			Read24Fx(0x2000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//4KW, 1K, HV12
		}
		else if(EQ("30F2011")||EQ("30F2012")){
			Read24Fx(0x2000,0,0x52,0x05BE,r?0x600:0);				//4KW, HV12
		}
		else if(!strncmp(dev,"30F301",6)){
			Read24Fx(0x4000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//8KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F401",6)){
			Read24Fx(0x8000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//16KW, 1K, HV12
		}
		else if(!strncmp(dev,"30F501",6)){
			Read24Fx(0xB000,ee?0x400:0,0x52,0x05BE,r?0x600:0);		//22KW, 1K, HV12
		}
		else if(EQ("30F6011")||EQ("30F6013")){
			Read24Fx(0x16000,ee?0x800:0,0x52,0x05BE,r?0x600:0);		//44KW, 2K, HV12
		}
		else if(EQ("30F6010")||EQ("30F6012")||EQ("30F6014")||EQ("30F6015")){
			Read24Fx(0x18000,ee?0x1000:0,0x52,0x05BE,r?0x600:0);	//49KW, 4K, HV12
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
		}
	}
//-------------ATMEL---------------------------------------------------------
	else if(!strncmp(dev,"AT",2)){
		if(EQ("AT90S1200")){
			ReadAT(0x400,ee?0x40:0,0);							//1K, 64
		}
		if(EQ("ATtiny12")){
			ReadAT(0x400,ee?0x40:0,LOCK+FUSE+CAL);					//1K, 64
		}
		if(EQ("ATtiny13")){
			ReadAT(0x400,ee?0x40:0,LOCK+FUSE+FUSE_H+CAL);			//1K, 64
		}
		else if(EQ("AT90S2313")){
			ReadAT(0x800,ee?0x80:0,0);							//2K, 128
		}
		else if(EQ("ATtiny26")){
			ReadAT(0x800,ee?0x80:0,LOCK+FUSE+FUSE_H+CAL+SLOW);			//2K, 128
		}
		else if(EQ("ATtiny261")||EQ("ATtiny24")){
			ReadAT(0x800,ee?0x80:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//2K, 128
		}
		else if(EQ("ATtiny2313")){
			ReadAT(0x800,ee?0x80:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL+SLOW);//2K, 128
		}
		else if(EQ("ATtiny48")){
			ReadAT(0x1000,ee?0x40:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);		//4K, 64
		}
		else if(EQ("ATtiny44")||EQ("ATtiny461")||EQ("ATtiny4313")||EQ("ATmega48")){
			ReadAT(0x1000,ee?0x100:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//4K, 256
		}
		else if(EQ("ATtiny88")){
			ReadAT(0x2000,ee?0x40:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);		//8K, 64
		}
		else if(EQ("AT90S8515")||EQ("AT90S8535")){
			ReadAT(0x2000,ee?0x100:0,0);						//8K, 256
		}
		else if(EQ("ATmega8")||EQ("ATmega8515")||EQ("ATmega8535")){
			ReadAT(0x2000,ee?0x200:0,LOCK+FUSE+FUSE_H+CAL);		//8K, 512
		}
		else if(EQ("ATmega88")||EQ("ATtiny84")||EQ("ATtiny861")){
			ReadAT(0x2000,ee?0x200:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//8K, 512
		}
		else if(EQ("ATmega16")){
			ReadAT(0x4000,ee?0x200:0,LOCK+FUSE+FUSE_H+CAL);		//16K, 512
		}
		else if(EQ("ATmega164A")||EQ("ATmega168")){
			ReadAT(0x4000,ee?0x200:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//16K, 512
		}
		else if(EQ("ATmega32")){
			ReadAT(0x8000,ee?0x400:0,LOCK+FUSE+FUSE_H+CAL);		//32K, 1K
		}
		else if(EQ("ATmega324A")||EQ("ATmega328")){
			ReadAT(0x8000,ee?0x400:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//32K, 1K
		}
		else if(EQ("ATmega64")||EQ("ATmega644A")){
			ReadAT(0x10000,ee?0x800:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//64K, 2K
		}
		else if(EQ("ATmega1284")){
			ReadAT(0x20000,ee?0x1000:0,LOCK+FUSE+FUSE_H+FUSE_X+CAL);	//128K, 4K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
		}
	}
//-------------I2C---------------------------------------------------------
	else if(!strncmp(dev,"24",2)||!strncmp(dev,"25",2)||!strncmp(dev,"93",2)){
		if(EQ("2400")){
			ReadI2C(0x10,0);						//16, 1B addr.
		}
		else if(EQ("2401")){
			ReadI2C(0x80,0);						//128, 1B addr.
		}
		else if(EQ("2402")){
			ReadI2C(0x100,0);						//256, 1B addr.
		}
		else if(EQ("2404")){
			ReadI2C(0x200,0);						//512, 1B addr.
		}
		else if(EQ("2408")){
			ReadI2C(0x400,0);						//1K, 1B addr.
		}
		else if(EQ("2416")){
			ReadI2C(0x800,0);						//2K, 1B addr.
		}
		else if(EQ("2432")){
			ReadI2C(0x1000,1);						//4K, 2B addr.
		}
		else if(EQ("2464")){
			ReadI2C(0x2000,1);						//8K, 2B addr.
		}
		else if(EQ("24128")){
			ReadI2C(0x4000,1);						//16K, 2B addr.
		}
		else if(EQ("24256")){
			ReadI2C(0x8000,1);						//32K, 2B addr.
		}
		else if(EQ("24512")){
			ReadI2C(0x10000,1);					//64K, 2B addr.
		}
		else if(EQ("241024")){
			ReadI2C(0x20000,0x201);				//128K, 2B addr.
		}
		else if(EQ("241025")){
			ReadI2C(0x20000,0x841);				//128K, 2B addr.
		}
//-------------Microwire EEPROM---------------------------------------------------------
		else if(EQ("93S46")||EQ("93x46")){
			Read93x(0x80,6,0);						//128, 6b addr
		}
		else if(EQ("93x46A")){
			Read93x(0x80,7,1);						//128, 6b addr x8
		}
		else if(EQ("93S56")||EQ("93x56")){
			Read93x(0x100,8,0);						//256, 8b addr
		}
		else if(EQ("93x56A")){
			Read93x(0x100,9,1);						//256, 8b addr x8
		}
		else if(EQ("93S66")||EQ("93x66")){
			Read93x(0x200,8,0);						//512, 8b addr
		}
		else if(EQ("93x66A")){
			Read93x(0x200,9,1);						//512, 8b addr x8
		}
		else if(EQ("93x76")){
			Read93x(0x400,10,0);						//1k, 10b addr
		}
		else if(EQ("93x76A")){
			Read93x(0x400,11,1);						//1k, 10b addr x8
		}
		else if(EQ("93x86")){
			Read93x(0x800,10,0);						//2k, 10b addr
		}
		else if(EQ("93x86A")){
			Read93x(0x800,11,1);						//2k, 10b addr x8
		}
//-------------SPI---------------------------------------------------------
		else if(EQ("25010")){
			Read25xx(0x80);							//128
		}
		else if(EQ("25020")){
			Read25xx(0x100);						//256
		}
		else if(EQ("25040")){
			Read25xx(0x200);						//512
		}
		else if(EQ("25080")){
			Read25xx(0x400);						//1K
		}
		else if(EQ("25160")){
			Read25xx(0x800);						//2K
		}
		else if(EQ("25320")){
			Read25xx(0x1000);						//4K
		}
		else if(EQ("25640")){
			Read25xx(0x2000);						//8K
		}
		else if(EQ("25128")){
			Read25xx(0x4000);						//16K
		}
		else if(EQ("25256")){
			Read25xx(0x8000);						//32K
		}
		else if(EQ("25512")){
			Read25xx(0x10000);						//64K
		}
		else if(EQ("251024")){
			Read25xx(0x20000);						//128K
		}
		else{
			PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
		}
	}
//-------------Unsupported device---------------------------------------------------------
	else{
		PrintMessage(strings[S_nodev_r]); //"Device not supported for reading\r\n");
	}
}
