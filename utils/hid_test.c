/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
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
#include <string.h>



int main (int argc, char **argv) {
#define L_IT 0
#define L_EN 1
	int n=64,d=0,info=0,v=0,q=0,r=1,lang=L_EN,c,i,j,block=0;
	int vid=0x04D8,pid=0x0100;
	char path[512]=""; 
	char buf[256];  
	for(i=0;i<256;i++) buf[i]=0;
	opterr = 0;
	int option_index = 0;
	struct hiddev_devinfo device_info;
	struct option long_options[] =
	{
		{"b",       no_argument,    &block, 1},
		{"block",   no_argument,    &block, 1},
		{"verbose", no_argument,        &v, 1},
		{"v",       no_argument,        &v, 1},
		{"quiet",   no_argument,        &q, 1},
		{"q",       no_argument,        &q, 1},
		{"info",    no_argument,     &info, 1},
		{"i",       no_argument,     &info, 1},
		{"help",    no_argument,       0, 'h'},
		{"path",    required_argument, 0, 'p'},
		{"repeat",  required_argument, 0, 'r'},
		{"delay",   required_argument, 0, 'd'},
		{0, 0, 0, 0}
	};
	if(strstr(getenv("LANG"),"it")!=0) lang=L_IT;
//	if(strstr(getenv("LANG"),"it")!=0) strings=strings_it;
	while ((c = getopt_long_only (argc, argv, "n:d:p:hr:",long_options,&option_index)) != -1)
        switch (c)
           {
           case 'h':	//guida
             if(!lang) printf("hid_test [opzioni] [dati]\n"
				"opzioni: \n"
				"-b, block\tusa lettura bloccante\n"
				"-d, delay\tritardo lettura (ms) [0]\n"
				"-h, help\tguida\n"
				"-i, info\tinformazioni sul dispositivo [no]\n"
				"-n\t\tdimensione report [64]\n"
				"-p, path\tpercorso dispositivo [/dev/usb/hiddev0] \n"
				"-q, quiet\tmostra solo risposta [no]\n"
				"-r, repeat\tripeti lettura N volte [1]\n\n"
				"-v, verbose\tmostra funzioni [no]\n"
				"es.  hid_test -i 1 2 3 4\n");
             else printf("hid_test [otions] [data]\n"
			 	"options: \n"
				"-b, block\tuse blocking read\n"
				"-d, delay\tread delay (ms) [0]\n"
				"-h, help\thelp\n"
				"-i, info\tdevice info [no]\n"
				"-n\t\treport size [64]\n"
				"-p, path\tdevice path [/dev/usb/hiddev0]\n"
				"-q, quiet\tprint response only [no]\n"
				"-r, repeat\trepeat N times [1]\n\n"
				"-v, verbose\tshow functions [no]\n"
				"e.g.  hid_test -i 1 2 3 4\n");
		exit(1);
            break;
            case 'n':	//dim report
             n = atoi(optarg);
             break;
           case 'd':	//ritardo lettura
             d = atoi(optarg);
             break;
          case 'p':	//percorso hiddev
             strncpy(path,optarg,sizeof(path));
             break;
            case 'r':	//ripeti lettura
             r = atoi(optarg);
             break;
           case '?':
             if (optopt == 'c')
               fprintf (stderr, "Option -%c requires an argument.\n", optopt);
             else if (isprint (optopt))
               fprintf (stderr, "Unknown option `-%c'.\n", optopt);
             else
               fprintf (stderr, "Unknown option character 0x%02x\n", optopt);
             return 1;
           default:
             //abort ();
            break;
           }
  
	for (j=0,i = optind; i < argc&&i<128; i++,j++) sscanf(argv[i], "%x", &buf[j]);
 
	int fd = -1;

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
			return -1;
		}
	}
	else{	//user supplied path
		if ((fd = open(path, O_RDONLY )) < 0) {
			printf("cannot open %s, make sure you have read permission on it",path);
			return -1;
		}
	}


	if(info){
		ioctl(fd, HIDIOCGDEVINFO, &device_info);
		printf("vendor 0x%04hx product 0x%04hx version 0x%04hx ",
			device_info.vendor, device_info.product, device_info.version);
		printf("has %i application%s ", device_info.num_applications,
			(device_info.num_applications==1?"":"s"));
		printf("and is on bus: %d devnum: %d ifnum: %d\n",
			device_info.busnum, device_info.devnum, device_info.ifnum);
		char name[256]= "Unknown";
		if(ioctl(fd, HIDIOCGNAME(sizeof(name)), name) < 0) perror("evdev ioctl");
		printf("The device on %s says its name is %s\n", path, name);
	}
	if(!q){ 
		printf("-> ");
	 	for(i=0;i<j;i++) printf("%02X ",(unsigned char)buf[i]);
		printf("\n");
	}
	if(v) printf("%slocking read\n",block?"B":"Non b");
	struct hiddev_report_info rep_info;	
	struct hiddev_usage_ref_multi ref_multi;
	struct hiddev_event ev[80];
	rep_info.report_type=HID_REPORT_TYPE_OUTPUT;
	rep_info.report_id=HID_REPORT_ID_FIRST;
	rep_info.num_fields=1;
	ref_multi.uref.report_type=HID_REPORT_TYPE_OUTPUT;
	ref_multi.uref.report_id=HID_REPORT_ID_FIRST;
	ref_multi.uref.field_index=0;
	ref_multi.uref.usage_index=0;
	ref_multi.num_values=n;
	for(i=0;i<n;i++) ref_multi.values[i]=buf[i];
	int res;
	res=ioctl(fd, HIDIOCSUSAGES, &ref_multi);
	if(v) printf("HIDIOCSUSAGES:%d\n",res);
	res=ioctl(fd,HIDIOCSREPORT, &rep_info);
	if(v) printf("HIDIOCSREPORT:%d\n",res);
	if(!block) for(j=0;j<r;j++){
		usleep(d*1000);
		rep_info.report_type=HID_REPORT_TYPE_INPUT;
		res=ioctl(fd,HIDIOCGREPORT, &rep_info);
		if(v) printf("HIDIOCGREPORT:%d\n",res);
		ref_multi.uref.report_type=HID_REPORT_TYPE_INPUT;
		res=ioctl(fd, HIDIOCGUSAGES, &ref_multi);
		if(v) printf("HIDIOCGUSAGES:%d\n",res);
		if(!q) printf("<- ");
		for(i=0;i<n;i++) printf("%02X ",ref_multi.values[i]);
		printf("\n");
	}
	else for(j=0;j<r;j++){
		usleep(d*1000);
		res=read(fd, ev,sizeof(struct hiddev_event) *n);
		if(!q) printf("<- ");
		for(i=0;i<n;i++) printf("%02X ",ev[i].value);
		printf("\n");
	}
	close(fd);
	exit(0);
}
