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


#if !defined _WIN32 && !defined __CYGWIN__
	#include <sys/ioctl.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <asm/types.h>
	#include <fcntl.h>
	#include <linux/hiddev.h>
	#include <linux/input.h>
	#include <sys/timeb.h>
#else
	#include <windows.h>
	#include <setupapi.h>
	#include <ddk/hidusage.h>
	#include <ddk/hidpi.h>
	#include <math.h>
	#include <sys/timeb.h>
	#include <wchar.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>

int FindDevice();

#if !defined _WIN32 && !defined __CYGWIN__

struct hiddev_report_info rep_info_i,rep_info_u;
struct hiddev_usage_ref_multi ref_multi_i,ref_multi_u;
struct hiddev_devinfo device_info;

#else
	#define write()	Result = WriteFile(WriteHandle,bufferU,n,&BytesWritten,NULL);
	#define read()	Result = ReadFile(ReadHandle,bufferI,n,&NumberOfBytesRead,(LPOVERLAPPED) &HIDOverlapped);\
					Result = WaitForSingleObject(hEventObject,50);\
					ResetEvent(hEventObject);\
					if(Result!=WAIT_OBJECT_0){\
						printf("communication timeout\r\n");\
					}

unsigned char bufferU[128],bufferI[128];
DWORD NumberOfBytesRead,BytesWritten;
ULONG Result;
HANDLE WriteHandle,ReadHandle;
OVERLAPPED HIDOverlapped;
HANDLE hEventObject;

#endif

int vid=0x04D8,pid=0x0100,info=0;

int main (int argc, char **argv) {
#define L_IT 0
#define L_EN 1
	int d=0,v=0,q=0,r=1,lang=L_EN,c,i,j,block=0;
	char path[512]="";
	char buf[256];
	for(i=0;i<256;i++) buf[i]=0;
	#if defined _WIN32
	int n=65;
	int langID=GetUserDefaultLangID();
	if((langID&0xFF)==0x10) lang=L_IT;
	#else
	int n=64;
	if(getenv("LANG")&&strstr(getenv("LANG"),"it")!=0) lang=L_IT;
	#endif
	opterr = 0;
	int option_index = 0;
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
	for (;j<n;j++) buf[j]=0;

	int fd = -1;

	if(FindDevice()<0) exit(1);

/*	if(path[0]==0){	//search all devices
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
	}*/
	if(!q){
		printf("-> ");
	 	for(i=0;i<j;i++) printf("%02X ",(unsigned char)buf[i]);
		printf("\n");
	}
	if(v) printf("%slocking read\n",block?"B":"Non b");

#if !defined _WIN32 && !defined __CYGWIN__
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
#else
	bufferU[0]=0;
	for(i=0;i<n;i++) bufferU[i+1]=buf[i];
	for(j=0;j<r;j++){
		Sleep(d);
		write();
		read();
		if(!q) printf("<- ");
		for(i=1;i<n;i++) printf("%02X ",bufferI[i]);
		printf("\n");
	}

#endif
	return 0;
}



int FindDevice(){
#if !defined _WIN32 && !defined __CYGWIN__
	struct hiddev_devinfo device_info;
	int i;
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
			printf(strings[S_noprog]);
			return -1;
		}
	}
	else{	//user supplied path
		if ((fd = open(path, O_RDONLY )) < 0) {
			printf(strings[S_DevPermission],path);
			exit(1);
		}
		ioctl(fd, HIDIOCGDEVINFO, &device_info);
		if(device_info.vendor!=vid||device_info.product!=pid){
			printf(strings[S_noprog]);
			return -1;
		}
	}
	printf(strings[S_progDev],path);

	rep_info_u.report_type=HID_REPORT_TYPE_OUTPUT;
	rep_info_i.report_type=HID_REPORT_TYPE_INPUT;
	rep_info_u.report_id=rep_info_i.report_id=HID_REPORT_ID_FIRST;
	rep_info_u.num_fields=rep_info_i.num_fields=1;
	ref_multi_u.uref.report_type=HID_REPORT_TYPE_OUTPUT;
	ref_multi_i.uref.report_type=HID_REPORT_TYPE_INPUT;
	ref_multi_u.uref.report_id=ref_multi_i.uref.report_id=HID_REPORT_ID_FIRST;
	ref_multi_u.uref.field_index=ref_multi_i.uref.field_index=0;
	ref_multi_u.uref.usage_index=ref_multi_i.uref.usage_index=0;
	ref_multi_u.num_values=ref_multi_i.num_values=DIMBUF;

#else
	char string[256];
	PSP_DEVICE_INTERFACE_DETAIL_DATA detailData;
	HANDLE DeviceHandle;
	HANDLE hDevInfo;
	GUID HidGuid;
	int MyDeviceDetected;
	char MyDevicePathName[1024];
	ULONG Length;
	ULONG Required;
	typedef struct _HIDD_ATTRIBUTES {
	    ULONG   Size;
	    USHORT  VendorID;
	    USHORT  ProductID;
	    USHORT  VersionNumber;
	} HIDD_ATTRIBUTES, *PHIDD_ATTRIBUTES;

	typedef void (__stdcall*GETHIDGUID) (OUT LPGUID HidGuid);
	typedef BOOLEAN (__stdcall*GETATTRIBUTES)(IN HANDLE HidDeviceObject,OUT PHIDD_ATTRIBUTES Attributes);
	typedef BOOLEAN (__stdcall*SETNUMINPUTBUFFERS)(IN  HANDLE HidDeviceObject,OUT ULONG  NumberBuffers);
	typedef BOOLEAN (__stdcall*GETNUMINPUTBUFFERS)(IN  HANDLE HidDeviceObject,OUT PULONG  NumberBuffers);
	typedef BOOLEAN (__stdcall*GETFEATURE) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*SETFEATURE) (IN  HANDLE HidDeviceObject, IN PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETREPORT) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*SETREPORT) (IN  HANDLE HidDeviceObject, IN PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETMANUFACTURERSTRING) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETPRODUCTSTRING) (IN  HANDLE HidDeviceObject, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	typedef BOOLEAN (__stdcall*GETINDEXEDSTRING) (IN  HANDLE HidDeviceObject, IN ULONG  StringIndex, OUT PVOID ReportBuffer, IN ULONG ReportBufferLength);
	HIDD_ATTRIBUTES Attributes;
	SP_DEVICE_INTERFACE_DATA devInfoData;
	int LastDevice = FALSE;
	int MemberIndex = 0;
	LONG Result;
	//char UsageDescription[256];

	Length=0;
	detailData=NULL;
	DeviceHandle=NULL;

	HMODULE hHID=0;
	GETHIDGUID HidD_GetHidGuid=0;
	GETATTRIBUTES HidD_GetAttributes=0;
	SETNUMINPUTBUFFERS HidD_SetNumInputBuffers=0;
	GETNUMINPUTBUFFERS HidD_GetNumInputBuffers=0;
	GETFEATURE HidD_GetFeature=0;
	SETFEATURE HidD_SetFeature=0;
	GETREPORT HidD_GetInputReport=0;
	SETREPORT HidD_SetOutputReport=0;
	GETMANUFACTURERSTRING HidD_GetManufacturerString=0;
	GETPRODUCTSTRING HidD_GetProductString=0;
	hHID = LoadLibrary("hid.dll");
	if(!hHID){
		printf("Can't find hid.dll");
		return 0;
	}
	HidD_GetHidGuid=(GETHIDGUID)GetProcAddress(hHID,"HidD_GetHidGuid");
	HidD_GetAttributes=(GETATTRIBUTES)GetProcAddress(hHID,"HidD_GetAttributes");
	HidD_SetNumInputBuffers=(SETNUMINPUTBUFFERS)GetProcAddress(hHID,"HidD_SetNumInputBuffers");
	HidD_GetNumInputBuffers=(GETNUMINPUTBUFFERS)GetProcAddress(hHID,"HidD_GetNumInputBuffers");
	HidD_GetFeature=(GETFEATURE)GetProcAddress(hHID,"HidD_GetFeature");
	HidD_SetFeature=(SETFEATURE)GetProcAddress(hHID,"HidD_SetFeature");
	HidD_GetInputReport=(GETREPORT)GetProcAddress(hHID,"HidD_GetInputReport");
	HidD_SetOutputReport=(SETREPORT)GetProcAddress(hHID,"HidD_SetOutputReport");
	HidD_GetManufacturerString=(GETMANUFACTURERSTRING)GetProcAddress(hHID,"HidD_GetManufacturerString");
	HidD_GetProductString=(GETPRODUCTSTRING)GetProcAddress(hHID,"HidD_GetProductString");
	if(HidD_GetHidGuid==NULL\
		||HidD_GetAttributes==NULL\
		||HidD_GetFeature==NULL\
		||HidD_SetFeature==NULL\
		||HidD_GetInputReport==NULL\
		||HidD_SetOutputReport==NULL\
		||HidD_GetManufacturerString==NULL\
		||HidD_GetProductString==NULL\
		||HidD_SetNumInputBuffers==NULL\
		||HidD_GetNumInputBuffers==NULL) return -1;


	HMODULE hSAPI=0;
	hSAPI = LoadLibrary("setupapi.dll");
	if(!hSAPI){
		printf("Can't find setupapi.dll");
		return 0;
	}
	typedef HDEVINFO (WINAPI* SETUPDIGETCLASSDEVS) (CONST GUID*,PCSTR,HWND,DWORD);
	typedef BOOL (WINAPI* SETUPDIENUMDEVICEINTERFACES) (HDEVINFO,PSP_DEVINFO_DATA,CONST GUID*,DWORD,PSP_DEVICE_INTERFACE_DATA);
	typedef BOOL (WINAPI* SETUPDIGETDEVICEINTERFACEDETAIL) (HDEVINFO,PSP_DEVICE_INTERFACE_DATA,PSP_DEVICE_INTERFACE_DETAIL_DATA_A,DWORD,PDWORD,PSP_DEVINFO_DATA);
	typedef BOOL (WINAPI* SETUPDIDESTROYDEVICEINFOLIST) (HDEVINFO);
	SETUPDIGETCLASSDEVS SetupDiGetClassDevsA=0;
	SETUPDIENUMDEVICEINTERFACES SetupDiEnumDeviceInterfaces=0;
	SETUPDIGETDEVICEINTERFACEDETAIL SetupDiGetDeviceInterfaceDetailA=0;
	SETUPDIDESTROYDEVICEINFOLIST SetupDiDestroyDeviceInfoList=0;
	SetupDiGetClassDevsA=(SETUPDIGETCLASSDEVS) GetProcAddress(hSAPI,"SetupDiGetClassDevsA");
	SetupDiEnumDeviceInterfaces=(SETUPDIENUMDEVICEINTERFACES) GetProcAddress(hSAPI,"SetupDiEnumDeviceInterfaces");
	SetupDiGetDeviceInterfaceDetailA=(SETUPDIGETDEVICEINTERFACEDETAIL) GetProcAddress(hSAPI,"SetupDiGetDeviceInterfaceDetailA");
	SetupDiDestroyDeviceInfoList=(SETUPDIDESTROYDEVICEINFOLIST) GetProcAddress(hSAPI,"SetupDiDestroyDeviceInfoList");
	if(SetupDiGetClassDevsA==NULL\
		||SetupDiEnumDeviceInterfaces==NULL\
		||SetupDiDestroyDeviceInfoList==NULL\
		||SetupDiGetDeviceInterfaceDetailA==NULL) return -1;


	/*
	The following code is adapted from Usbhidio_vc6 application example by Jan Axelson
	for more information see see http://www.lvr.com/hidpage.htm
	*/

	/*
	API function: HidD_GetHidGuid
	Get the GUID for all system HIDs.
	Returns: the GUID in HidGuid.
	*/
	HidD_GetHidGuid(&HidGuid);

	/*
	API function: SetupDiGetClassDevs
	Returns: a handle to a device information set for all installed devices.
	Requires: the GUID returned by GetHidGuid.
	*/
	hDevInfo=SetupDiGetClassDevs(&HidGuid,NULL,NULL,DIGCF_PRESENT|DIGCF_INTERFACEDEVICE);
	devInfoData.cbSize = sizeof(devInfoData);

	//Step through the available devices looking for the one we want.
	//Quit on detecting the desired device or checking all available devices without success.
	MemberIndex = 0;
	LastDevice = FALSE;
	do
	{
		/*
		API function: SetupDiEnumDeviceInterfaces
		On return, MyDeviceInterfaceData contains the handle to a
		SP_DEVICE_INTERFACE_DATA structure for a detected device.
		Requires:
		The DeviceInfoSet returned in SetupDiGetClassDevs.
		The HidGuid returned in GetHidGuid.
		An index to specify a device.
		*/
		Result=SetupDiEnumDeviceInterfaces (hDevInfo, 0, &HidGuid, MemberIndex, &devInfoData);
		if (Result != 0)
		{
			//A device has been detected, so get more information about it.
			/*
			API function: SetupDiGetDeviceInterfaceDetail
			Returns: an SP_DEVICE_INTERFACE_DETAIL_DATA structure
			containing information about a device.
			To retrieve the information, call this function twice.
			The first time returns the size of the structure in Length.
			The second time returns a pointer to the data in DeviceInfoSet.
			Requires:
			A DeviceInfoSet returned by SetupDiGetClassDevs
			The SP_DEVICE_INTERFACE_DATA structure returned by SetupDiEnumDeviceInterfaces.

			The final parameter is an optional pointer to an SP_DEV_INFO_DATA structure.
			This application doesn't retrieve or use the structure.
			If retrieving the structure, set
			MyDeviceInfoData.cbSize = length of MyDeviceInfoData.
			and pass the structure's address.
			*/
			//Get the Length value.
			//The call will return with a "buffer too small" error which can be ignored.
			Result = SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInfoData, NULL, 0, &Length, NULL);

			//Allocate memory for the hDevInfo structure, using the returned Length.
			detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Length);

			//Set cbSize in the detailData structure.
			detailData -> cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

			//Call the function again, this time passing it the returned buffer size.
			Result = SetupDiGetDeviceInterfaceDetail(hDevInfo, &devInfoData, detailData, Length,&Required, NULL);

			// Open a handle to the device.
			// To enable retrieving information about a system mouse or keyboard,
			// don't request Read or Write access for this handle.
			/*
			API function: CreateFile
			Returns: a handle that enables reading and writing to the device.
			Requires:
			The DevicePath in the detailData structure
			returned by SetupDiGetDeviceInterfaceDetail.
			*/
			DeviceHandle=CreateFile(detailData->DevicePath,
				0, FILE_SHARE_READ|FILE_SHARE_WRITE,
				(LPSECURITY_ATTRIBUTES)NULL,OPEN_EXISTING, 0, NULL);

			/*
			API function: HidD_GetAttributes
			Requests information from the device.
			Requires: the handle returned by CreateFile.
			Returns: a HIDD_ATTRIBUTES structure containing
			the Vendor ID, Product ID, and Product Version Number.
			Use this information to decide if the detected device is
			the one we're looking for.
			*/

			//Set the Size to the number of bytes in the structure.
			Attributes.Size = sizeof(Attributes);
			Result = HidD_GetAttributes(DeviceHandle,&Attributes);

			//Is it the desired device?
			MyDeviceDetected = FALSE;
			if (Attributes.VendorID == vid)
			{
				if (Attributes.ProductID == pid)
				{
					//Both the Vendor ID and Product ID match.
					MyDeviceDetected = TRUE;
					strcpy(MyDevicePathName,detailData->DevicePath);

					// Get a handle for writing Output reports.
					WriteHandle=CreateFile(detailData->DevicePath,
						GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE,
						(LPSECURITY_ATTRIBUTES)NULL,OPEN_EXISTING,0,NULL);

					//Get a handle to the device for the overlapped ReadFiles.
					ReadHandle=CreateFile(detailData->DevicePath,
						GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,(LPSECURITY_ATTRIBUTES)NULL,
						OPEN_EXISTING,FILE_FLAG_OVERLAPPED,NULL);

					if (hEventObject) CloseHandle(hEventObject);
					hEventObject = CreateEvent(NULL,TRUE,TRUE,"");

					//Set the members of the overlapped structure.
					HIDOverlapped.hEvent = hEventObject;
					HIDOverlapped.Offset = 0;
					HIDOverlapped.OffsetHigh = 0;
					Result=HidD_SetNumInputBuffers(DeviceHandle,64);
				}
				else
					//The Product ID doesn't match.
					CloseHandle(DeviceHandle);
			}
			else
				//The Vendor ID doesn't match.
				CloseHandle(DeviceHandle);
		//Free the memory used by the detailData structure (no longer needed).
		free(detailData);
		}
		else
			//SetupDiEnumDeviceInterfaces returned 0, so there are no more devices to check.
			LastDevice=TRUE;
		//If we haven't found the device yet, and haven't tried every available device,
		//try the next one.
		MemberIndex = MemberIndex + 1;
	} //do
	while ((LastDevice == FALSE) && (MyDeviceDetected == FALSE));
	//Free the memory reserved for hDevInfo by SetupDiClassDevs.
	SetupDiDestroyDeviceInfoList(hDevInfo);

	if (MyDeviceDetected == FALSE){
		printf("Can't find device\n");
		return -1;
	}

	if(info){
		printf("Device detected: vid=0x%04X pid=0x%04X\nPath: %s\n",vid,pid,MyDevicePathName);
		if(HidD_GetManufacturerString(DeviceHandle,string,sizeof(string))==TRUE) wprintf(L"Manufacturer string: %s\n",string);
		if(HidD_GetProductString(DeviceHandle,string,sizeof(string))==TRUE) wprintf(L"Product string: %s\n",string);
	}
#endif
	return 0;
}
