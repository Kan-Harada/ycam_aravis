#include <glib.h>
#include <arv.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <assert.h>

#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>

using namespace std;
using namespace cv;

#define YCAM_VERSION	0x00000000
#define YCAM_ADDRESS	0x00000024
#define ACQUISITION_FPS	0x00101000
#define EXPOSURE_TIME	0x00101004
#define GAIN_ANALOG		0x00101008
#define GAIN_DIGITAL	0x0010100C
#define CAPTURE_CNT		0x00100020
#define SERIAL_PORT		0x00300004
enum {
	ycam_version,
	ycam_address,
	acquisition_fps,
	exposure_time,
	gain_analog,
	gain_digital,
	capture_cnt,
	serial_port
};

unsigned long ycamreg[]={
	YCAM_VERSION,
	YCAM_ADDRESS,
	ACQUISITION_FPS,
	EXPOSURE_TIME,
	GAIN_ANALOG,
	GAIN_DIGITAL,
	CAPTURE_CNT,
	SERIAL_PORT
};


enum {
	E4,
	E8,
	E16,
	E24,
	E32,
	E50
};
unsigned long camera_exposure[] {
	4000,
	8000,
	16000,
	24000,
	32000,
	49000
};

unsigned long proj_exposure[] {
	8333,
	8333,
	16000,
	24000,
	32000,
	50000
};

unsigned long fps_vga[] {
	116,
	116,
	60,
	40,
	30,
	20
};
unsigned long fps_sxga[] {
	60,
	60,
	60,
	40,
	30,
	20
};


#define PACKET_TIMEOUT	1000
#define FRAME_RETENTION	200000
static pthread_t tid;
static char keycmd[16];
static int width,height,exposure;
static std::string ip;
static gboolean arv_option_realtime = TRUE;
static gboolean arv_option_high_priority = TRUE;
static gboolean loop_exit=FALSE;
typedef struct 
{
    GMainLoop *main_loop;	// wait for finish program
    int	frames;				// frame id
    int counter;			// counter for each capture for ps
    int	ack;				// ack for capture cmd and result
    int mode;				// 0=transfer raw data,1=transfer w,bin,ph1,ph2
    int pmode;				// projector pattern(1,2,4)
	Mat *img;    			// buffers for each capture
} ApplicationData;
ApplicationData *adp;

/* catch the exiting signal */
static void set_exit(int signal) {
	cerr<<"Exit Signal"<<endl;
	loop_exit=TRUE;
}
/* catch the event for camera lost */
static void ctl_lost_cb(ArvGvDevice *dev){
    loop_exit=TRUE;
}

/* camera and camera device handler */
ArvCamera *gCamera;
ArvDevice *gDevice;

/* get camera 
 *   return the first camera on interface */
ArvCamera *get_camera(void) {
	arv_update_device_list();
	ArvCamera* camera=NULL;
	for(unsigned int i=0, j=arv_get_n_devices(); i<j; i++) {
		camera=arv_camera_new(arv_get_device_id(i));
		string vendor=arv_camera_get_vendor_name(camera);
		string model=arv_camera_get_model_name(camera);
		cout<<"Vender="<<vendor<<",Model="<<model<<endl;
		break;
		g_object_unref(camera);
	}
	return camera;
}

/* get the camera IP address via GVCP */
std::string get_ipaddress(void) {
	char ret[64];
	unsigned int val;
	arv_device_read_register(gDevice,YCAM_ADDRESS,&val,NULL);
	sprintf(ret,"%d.%d.%d.%d\n",(val>>24)&0x0ff,(val>>16)&0x0ff,(val>>8)&0x0ff,val&0x0ff);
	return ret;
}

/* set camera register */
void set_ycam(int addr,int val) {
	arv_device_write_register(gDevice,ycamreg[addr],val,NULL);
	guint32 dat;
	arv_device_read_register(gDevice,ycamreg[addr],(guint32*)&dat,NULL);
	if(val!=dat) {
		cout<<"failed YCAMREG["<<addr<<"]"<<val<<", result="<<dat<<endl;
	}
}

/* write setting for projector */
void uart_write(const std::string wstr) {
	for(int i=0; wstr.c_str()[i]!=0; i++) {
		unsigned long dat=(~wstr.c_str()[i]<<16) | wstr.c_str()[i];
		arv_device_write_register(gDevice,SERIAL_PORT,dat,NULL);
		if(wstr.c_str()[i]=='\n') break;
	}
	if(wstr.c_str()[0]=='d' || wstr.c_str()[0]=='D') sleep(1);
	else usleep(200000);
}
/* read projector uarwt buffer */
std::string uart_read(void) {
	static char *ret=new char[8192];
	ret[0]=0;
	unsigned long dat;
	for(int i=0; i<8191 ; i++) {
		arv_device_read_register(gDevice,SERIAL_PORT,(guint32*)&dat,NULL);
		ret[i]=dat & 0xff;
		if(ret[i]==0) break;
		ret[i+1]=0;
	}
	return ret;
}
/* load projector pattern(wait for 3 seconds after setting) */
void pset_PatternLoad(int ptn) {
	char cmd[8];
	sprintf(cmd,"z%d\n",ptn);
	uart_write(cmd);
}
/* set projector exposure time(call pattern setting after setting exptime) */
void pset_ExposureTime(int et) {
	char cmd[8];
	sprintf(cmd,"x%d\n",et);
	uart_write(cmd);
}
/* set projector brightness */
void pset_Intensity(int intens) {
	char cmd[32];
	sprintf(cmd,"i%02x%02x%02x\n",intens,intens,intens);
	uart_write(cmd);
}
void pset_Reset(void) {
	uart_write("r\n");
}
/* validate setting - execute after changing parameter */
int pset_validate(void) {
	uart_write("v\n");
	usleep(500000);
	int ret=atoi(uart_read().c_str())&0x1f;
	if(ret!=0) {
		cout<<"validate result="<<ret<<endl;
	}
}
/* get LED temperature */
int pset_gettemp(void) {
	uart_write("g\n");
	usleep(100000);
	return atoi(uart_read().c_str());
}
/* get LED Intensity */
int pset_getintensity(void) {
	uart_write("i\n");
	usleep(100000);
	return atoi(uart_read().c_str());
}
void pset_getversion(void) {
	uart_write("y\n");
	usleep(100000);
	char v[32];
	strcpy(v,uart_read().c_str());
	char *p=strchr(v,'\n');
	*p=0;
	cout<<"firmware version="<<v<<endl;
}
/* stop(0)/go(2) execute after validating */ 
void pset_stopgo(int n) {
	char cmd[8];
	sprintf(cmd,"q%d\n",n);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}

void set_exposure(int et, int pmode) {
	set_ycam(acquisition_fps,10);
	set_ycam(exposure_time,camera_exposure[et]);
	if(width==2560) {
		set_ycam(acquisition_fps,fps_sxga[et]);
	} else { 
		set_ycam(acquisition_fps,fps_vga[et]);
	}
	pset_stopgo(0);
	int vres=1;
	do {
		pset_ExposureTime(proj_exposure[et]);
		pset_PatternLoad(pmode);
		vres=pset_validate();
	} while(vres);
	pset_stopgo(2);
}

/* save phase data */
void phwrite(char *fname,Mat &m) {
	FILE *fp=fopen(fname,"wb");
	for(int i=0; i<m.rows; i++) {
		unsigned short *sp=(unsigned short*)m.ptr<unsigned char>(i);
		for(int j=0; j<m.cols; j++) {
			float val=(float)*sp++/(float)0x1fff;
			fwrite(&val,sizeof(float),1,fp);
		}
	}
	fclose(fp);
}

/* convert aravis image to opencv Mat image */
void copyArvImage(ArvBuffer * buffer, Mat &m) {
	size_t buffer_size;
	char *buffer_dat=(char*)arv_buffer_get_data(buffer,&buffer_size);
	int _width; int _height;
	arv_buffer_get_image_region(buffer,NULL,NULL,&_width,&_height);
	int bit_depth=ARV_PIXEL_FORMAT_BIT_PER_PIXEL(arv_buffer_get_image_pixel_format(buffer));

	if(m.cols!=_width || m.rows!=_height) {
		cerr<<"new Mat("<<m.cols<<","<<m.rows<<"),("<<_width<<","<<_height<<")"<<endl;
		m=Mat_<unsigned char>(_height,_width);
	}
	for(int i=0; i<_height; i++) {
		memcpy(m.ptr<unsigned char>(i),&buffer_dat[i*_width],_width);
	}
}

/* stream callback called, initialize stream */
static void stream_cb(void* args, ArvStreamCallbackType type, ArvBuffer*buf) {
	if (type == ARV_STREAM_CALLBACK_TYPE_INIT) {
		if(arv_option_realtime) {
			if (!arv_make_thread_realtime (10))
				printf ("This system doesn't support realtime stream thread\n");
			else
				printf ("Succeeded in making stream thread realtime\n");
		} else if (arv_option_high_priority) {
			if (!arv_make_thread_high_priority (-10))
				printf ("This system doesn't support high priority stream thread\n");
			else 
				printf ("Succeeded in making stream thread high priority\n");
		}
	}
}

/* create stream (open GVSP)*/
ArvGvStream *CreateStream(void) {
	ArvGvStream *stream=(ArvGvStream*)arv_device_create_stream(gDevice,stream_cb,NULL);
	if(stream) {
		g_object_set(stream,"packet-timeout",PACKET_TIMEOUT,"frame-retention",FRAME_RETENTION,NULL);
		gint n_payload=arv_camera_get_payload(gCamera);
		for(int i=0; i<50; i++) {
			ArvBuffer *buff=arv_buffer_new(n_payload,NULL);
			arv_stream_push_buffer((ArvStream*)stream,buff);
		}
	}
	return stream;
}

/* stream callback */
static void new_buffer_cb (ArvStream *stream,ApplicationData *data) {
	ApplicationData *pData=(ApplicationData*)data;
	ArvBuffer *buffer;
	buffer=arv_stream_try_pop_buffer(stream);
	if (buffer!=NULL) {
		if(arv_buffer_get_status(buffer)==ARV_BUFFER_STATUS_SUCCESS) {
			copyArvImage(buffer,pData->img[pData->counter]);
			pData->ack--;
			pData->counter++;
		}
		arv_stream_push_buffer(stream,buffer);
	}
}

/* timer callback - create capture event*/
static gboolean periodic_task_cb(void *data) {
	static int i,loopcnt=0,_m=0;
	static int run=1,timeout=0;
	ApplicationData *pData=(ApplicationData*)data;
	static int pm=pData->pmode;
	int _pm=-1;

	if(loopcnt++==0) {
		set_exposure(exposure,pm);
	}
	if(loop_exit==TRUE) {
 		g_main_loop_quit(pData->main_loop);
//  		void *res;
//  		pthread_join(tid,&res);
        return FALSE;
	}
	else {
		int tgcnt=pData->mode==1 ? 5: pData->frames;
		if(pData->counter==tgcnt) {
			timeout=0;
// 			for(int j=0; j<pData->counter; j++) {
// 				char fname[64];
// 				if(pData->mode==0 || (pData->mode==1 && j<3)) {
// 					sprintf(fname,"/tmp/raw%02d.pgm",j);
// 					imwrite(fname,pData->img[j]);
// 				}
// 				else if(pData->mode==1){
// 					sprintf(fname,"/tmp/phase%02d.dat",j-3);
// 					phwrite(fname,pData->img[j]);
// 				}
// 			}
		}
		if(keycmd[0]!=0) {
			cout<<"keycmd="<<keycmd<<endl;
			if(keycmd[0]=='q') {
				loop_exit=TRUE;
			}
			else if(!strncmp(keycmd,"run",3)) {
				run=1;
			}
			else if(!strncmp(keycmd,"stop",4)) {
				run=0;
			}
			else if(keycmd[0]=='m') {
				_pm=keycmd[1]-'0';
				if(_pm!=1 && _pm!=2 && _pm!=4) _pm=4;
			}
			else if(!strncmp(keycmd,"reset",5)) {
				pset_Reset();
			}
			else if(keycmd[0]=='b') {
				cout<<"LED Intensity="<<pset_getintensity()<<endl;
			}
			memset(keycmd,0,sizeof(keycmd));
		}
		int m=mean(pData->img[1])[0];
		cout<<"ack("<<i++<<")="<<pData->counter<<", mean="<<m<<endl;
		if((pData->counter!=0 && m<40*exposure) || timeout) {
			if(timeout) cerr<<"## timeout error ##"<<endl;
			loop_exit=TRUE;
			g_main_loop_quit(pData->main_loop);
			return FALSE;
		}
		else if(run){
			timeout=1;
			pData->ack=pData->mode==1 ? 5: pData->frames;
			pData->counter=0;
			int n=13;
			unsigned long wdata;
			if(pm==1) {
				wdata=pData->frames=13;
				n=13;
			}
			else if(pm==2) {
				wdata=pData->frames=1;
				n=1;
			}
			else {
				wdata=pData->frames=14;
				n=14;
			}
			if(pData->mode==1) {
				wdata|=0x0200;
				wdata=((~wdata)<<16) | wdata;
			}
			arv_device_write_register(gDevice,CAPTURE_CNT,wdata,NULL);
			usleep(300000);
			if(_pm!=-1) {
				pm=_pm;
				set_exposure(exposure,pm);
				cout<<loopcnt<<":change pattern to "<<pm<<endl;
			}
		}
	}
	cout<<"temp="<<pset_gettemp()<<endl;
	return TRUE;
}

void *cmd_input(void*) {
	char buf[16];
	memset(keycmd,0,16);
	fcntl(0,F_SETFL,O_NONBLOCK);
	do {
		int n=scanf("%s",buf);
		if(keycmd[0]==0) {
			strcpy(keycmd,buf);
			buf[0]=0;
		}
	} while(loop_exit!=TRUE);
}

int main(int argc,char **argv) {
	int capcnt,mode,intensity;
	width=2560; height=1024;
	capcnt=14;
	exposure=4;
	intensity=255;
	mode=0;

	//command input thread
	pthread_create(&tid,NULL,cmd_input,(void*)NULL);


	gCamera=get_camera();
	if(gCamera==NULL) {
		cout<<"No Camera. exit!"<<endl;
		return -1;
	}
	gDevice=arv_camera_get_device(gCamera);
	unsigned int val;
	arv_device_read_register(gDevice,YCAM_VERSION,&val,NULL);
	printf("version   : 0x%0X\n",val);
	ip=get_ipaddress();
	cout<<"IP Address: "<<ip<<endl;
	uart_read();
	pset_getversion();

	//Setup YCAM3D
	arv_device_set_integer_feature_value(gDevice,"TriggerMode",1);
	arv_device_set_integer_feature_value(gDevice,"Height",height);
	arv_device_set_integer_feature_value(gDevice,"Width",width);
	arv_camera_gv_set_packet_size(gCamera,8192);
 	pset_Intensity(intensity);

	// open stream(GVP)
	ArvGvStream *stream=NULL;
	for(;stream==NULL;) {
		stream=CreateStream();
		if(stream==NULL) {
			cerr<<"Failed in creating stream. Retrying..."<<endl;
			sleep(1);
		}
	}

	ApplicationData ad={NULL,capcnt,0,0,mode,capcnt==13 ? 1: 4,NULL};
	adp=&ad;
	ad.img=new Mat[ad.frames];
	// get capture buffer
	for(int i=0; i<ad.frames; i++) {
		ad.img[i]=Mat_<uchar>(height,width);
	}
	g_signal_connect(stream,"new-buffer",G_CALLBACK(new_buffer_cb),&ad); // capture callback
	g_signal_connect(gDevice,"control_lost",G_CALLBACK(ctl_lost_cb),NULL); // lost camera callback
 	g_timeout_add_seconds(1,periodic_task_cb,&ad); // timer callback
	arv_stream_set_emit_signals((ArvStream*)stream,TRUE);

	void (*sigint_handler_old)(int)=signal(SIGINT,set_exit);

	arv_device_execute_command(gDevice,"AcquisitionStart");
	ad.main_loop = g_main_loop_new (NULL,FALSE);
	g_main_loop_run(ad.main_loop); // waiting

	cerr<<"Exiting ... "<<argv[0]<<endl;
	signal(SIGINT,sigint_handler_old);
	// wait for all capture done
	while(ad.ack>0) {
		usleep(10000);
	}
	g_main_loop_unref(ad.main_loop);
	arv_device_execute_command(gDevice, "AcquisitionStop");
	arv_stream_set_emit_signals((ArvStream*)stream,FALSE);
	g_object_unref(stream);
 	g_object_unref(gCamera);
	return 0;
}
