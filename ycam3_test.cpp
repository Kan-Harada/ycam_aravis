#include <glib.h>
#include <arv.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <assert.h>

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

#define PACKET_TIMEOUT	1000
#define FRAME_RETENTION	200000
static std::string ip;
static gboolean arv_option_realtime = TRUE;
static gboolean arv_option_high_priority = TRUE;
static gboolean loop_exit=FALSE;
typedef struct 
{
    GMainLoop *main_loop;	// waint for finish program
    int	frames;				// frame id
    int counter;			// counter for each capture for ps
    int	ack;				// ack for capture cmd and result
	Mat *img;    			// buffers for each capture
} ApplicationData;

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
 *   return the first camera on interface
 */
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

/*
 * get the camera IP address via GVCP
 */
std::string get_ipaddress(void) {
	char ret[64];
	unsigned int val;
	arv_device_read_register(gDevice,YCAM_ADDRESS,&val,NULL);
	sprintf(ret,"%d.%d.%d.%d\n",(val>>24)&0x0ff,(val>>16)&0x0ff,(val>>8)&0x0ff,val&0x0ff);
	return ret;
}

/*
 * set camera register
 */
void set_ycam(int addr,int val) {
	arv_device_write_register(gDevice,ycamreg[addr],val,NULL);
}

/*
 * write setting for projector
 */
void uart_write(const std::string wstr) {
	for(int i=0; wstr.c_str()[i]!=0; i++) {
		unsigned long dat=(~wstr.c_str()[i]<<16) | wstr.c_str()[i];
		arv_device_write_register(gDevice,SERIAL_PORT,dat,NULL);
		if(wstr.c_str()[i]=='\n') break;
	}
	if(wstr.c_str()[0]=='d' || wstr.c_str()[0]=='D') sleep(1);
	else usleep(200000);
}
/*
 * read projector uarwt buffer
 */
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
/*
 * change trigger mode of projector
 */
void pset_TrigMode(int f) {
	char cmd[8];
	sprintf(cmd,"a%d\n",f ? 1: 0);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}
/*
 * load projector pattern(wait for 3 seconds after setting)
 */
void pset_PatternLoad(int ptn) {
	char cmd[8];
	sprintf(cmd,"z%d\n",ptn);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}
/*
 * set projector exposure time(call pattern setting after setting exptime)
 */
void pset_ExposureTime(int et) {
	char cmd[8];
	sprintf(cmd,"x%d\n",et);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}
/*
 * set projector brightness
 */
void pset_Intensity(int intens) {
	char cmd[32];
	sprintf(cmd,"i%02x%02x%02x\n",intens,intens,intens);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}
void pset_Interval(int it) {
	char cmd[8];
	sprintf(cmd,"o%d\n",it);
	uart_write(cmd);
	usleep(100000);
	uart_read();
}

void copyArvImage(ArvBuffer * buffer, Mat &m) {
	size_t buffer_size;
	char *buffer_dat=(char*)arv_buffer_get_data(buffer,&buffer_size);
	int width; int height;
	arv_buffer_get_image_region(buffer,NULL,NULL,&width,&height);
	int bit_depth=ARV_PIXEL_FORMAT_BIT_PER_PIXEL(arv_buffer_get_image_pixel_format(buffer));

	if(m.cols!=width || m.rows!=height) {
		cerr<<"new Mat("<<m.cols<<","<<m.rows<<"),("<<width<<","<<height<<")"<<endl;
		m=Mat_<unsigned char>(height,width);
	}
	for(int i=0; i<height; i++) {
		memcpy(m.ptr<unsigned char>(i),&buffer_dat[i*width],width);
	}
}

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

static gboolean periodic_task_cb(void *data) {
	static int i;
	ApplicationData *pData=(ApplicationData*)data;
	if(loop_exit==TRUE) {
 		g_main_loop_quit(pData->main_loop);
        return FALSE;
	}
	else {
		if(pData->counter==pData->frames) {
			for(int j=0; j<pData->counter; j++) {
				char fname[64];
				sprintf(fname,"/tmp/raw%02d.pgm",j);
				imwrite(fname,pData->img[j]);
			}
		}
		cout<<"ack("<<i++<<")="<<pData->counter<<endl;
		sleep(1);
		pData->ack=pData->frames;
		pData->counter=0;
		arv_device_write_register(gDevice,CAPTURE_CNT,pData->frames,NULL);
	}
	return TRUE;
}


int main(int argc,char **argv) {
	if(argc!=3) {
		cerr<<argv[0]<<":usage vga/sxga [CapCount]"<<endl;
		cerr<<argv[0]<<":default=sxga 13"<<endl;
	}
	int width,height,capcnt;
	if(argc>=2 && !strcmp(argv[1],"vga")) {
		width=1280; height=480;
	}
	else {
		width=2560; height=1024;
	}
	capcnt=argc>=3 ? atoi(argv[2]): 13;
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

	//Setup YCAM3D
	arv_device_set_integer_feature_value(gDevice,"TriggerMode",1);
	arv_device_set_integer_feature_value(gDevice,"Height",height);
	arv_device_set_integer_feature_value(gDevice,"Width",width);
	set_ycam(exposure_time,8300);
	set_ycam(acquisition_fps,width==2560 ? 60: 116);
	arv_camera_gv_set_packet_size(gCamera,8192);
	pset_TrigMode(0);        usleep(500000);
	pset_ExposureTime(8333); usleep(500000);
 	pset_Intensity(120);     usleep(500000);
 	pset_Interval(10);       usleep(500000);
   	pset_PatternLoad(1);     sleep(3);
	pset_TrigMode(1);        usleep(500000);

	ArvGvStream *stream=NULL;
	for(;stream==NULL;) {
		stream=CreateStream();
		if(stream==NULL) {
			cerr<<"Failed in creating stream. Retrying..."<<endl;
			sleep(1);
		}
	}

	ApplicationData ad={NULL,capcnt,0,0,NULL};
	ad.img=new Mat[ad.frames];
	for(int i=0; i<ad.frames; i++) {
		ad.img[i]=Mat_<uchar>(height,width);
	}
	g_signal_connect(stream,"new-buffer",G_CALLBACK(new_buffer_cb),&ad);
	g_signal_connect(gDevice,"control_lost",G_CALLBACK(ctl_lost_cb),NULL);
	g_timeout_add_seconds(2,periodic_task_cb,&ad);
	arv_stream_set_emit_signals((ArvStream*)stream,TRUE);

	void (*sigint_handler_old)(int)=signal(SIGINT,set_exit);

	arv_device_execute_command(gDevice,"AcquisitionStart");
	ad.main_loop = g_main_loop_new (NULL,FALSE);
	g_main_loop_run(ad.main_loop);

	cerr<<"Exiting ... "<<argv[0]<<endl;
	signal(SIGINT,sigint_handler_old);
	while(ad.ack>0) {
		usleep(10000);
	}
	g_main_loop_unref(ad.main_loop);
	pset_TrigMode(0);
	arv_device_execute_command(gDevice, "AcquisitionStop");
	arv_stream_set_emit_signals((ArvStream*)stream,FALSE);
	g_object_unref(stream);
 	g_object_unref(gCamera);
	return 0;
}