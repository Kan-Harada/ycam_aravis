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
    GMainLoop *main_loop;	// wait for finish program
    int	frames;				// frame id
    int counter;			// counter for each capture for ps
    int	ack;				// ack for capture cmd and result
    int mode;				// 0=transfer raw data,1=transfer w,bin,ph1,ph2
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
	cout<<ret<<endl;
	return ret;
}
/* change trigger mode of projector */
void pset_TrigMode(int f) {
	char cmd[8];
	sprintf(cmd,"a%d\n",f ? 1: 0);
	uart_write(cmd);
	usleep(100000);
	uart_read();
	cout<<"# Set Tigger Mode: "<<f<<endl;
}
/* load projector pattern(wait for 3 seconds after setting) */
void pset_PatternLoad(int ptn) {
	char cmd[8];
	sprintf(cmd,"z%d\n",ptn);
	uart_write(cmd);
	usleep(100000);
	uart_read();
	cout<<"# Load Pattern: "<<ptn<<endl;
}
/* set projector exposure time(call pattern setting after setting exptime) */
void pset_ExposureTime(int et) {
	char cmd[8];
	sprintf(cmd,"x%d\n",et);
	uart_write(cmd);
	usleep(100000);
	uart_read();
	cout<<"# Set ExposureTime: "<<et<<endl;
}
/* set projector brightness */
void pset_Intensity(int intens) {
	char cmd[32];
	sprintf(cmd,"i%02x%02x%02x\n",intens,intens,intens);
	uart_write(cmd);
	usleep(100000);
	uart_read();
	cout<<"# Set Intensity: "<<intens<<endl;
}
void pset_Interval(int it) {
	char cmd[8];
	sprintf(cmd,"o%d\n",it);
	uart_write(cmd);
	usleep(100000);
	uart_read();
	cout<<"# Set Interval: "<<it<<endl;
}
void pset_capture(void) {
	uart_write("o2\n");
	usleep(100000);
	uart_read();
	cout<<"# Exec Capture"<<endl;
}
void pget_temp(void) {
	uart_write("g\n");
	usleep(100000);
	cout<<"# Get Temperature: "<<uart_read()<<endl;
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
	static int i;
	ApplicationData *pData=(ApplicationData*)data;
	if(loop_exit==TRUE) {
 		g_main_loop_quit(pData->main_loop);
        return FALSE;
	}
	else {
		int tgcnt=pData->mode==1 ? 5: pData->frames;
		if(pData->counter==tgcnt) {
			for(int j=0; j<pData->counter; j++) {
				char fname[64];
				if(pData->mode==0 || (pData->mode==1 && j<3)) {
					sprintf(fname,"/tmp/raw%02d.pgm",j);
					imwrite(fname,pData->img[j]);
				}
				else if(pData->mode==1){
					sprintf(fname,"/tmp/phase%02d.dat",j-3);
					phwrite(fname,pData->img[j]);
				}
			}
		}
		cout<<"ack("<<i++<<")="<<pData->counter<<endl;
		sleep(1);
		pData->ack=pData->mode==1 ? 5: pData->frames;
		pData->counter=0;
		unsigned long wdata=pData->frames;
		if(pData->mode==1) {
			wdata|=0x0200;
			wdata=((~wdata)<<16) | wdata;
		}
// 		arv_device_write_register(gDevice,CAPTURE_CNT,wdata,NULL);
		pset_capture();
		pget_temp();
	}
	return TRUE;
}


int main(int argc,char **argv) {
	cerr<<"------------------------------------------------"<<endl;
	cerr<<argv[0]<<":usage vga/sxga [CapCount] [Mode=0/1]"<<endl;
	cerr<<argv[0]<<":default=sxga 13 0"<<endl;
	cerr<<"Mode(1) send decoded code/phase data."<<endl;
	cerr<<"------------------------------------------------"<<endl;
	int width,height,capcnt,mode;
	if(argc>=2 && !strcmp(argv[1],"vga")) {
		width=1280; height=480;
	}
	else {
		width=2560; height=1024;
	}
	capcnt=argc>=3 ? atoi(argv[2]): 13;
	mode=argc>=4 ? atoi(argv[3]): 0;
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
	set_ycam(acquisition_fps,width==20);
	arv_camera_gv_set_packet_size(gCamera,8192);
	pset_ExposureTime(20);usleep(500000);
 	pset_Intensity(250);     usleep(500000);
 	pset_Interval(60);       usleep(500000);
   	pset_PatternLoad(1);     sleep(3);

	// open stream(GVP)
	ArvGvStream *stream=NULL;
	for(;stream==NULL;) {
		stream=CreateStream();
		if(stream==NULL) {
			cerr<<"Failed in creating stream. Retrying..."<<endl;
			sleep(1);
		}
	}

	ApplicationData ad={NULL,capcnt,0,0,mode,NULL};
	ad.img=new Mat[ad.frames];
	// get capture buffer
	for(int i=0; i<ad.frames; i++) {
		ad.img[i]=Mat_<uchar>(height,width);
	}
	g_signal_connect(stream,"new-buffer",G_CALLBACK(new_buffer_cb),&ad); // capture callback
	g_signal_connect(gDevice,"control_lost",G_CALLBACK(ctl_lost_cb),NULL); // lost camera callback
	g_timeout_add_seconds(5,periodic_task_cb,&ad); // timer callback
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
	pset_TrigMode(0);
	arv_device_execute_command(gDevice, "AcquisitionStop");
	arv_stream_set_emit_signals((ArvStream*)stream,FALSE);
	g_object_unref(stream);
 	g_object_unref(gCamera);
	return 0;
}
