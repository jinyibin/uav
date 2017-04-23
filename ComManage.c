#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include "datatype.h"
#include "control.h"
#include "crc.h"
#include "serial.h"
#include "ComManage.h"
#include "ProtocolImu.h"
#include "fpga.h"
#include <fcntl.h>
#include "sbg_ellipse.h"


static int running = 0;

#define MAX(a,b) (a>b?a:b)

static pthread_t recv_pid;
static void *sensor_data_collect();

int sensor_open()
{
#ifdef USE_SBG_ELLIPSE
	gps_fd = serial_open(GPS_SENSOR_COM, 921600, 0, 1);
#endif
#ifdef USE_SBG_IG500
	gps_fd = serial_open(GPS_SENSOR_COM, 230400, 0, 1);
#endif
    control_fd = serial_open(CONTROL_COM, 115200, 0, 1);
    leddar_fd = serial_open(LEDDAR_COM, 115200, 0, 1);
//	if (gps_fd < 0 || high_fd < 0 || control_fd < 0) {
	if (gps_fd < 0 ) {

		print_err("sensor open failed, gps_fd = %d\n", gps_fd);
		return SERIAL_GPS_OPEN_FAILED;
	}

	if (control_fd < 0) {

		print_err("sensor open failed,control_fd = %d\n",  control_fd);
		return SERIAL_CTRL_OPEN_FAILED;
	}
	if (leddar_fd < 0) {

		print_err("sensor open failed,leddar_fd = %d\n",  leddar_fd);
		return SERIAL_CTRL_OPEN_FAILED;
	}

	if(fcntl(leddar_fd,F_SETFL,FNDELAY) < 0)//非阻塞，覆盖前面open的属性
	  {
	           printf("fcntl failed\n");
	   }
	running = 1;
	pthread_create(&recv_pid, NULL, sensor_data_collect, NULL);
	return 0;
}

void sensor_close()
{
	void* result = NULL;
	running = 0;
	pthread_join(recv_pid,&result);
	serial_close(gps_fd);
//	serial_close(high_fd);
	serial_close(control_fd);
}


extern uint32 command;
static void *sensor_data_collect()
{

	//int i = 0;
	//int m = 0;
	unsigned int data_len = 0;
	int maxfd = 0;
	fd_set rfds;

	unsigned char buf_gps[BUF_SIZE_GPS];          // buffer to store raw gps frame data
	unsigned char buf_ctrl[BUF_SIZE_CTRL];        // buffer to store raw control frame data
	unsigned char ctrl_frame_data[BUF_SIZE_CTRL]; // buffer to store data field of control frame


	 frame_info frame_info_gps={0,0};
	 frame_info frame_info_ctrl={0,0};

	 frame_wait_confirm frame_wait_confirm = {ctrl_frame_data,0,0};

	struct timeval tv;

	maxfd = MAX(gps_fd, control_fd);
	while (running) {
	         FD_ZERO(&rfds);
	         FD_SET(gps_fd, &rfds);
	         FD_SET(control_fd, &rfds);
	         tv.tv_sec=0;
             tv.tv_usec=80000; //80 ms
		     if (select(1 + maxfd, &rfds, NULL, NULL, &tv) > 0) {
				if (FD_ISSET(gps_fd, &rfds)) {
#ifdef USE_SBG_ELLIPSE
DATA_RECV:  		data_len=serial_data_recv_ellipse(&frame_info_gps,buf_gps);
#endif
#ifdef USE_SBG_IG500
            		data_len=serial_data_recv_gps(&frame_info_gps,buf_gps);
#endif
                    if(data_len > 0){
                        //print_debug("%4d ,frame time: %d,length : %d\n",i++, *(unsigned int *)(buf_gps+41),data_len);
#ifdef USE_SBG_ELLIPSE
                    	ellipse_data_parse(buf_gps, &frame_info_gps);
#endif
#ifdef USE_SBG_IG500
                    	gps_data_parse(buf_gps, &frame_info_gps);
#endif
                    	if(frame_info_gps.bytes_received > frame_info_gps.frame_size){
                    	     memmove(buf_gps,buf_gps+frame_info_gps.frame_size,frame_info_gps.bytes_received-frame_info_gps.frame_size);
                    	}
                    	frame_info_gps.frame_size=0;
                    	frame_info_gps.bytes_received -= data_len;
#ifdef USE_SBG_ELLIPSE
                    	if(frame_info_gps.bytes_received >  ELLIPSE_FRAME_MINIMUM_LEN)
                    		goto DATA_RECV;
#endif

                    }

				} else if (FD_ISSET(control_fd, &rfds)) {

					data_len = serial_data_recv_ctrl(&frame_info_ctrl,buf_ctrl);
					if(data_len > 0){

					    control_data_parse(buf_ctrl,&frame_info_ctrl,&frame_wait_confirm);
                     	if(frame_info_ctrl.bytes_received > frame_info_ctrl.frame_size){
					        memmove(buf_ctrl,buf_ctrl+frame_info_ctrl.frame_size,frame_info_ctrl.bytes_received-frame_info_ctrl.frame_size);
                     	}
					    frame_info_ctrl.frame_size=0;
					    frame_info_ctrl.bytes_received -= data_len;
					}
				}
		} else {
#ifdef debug
			if(command==0)
			    print_err("Device read timeout\n");// serial port received timeout.
#else
			fault_status_response(SERIAL_NO_DATA);
#endif
		}
	}

}



