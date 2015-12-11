#ifndef COMMANAGE_H
#define COMMANAGE_H
#include "datatype.h"


#define SERIAL_BUF_SIZE 1024

#define GPS_SENSOR_COM "/dev/ttyLP1"
#define CONTROL_COM "/dev/ttyLP3"
#define HIGHT_SENOR_COM "/dev/ttyS3"


#define GPS_SENSOR 0
#define CONTROL_DEVICE 1
#define HIGH_SENSOR 2


#define BUF_SIZE_CTRL 4096

/*
typedef struct serial_buf_queue_s
{
	uint8 buf[SERIAL_BUF_SIZE];
	uint32 head;
	uint32 tail;
	uint32 len;
	pthread_mutex_t lock;
	pthread_cond_t cond;
} serial_buf_queue_s;
*/
typedef struct frame_info
{

	uint32 bytes_received;
	uint32 frame_size;

}frame_info;

typedef struct frame_wait_confirm
{

	uint8 *data;        // pointer to buffer storing the data waiting to be processed
	uint8 type;        //frame type waiting to be confirmed
	uint16 frame_num;   // total number of frames that a whole data packet is divided
	uint16 frame_id;    // which frame it is in a data packet
	uint32 data_size;

}frame_wait_confirm;

 int control_fd ;
 int gps_fd ;

int sensor_open();
void sensor_close();

#endif
