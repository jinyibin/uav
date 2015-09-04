#ifndef SENSOR_H
#define SENSOR_H
#include "datatype.h"


#define SERIAL_BUF_SIZE 1024

#define GPS_SENSOR_COM "/dev/ttyLP1"
#define CONTROL_COM "/dev/ttyLP3"
#define HIGHT_SENOR_COM "/dev/ttyS3"


#define GPS_SENSOR 0
#define CONTROL_DEVICE 1
#define HIGH_SENSOR 2

#define BUF_SIZE_GPS 512
#define BUF_SIZE_CTRL 4096
#define GPS_FRAME_LEN 104


#define GPS_FRAME_START1 0xFF
#define GPS_FRAME_START2 0x02
#define GPS_FRAME_START3 0x90
#define GPS_FRAME_END 0x03
#define GPS_FRAME_MINIMUM_LEN 104
#define GPS_FRAME_MAX_LEN     200

#define GPS_FRAME_LEN_NO_DATA 8       // total length of field other than data
#define GPS_FRAME_CRC_LEN    GPS_FRAME_LEN-5
#define GPS_FRAME_MASK_CMD   2        // cmd field position start from 0
#define GPS_FRAME_MASK_FRAME_SIZE 3   // frame size field position start from 0



typedef struct serial_buf_queue_s
{
	uint8 buf[SERIAL_BUF_SIZE];
	uint32 head;
	uint32 tail;
	uint32 len;
	pthread_mutex_t lock;
	pthread_cond_t cond;
} serial_buf_queue_s;

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

int sensor_open();
void sensor_close();
int control_data_parse(unsigned char *buf, frame_info *frame_info,frame_wait_confirm *frame_wait_confirm);
void gps_data_parse(unsigned char *buf, frame_info *frame_info);
int control_cmd_send(uint8 *buf,uint32 buf_size);

#endif
