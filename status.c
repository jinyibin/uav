#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "datatype.h"
#include "control.h"
#include "crc.h"
#include "serial.h"
#include "status.h"
#include "sensor.h"

static uint16 flying_status = 0;
static flying_attitude_s flying_attitude;
static uint64 fa_timestamp = 0;


uint64 get_current_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec/1000;
}

flying_attitude_s *get_flying_attitude()
{
	return &flying_attitude;
}

void set_flying_attitude(uint8 *buf)
{
	flying_attitude_s *p;
	uint8 data[GPS_FRAME_LEN-8];
	p=&flying_attitude;

	memcpy(data,buf+5,GPS_FRAME_LEN-8);//4bytes alignment of the data

	p->roll=*((float *)data);
	p->pitch=*((float *)(data+4));
	p->yaw=*((float *)(data+8));
	p->gx=*((float *)(data+12));
	p->gy=*((float *)(data+16));
	p->gz=*((float *)(data+20));
	p->ax=*((float *)(data+24));
	p->ay=*((float *)(data+28));
	p->az=*((float *)(data+32));
	p->g_time=*((unsigned int *)(data+36));
	p->vn=*((unsigned int *)(data+40));
	p->ve=*((unsigned int *)(data+44));
	p->vd=*((unsigned int *)(data+48));
	p->h=*((unsigned int *)(data+52));
	p->b_h=*((unsigned int *)(data+56));
	p->Long=*((double *)(data+60));
	p->lat=*((double *)(data+68));
	p->g_h=*((double *)(data+76));
	p->vx=*((float *)(data+84));
	p->vy=*((float *)(data+88));
	p->vz=*((float *)(data+92));
	//memcpy(&flying_attitude, buf, sizeof(flying_attitude_s));
	fa_timestamp = get_current_time();
}

void gps_time_update(uint32 g_time)
{
	
}
uint16 get_flying_status()
{
	return flying_status;
}

void set_flying_status(uint16 status)
{
	flying_status = status;
}

uint16 get_aircraft_no()
{
	return 0x0001;
}

void firmware_upgrade(uint8 *buf, uint32 size)
{
	FILE *fp = NULL;
	#define FILE_PATH "/data/backup/upgrade"
	fp = fopen(FILE_PATH, "w+");
	if (fp == NULL) {
		print_err("File %s open failed\n", FILE_PATH);
		return;
	}
	fwrite(buf, 1, size, fp);
	fclose(fp);
	fp = NULL;
}

int flying_attitude_sensor_is_active()
{
	return (get_current_time() - fa_timestamp) <= CONTROL_DUTY;
}
