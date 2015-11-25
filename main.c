#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datatype.h"
#include "control.h"
#include "interface.h"
#include "fpga.h"
#include "adc.h"
#include "sensor.h"
#include "serial.h"
#include "status.h"
#include <unistd.h>
#include <sys/time.h>
/*-----------------------------------------------------------------------*/
    uint32 command;
    uint32 frequency;
    uint32 counter_global=0;

int main( int argc,char *argv[])
{
    uint32 period;
	if(argc<2){
	           printf(" usage: uav [command] [gap] ]\n");
	           printf(" [command]: 0--normal mode,used when run in the air\n");
	           printf(" [command]: 1--test mode,used for platform test.use this mode  \n");
	           printf("               when no IMU connected ,\n");
	           printf(" [command]: 2--manual mode,used when capturing PWM data\n");
	           printf(" [frequency]: the frequency (ms/frame) in which UAV send fly status data\n");
	        }
	command = atol(argv[1]);
	frequency = atol(argv[2]);
    period= frequency * 1000;
    printf("version:20151124-1926\n");

	int ret = -1;
	uint32 counter=0;
	struct timeval tpStart,tpEnd;
	uint64 start_time, stop_time;
	while (ret < 0 ){
		ret = poweron_self_check();
	}
	while (get_flying_status() < AIRCRAFT_READY) {
		usleep(500000);
		flying_status_return();
	}
	/*
	while (get_flying_status() < AIRCRAFT_TAKEOFF) {
		flying_status_return();
		usleep(40000);
	}
	*/
	auto_flying_start();
	usleep(10000);
	while (1) {

		gettimeofday(&tpStart, NULL);
		start_time = tpStart.tv_sec * 1000000 + tpStart.tv_usec;

		flying_status_return();
		/*
		//save data every 1 minute
		 counter++;
		if(counter==3000){
			counter=0;
			fclose(fp_fly_status);
		    if((fp_fly_status=fopen("fly_status","ab+"))==NULL){
		      printf("reopen file failed :fly_status\n");
		    }
		}
        */
	    gettimeofday(&tpStart, NULL);
		stop_time = tpStart.tv_sec * 1000000 + tpStart.tv_usec;
		if ((stop_time  - start_time) < period) {
			usleep(period-(stop_time  - start_time));
		} else {
			print_err("flying status return cost too much time\n");
		}
	}
	sensor_close();
	adc_close();
	spi_close();
	exit(0);
	return 0;
}
