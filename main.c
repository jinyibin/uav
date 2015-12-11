#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datatype.h"
#include "control.h"
#include "interface.h"
#include "fpga.h"
#include "adc.h"
#include "ComManage.h"
#include "serial.h"
#include <unistd.h>
#include <sys/time.h>
/*-----------------------------------------------------------------------*/
    uint32 command;
    uint32 frequency;
    uint32 counter_global=0;

int main( int argc,char *argv[])
{

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

    printf("version:20151211-1003\n");

	int ret = -1;
	uint32 counter=0;
	struct timeval tpStart,tpEnd;
	uint64 start_time, stop_time;
	while (ret < 0 ){
		ret = poweron_self_check();
	}
	while (get_flying_status() < AIRCRAFT_READY) {
		usleep(500000);
		flying_status_return(1);
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

        if((frequency%CONTROL_PERIOD_MS)==0)
		   flying_status_return(1);
        else
           flying_status_return(0);

		//save log file every 1 minute
		 counter++;
		if(counter==(CONTROL_FREQUENCY*60*1)){
			counter=0;
			fclose(fp_fly_status);
			memset(log_file_name,sizeof(log_file_name),0);
			generate_file_name(log_file_name);
		    if((fp_fly_status=fopen(log_file_name,"wb+"))==NULL){
		      printf("can not open file:%s\n",log_file_name);
		    }
		}

	    gettimeofday(&tpStart, NULL);
		stop_time = tpStart.tv_sec * 1000000 + tpStart.tv_usec;
		if ((stop_time  - start_time) < CONTROL_PERIOD_US) {//save data every 20ms
			usleep(CONTROL_PERIOD_US-(stop_time  - start_time));
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
