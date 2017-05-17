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

    uint32 counter_global=0;
int main( int argc,char *argv[])
{

	if(argc<2){
	           printf(" usage: uav [command] [gap] ] [debug] [test airline]\n");
	           printf(" [command]: 0--normal mode,used when run in the air\n");
	           printf(" [command]: 1--test mode,used for platform test.use this mode  \n");
	           printf("               when no IMU connected ,\n");
	           printf(" [command]: 2--manual mode,used when capturing PWM data\n");
	           printf(" [gap]: the frequency (ms/frame) in which UAV send fly status data\n");
	           printf(" [debug]: 1----printf debug information,default 0 \n");
	           printf(" [test airline]: 1----test airline,get waypoint every 10s,default 0 \n");
	        }
	command = atol(argv[1]);
	frequency = atol(argv[2]);
	debug_enable=atol(argv[3]);
	test_airline=atol(argv[4]);

	printf("----------------------------------------------------------------\n");
    printf("Firmware is running in ");
	switch(command){
	case 0:printf("normal mode:%d\n",command);break;
	case 1:printf("test mode:%d\n",command);break;
	case 2:printf("manual mode:%d\n",command);break;
	default:printf("error mode %d\n",command);break;
	}
	switch(debug_enable){
	case 0:printf("Debug information print disabled:%x\n",debug_enable);break;
	case 1:printf("Debug information print enabled:%x\n",debug_enable);break;
	default:printf("Debug information print set error %x\n",debug_enable);break;
	}

	if(test_airline==1)
		printf("Testing airline switch\n");
	printf("Flying data is downloaded every %d ms.\n",frequency);
    printf("Firmware build version:%s-%s\n",__DATE__,__TIME__);


	int ret = -1;
	uint32 counter=0;
	uint64 start_time, stop_time;
	while (ret < 0 ){
		ret = poweron_self_check();
	}

	while (get_flying_status() < AIRCRAFT_READY) {
		usleep(500000);
		flying_status_return(1);
	}

	//auto_flying_start();
	usleep(CONTROL_PERIOD_US/2);
	while (1) {


		start_time = get_current_time();

		read_rc_data(rc_data); // read remote control data

		autopilot_control();   // run auto pilot control algorithm

		stop_time = get_current_time();
		time_estimation.algorithm=stop_time  - start_time;

		if (time_estimation.algorithm < (CONTROL_PERIOD_US/2)){
			//auto pilot control algorithm takes so little time that we still have time to save data and download to the ground
            if(((counter*CONTROL_PERIOD_MS)%frequency)==0)
		       flying_status_return(1);// save data to flash and download to the ground
            else
               flying_status_return(0);// only save data to flash
            counter++;
            //save log file every 1 minute
		    if(counter==(CONTROL_FREQUENCY*60*1)){
			   counter=0;
			   fclose(fp_fly_status);
			   memset(log_file_name,sizeof(log_file_name),0);
			   generate_file_name(log_file_name);
		       if((fp_fly_status=fopen(log_file_name,"wb"))==NULL){
		         printf("can not open file:%s\n",log_file_name);
		       }
     		}

		    stop_time = get_current_time();
		    time_estimation.data_return=stop_time  - start_time;
		    if (time_estimation.data_return < CONTROL_PERIOD_US) {//save data every 20ms
			   usleep(CONTROL_PERIOD_US-time_estimation.data_return);
		    } else {
			   print_err("flying status return cost too much time\n");
		    }
		}else if(time_estimation.algorithm < CONTROL_PERIOD_US){
			// if auto pilot control algorithm takes too much time ,then do not save data and download to the ground
			usleep(CONTROL_PERIOD_US-time_estimation.algorithm);
			//printf("algorithm takes %d us\n",time_estimation.algorithm);
		}else
			// auto pilot control algorithm is running over time
			print_err("algorithm cost too much time\n");

	}
	sensor_close();
	adc_close();
	spi_close();
	exit(0);
	return 0;
}
