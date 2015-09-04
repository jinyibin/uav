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

int main( int argc,char *argv[])
{
	int ret = -1;
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

	    gettimeofday(&tpStart, NULL);
		stop_time = tpStart.tv_sec * 1000000 + tpStart.tv_usec;
		if ((stop_time  - start_time) < 20000) {
			usleep(20000-(stop_time  - start_time));
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
