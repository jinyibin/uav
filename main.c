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

int main( int argc,char *argv[])
{
	int ret = -1;
	while (ret < 0 ){
		ret = poweron_self_check();
	}
	while (get_flying_status() < AIRCRAFT_READY) {
		usleep(500000);
		flying_status_return();
	}
	while (get_flying_status() < AIRCRAFT_TAKEOFF) {
		flying_status_return();
		usleep(40000);
	}
	auto_flying_start();
	while (1) {
		usleep(20000);
		flying_status_return();
	}
	sensor_close();
	adc_close();
	spi_close();
	exit(0);
	return 0;
}
