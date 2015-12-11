#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "adc.h"
#include "control.h"
#include "datatype.h"


static const char *adc0_temperature="/sys/bus/iio/devices/iio:device0/in_temp_input";
static const char *adc1_vin="/sys/bus/iio/devices/iio:device1/in_voltage8_raw";
static const char *adc0_v_monitor="/sys/bus/iio/devices/iio:device0/in_voltage9_raw";
static int fd_temp = -1;
static int fd_voltage_in = -1;

int adc_init()
{


   fd_temp = open(adc0_temperature,O_RDONLY);
   if(fd_temp<0){
	   print_err("can not open adc_temperature\n");
	   return ADC_TEMP_OPEN_FAILED;
   }
   fd_voltage_in = open(adc1_vin,O_RDONLY);

   if(fd_voltage_in<0){
	   print_err("can not open adc_vin\n");
	   return ADC_PS_OPEN_FAILED;
   }
   return fd_voltage_in;
}

void adc_close()
{
	if (fd_temp > 0)
		close(fd_temp);
	fd_temp = -1;

	if (fd_voltage_in > 0)
		close(fd_voltage_in);
	fd_voltage_in = -1;
}


int get_cpu_temperature()
{
    unsigned char buf[8];
     int raw_data;
    int temperature;
    int nread;
    int ret;

    fd_temp = open(adc0_temperature,O_RDONLY);
    if(fd_temp<0){
 	   print_err("can not open adc_temperature\n");
 	   return ADC_TEMP_OPEN_FAILED;
    }

    nread = read(fd_temp,buf,8);
    if(nread <= 0){
    	print_err("can not read temperature \n");
    	return -50; // temperature could be very low ,so can not return -1
    }
    raw_data=atoi(buf);// convert string to int

    /*
     * Calculate in degree Celsius times 1000
     * Using sensor slope of 1.84 mV/°C and
     * V at 25°C of 696 mV
     */
    //temperature = 25000-(raw_data*3300/4096-696)*1000000/1840;
    temperature = raw_data;

	if (fd_temp > 0)
		close(fd_temp);
	fd_temp = -1;

    return temperature;
}

int get_input_voltage()
{
    unsigned char buf[8];
    int raw_data;
    int voltage;
    int nread;

    fd_voltage_in = open(adc1_vin,O_RDONLY);

       if(fd_voltage_in<0){
    	   print_err("can not open adc_vin\n");
    	   return ADC_PS_OPEN_FAILED;
       }
    nread = read(fd_voltage_in,buf,8);
    if(nread <= 0){
    	print_err("can not read input voltage \n");
    	return -1;
    }
    raw_data=atoi(buf);// convert string to int
    voltage = raw_data>>4; // trim the 12bit ADC to 8bit

    //voltage = (raw_data *3300/4096)*11; // there is a resistor divider(/11) between the voltage monitored and adc
	if (fd_voltage_in > 0)
		close(fd_voltage_in);
	fd_voltage_in = -1;
    return voltage ;
}

int get_monitor_voltage()
{
    unsigned char buf[8];
    int raw_data;
    int voltage;
    int nread;

    fd_voltage_in = open(adc0_v_monitor,O_RDONLY);

       if(fd_voltage_in<0){
    	   print_err("can not open adc_v_monitor\n");
    	   return ADC_PS_OPEN_FAILED;
       }
    nread = read(fd_voltage_in,buf,8);
    if(nread <= 0){
    	print_err("can not read monitor voltage \n");
    	return -1;
    }
    raw_data=atoi(buf);// convert string to int
    voltage = raw_data>>4; // trim the 12bit ADC to 8bit

    //voltage = (raw_data *3300/4096)*11; // there is a resistor divider(/11) between the voltage monitored and adc
	if (fd_voltage_in > 0)
		close(fd_voltage_in);
	fd_voltage_in = -1;
    return voltage ;
}
