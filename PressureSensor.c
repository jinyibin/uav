/*
 * PressureSensor.c

 *
 *  Created on: Feb 22, 2016
 *      Author: root
 */
#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include "PressureSensor.h"



static int 	pressure_i2c_file;

static int set_i2c_register(int file,unsigned char addr,unsigned char reg,unsigned char value) {

    unsigned char outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    /* The first byte indicates which register we'll write */
    outbuf[0] = reg;

    /*
     * The second byte indicates the value to write.  Note that for many
     * devices, we can write multiple, sequential registers at once by
     * simply making outbuf bigger.
     */
    outbuf[1] = value;

    /* Transfer the i2c packets to the kernel and verify it worked */
    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }

    return 0;
}


static int get_i2c_register(int file,unsigned char addr,unsigned char reg,unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }
    *val = inbuf;

    return 0;
}

void pressure_sensor_init(){

	    // Open a connection to the I2C userspace control file.
	  if ((pressure_i2c_file = open(I2C_FILE_NAME, O_RDWR)) < 0) {
	        perror("Unable to open i2c control file");
	        exit(1);
	    }
	  // Set sensor to Altimeter output
	  set_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x26,0xB8) ;
	  // Enable data flags in PT_DATA_CFG
	  set_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x13,0x07) ;
	  // Set active
	  set_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x26,0xB9) ;



}
int get_altimeter(){
	unsigned char status;
	unsigned char altimeter_msb;
	unsigned char altimeter_csb;
	unsigned char altimeter_lsb;
	 // Read status register
    get_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x00,&status) ;
    //if((status&0x08)==0x08){//sensor data ready
    	get_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x01,&altimeter_msb) ;
    	get_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x02,&altimeter_csb) ;
    	get_i2c_register(pressure_i2c_file,PRESSURE_SLAVE_ADDRESS,0x03,&altimeter_lsb) ;
    //}else{
    //	printf("pressure sensor data is not ready:%x\n",status);
    //	return 0;
    //}
    	//printf("%x,%x,%x,%x\n",altimeter_msb,altimeter_csb,altimeter_lsb,status);
    return (altimeter_msb<<24)|((altimeter_csb<<16)|altimeter_lsb);
}

