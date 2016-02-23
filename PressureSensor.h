/*
 * PressureSensor.h
 *
 *  Created on: Feb 22, 2016
 *      Author: root
 */

#ifndef PRESSURESENSOR_H_
#define PRESSURESENSOR_H_

#define I2C_FILE_NAME "/dev/i2c-0"


/*---------------pressure sensor MPL3115A2----------------------*/
#define PRESSURE_SLAVE_ADDRESS 0x60
#define PRESSURE_STATUS_REG_ADDRESS 0x00
#define PRESSURE_DATA_MSB_ADDRESS 0x01
#define PRESSURE_DATA_CSB_ADDRESS 0x02
#define PRESSURE_DATA_LSB_ADDRESS 0x03
#define PRESSURE_CTRL_REG_ADDRESS 0x26


void pressure_sensor_init();
int get_altimeter();

/*--------------------------------------------------------------*/




#endif /* PRESSURESENSOR_H_ */
