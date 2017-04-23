/*
 * imu.h
 *
 *  Created on: Dec 11, 2015
 *      Author: root
 */

#ifndef PROTOCOLIMU_H_
#define PROTOCOLIMU_H_

#define BUF_SIZE_GPS 512
#define GPS_FRAME_LEN 130

#define GPS_FRAME_START1 0xFF
#define GPS_FRAME_START2 0x02
#define GPS_FRAME_START3 0x90
#define GPS_FRAME_END 0x03
#define GPS_FRAME_MINIMUM_LEN 130
#define GPS_FRAME_MAX_LEN     200

#define GPS_FRAME_EULER_MASK          0
#define GPS_FRAME_GYRO_MASK           12
#define GPS_FRAME_ACCEL_MASK          24
#define GPS_FRAME_TIME_MASK           36
#define GPS_FRAME_STATUS_MASK         40
#define GPS_FRAME_NAVIGATION_MASK     44
#define GPS_FRAME_BARO_MASK           60
#define GPS_FRAME_POS_MASK            64
#define GPS_FRAME_VEL_MASK            88
#define GPS_FRAME_ATTI_ACCURACY_MASK  100
#define GPS_FRAME_POS_ACCURACY_MASK   104
#define GPS_FRAME_VEL_ACCURACY_MASK   108
#define GPS_FRAME_UTC_MASK            112

#define GPS_FRAME_LEN_NO_DATA 8       // total length of field other than data
#define GPS_FRAME_CRC_LEN    GPS_FRAME_LEN-5
#define GPS_FRAME_MASK_CMD   2        // cmd field position start from 0
#define GPS_FRAME_MASK_FRAME_SIZE 3   // frame size field position start from 0

void gps_data_parse(unsigned char *buf, frame_info *frame_info);
unsigned int serial_data_recv_gps(frame_info *frame_info,unsigned char *buf);
void set_flying_attitude(uint8 *buf);
#ifdef USE_SBG_IG500
int flying_attitude_sensor_is_active();
flying_attitude_s *get_flying_attitude();
#endif
#endif /* IMU_H_ */
