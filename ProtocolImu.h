/*
 * imu.h
 *
 *  Created on: Dec 11, 2015
 *      Author: root
 */

#ifndef PROTOCOLIMU_H_
#define PROTOCOLIMU_H_

#define BUF_SIZE_GPS 512
#define GPS_FRAME_LEN 104

#define GPS_FRAME_START1 0xFF
#define GPS_FRAME_START2 0x02
#define GPS_FRAME_START3 0x90
#define GPS_FRAME_END 0x03
#define GPS_FRAME_MINIMUM_LEN 104
#define GPS_FRAME_MAX_LEN     200

#define GPS_FRAME_LEN_NO_DATA 8       // total length of field other than data
#define GPS_FRAME_CRC_LEN    GPS_FRAME_LEN-5
#define GPS_FRAME_MASK_CMD   2        // cmd field position start from 0
#define GPS_FRAME_MASK_FRAME_SIZE 3   // frame size field position start from 0

void gps_data_parse(unsigned char *buf, frame_info *frame_info);
unsigned int serial_data_recv_gps(frame_info *frame_info,unsigned char *buf);

#endif /* IMU_H_ */
