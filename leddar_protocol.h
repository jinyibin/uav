/*
 * leddar_protocol.h
 *
 *  Created on: Feb 21, 2017
 *      Author: user
 */

#ifndef LEDDAR_PROTOCOL_H_
#define LEDDAR_PROTOCOL_H_

#include "datatype.h"
#include "ComManage.h"

#define MAX_SIZE  200

#define LEDDAR_ADDRESS           0x01
#define LEDDAR_DETECTIONS_CMD    0x41
#define LEDDAR_DETECTIONS_NUM    2
#define LEDDAR_FRAME_LEN_NO_DATA 12
#define LEDDAR_FRAME_MIN_LEN     12
#define LEDDAR_FRAME_MAX_LEN     66

#define LEDDAR_LED_PWR           6



/**
 * \brief leddar数据的数据结构
 */
typedef struct leddar_detection
{
	unsigned char flag;          /* 标志位 */
	unsigned char seg;           /* 扇区号 */
	uint16  distance;      /* 距离 */
	unsigned int  amplitude;     /* 振幅 */

}leddar_detection;

int leddar_detection_request(void);
unsigned int leddar_detection_get(frame_info *frame_info, uint8 *buf);
unsigned int crc16_calc_leddar (unsigned char *CRC_Buf, unsigned char CRC_Leni);
leddar_detection *get_leddar_detection_data();

#endif /* LEDDAR_PROTOCOL_H_ */
