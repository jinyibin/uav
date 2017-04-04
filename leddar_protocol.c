/*
 * leddar_protocol.c
 *
 *  Created on: Feb 21, 2017
 *      Author: user
 */
#include <stddef.h>
#include <pthread.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "leddar_protocol.h"
#include "ComManage.h"

static leddar_detection leddar_detect[8];

/**
 * \brief send leddar detection data request
 *
 */
int leddar_detection_request(void)
{
	uint8 buf[4];
    buf[0]=0x01;
    buf[1]=0x41;
    buf[2]=0xc0;
    buf[3]=0x10;
    return serial_write(leddar_fd, buf, 4);
}
/**
 * \brief 雷达数据解析函数
 *
 *	\parma[out] leddar_data 雷达数据的结构体
 *	\parma[in]  frame_info  数据帧长度
 *	\parma[in]  rev_buf     串口数据缓存器
 */
void leddar_parse_detection (leddar_detection *leddar_data, uint8 data_length,uint8 *rev_buf)
{
	int i = 0;

	uint8 segment_id=0;

	/*
	for(i=0;i < 8;i++){
       leddar_data[i].amplitude = 0;
       leddar_data[i].distance  = 0;
       leddar_data[i].flag      = 0;
       leddar_data[i].seg       = 0;
	}
    */
	for (i = 3; i < data_length - LEDDAR_FRAME_LEN_NO_DATA; i += 6) {
		segment_id=rev_buf[i + 5];
		leddar_data[segment_id].distance  = (unsigned int)(rev_buf[i] |
				                                 (rev_buf[i + 1] << 8));
		leddar_data[segment_id].amplitude = (unsigned int)(rev_buf[i + 2] |
												 (rev_buf[i + 3] << 8));
		//printf("distance : %d, amp:%d \n",leddar_data[segment_id].distance,leddar_data[segment_id].amplitude);
		leddar_data[segment_id].flag      = rev_buf[i + 4];
	}
}

/**
 * \brief read leddar detection data
 * \detail use this function only if whole data packet is received already
 */
unsigned int leddar_detection_get (frame_info *frame_info, unsigned char *buf)
{
	unsigned int i = 0;
	unsigned int frame_head_found = 0;
	unsigned int frame_crc        = 0;


	int len = 0;

	//printf("len %d ,rece %d\n",len,frame_info->bytes_received);
	len = read(leddar_fd, buf  + frame_info->bytes_received, 254 - frame_info->bytes_received);
	if(len<0)
		return 0;
    frame_info->bytes_received += len;
	//printf("len %d ,rece %d\n",len,frame_info->bytes_received);

	while (frame_info->bytes_received > LEDDAR_FRAME_MIN_LEN) {

		/* 接收到大于2的数据的话，就开始检测帧头 */
		if (frame_info->bytes_received >= 2) {

			//printf("searching head of frame\n");

			for (i = 0; i < frame_info->bytes_received -1; i++) {
				if ((buf[i]      == LEDDAR_ADDRESS) &&(buf[i + 1]) == LEDDAR_DETECTIONS_CMD) {
					if (i > 0) {
						memmove(buf, buf + i, frame_info->bytes_received - i);
						frame_info->bytes_received=frame_info->bytes_received-i;
					}
					/* 表示已经接收到帧头 */
					frame_head_found = 1;
					//printf("get head of frame\n");
					break;
				}
			}

		} else {  /* 检测不到帧头，就继续接收 */
			//printf("no head of leddar frame\n");
			return 0;
		}

		/* 接收到帧头的话，就开始判断数据的有效性 */
		if (frame_head_found) {

				frame_crc = (buf[frame_info->bytes_received- 1] << 8) | buf[frame_info->bytes_received - 2];

				/* crc校验 */
				if (frame_crc == crc16_calc_leddar(buf, frame_info->bytes_received - 2)) {
					//printf("leddar crc code is right..^^..\n");
					leddar_parse_detection(leddar_detect,frame_info->bytes_received,buf);

					return 1;
				} else {
					//printf("leddar crc code is wrong\n");
					frame_head_found = 0;
				        memmove(buf, buf + 2, frame_info->bytes_received - 2);
				        frame_info->bytes_received = frame_info->bytes_received - 2;

				}
		} else { /* 接收到错误的头部，将头部删去， */
				//printf("get a wrong leddar frame head\n");
				memmove(buf, buf + 2, frame_info->bytes_received - 2);
				frame_info->bytes_received = frame_info->bytes_received - 2;
				frame_head_found = 0;

		}
	}

	return 0;
}

/**
 * \brief 计算crc校验
 *
 * \parma[in] CRC_Buf  要计算crc的数组
 * \parma[in] CRC_Leni 数组的长度
 *
 * \retval 计算出来的crc校验码（小端）
 */
unsigned int crc16_calc_leddar (unsigned char *crc_buf, unsigned char crc_leni)
{
	unsigned char i, j;
	unsigned int crc_sumx;
	crc_sumx = 0xFFFF;

	for (i = 0; i < crc_leni; i++) {
		crc_sumx ^= *(crc_buf + i);
		for (j = 0; j < 8; j++) {
			if (crc_sumx & 0x01) {
				crc_sumx >>= 1;
				crc_sumx ^= 0xA001;
			} else {
				crc_sumx >>= 1;
			}
		}
	}
	return (crc_sumx);
}

leddar_detection *get_leddar_detection_data(){

	return &leddar_detect;
}
