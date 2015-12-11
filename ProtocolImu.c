#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include "datatype.h"
#include "control.h"
#include "crc.h"
#include "serial.h"
#include "ComManage.h"
#include "ProtocolImu.h"
#include "fpga.h"



void gps_data_parse(unsigned char *buf, frame_info *frame_info)
{

	set_flying_attitude(buf);
	if (get_system_status() == SYS_INIT)
		set_system_status(SYS_SENSOR_READY);

}

/* serial_data_recv_gps()
 * try to receive a frame from the serial port for gps
 * frame format : FF 02 cmd(90) length(2bytes) data crc(2bytes) 03
 * frame_info: pointer to store frame information ,including frame size and valid number of bytes received
 * buf:    allocated buffer to hold raw received data
 * return  FRAME SIZE  if have received a valid frame
 *         0 if have not received a valid frame
 */
unsigned int serial_data_recv_gps(frame_info *frame_info,unsigned char *buf)
{

	  unsigned int nread=0;
	  unsigned int i=0;
	  unsigned int frame_head_found=0;
	  unsigned int frame_crc;

  	// make sure buf never will be overflowed
    nread=read(gps_fd,buf+frame_info->bytes_received,BUF_SIZE_GPS-frame_info->bytes_received);
    frame_info->bytes_received += nread;

	while(frame_info->bytes_received > 0){
	   // start searching frame head if at least 2 bytes has been received
		//print_debug("gps bytes received %d\n",frame_info->bytes_received);
	   if(frame_info->bytes_received >= 3){
	        for(i=0;i<frame_info->bytes_received-2;i++){
	             if((buf[i]==GPS_FRAME_START1)&&(buf[i+1]==GPS_FRAME_START2)&&(buf[i+2]==GPS_FRAME_START3)){
	                //found the frame head ,remove useless received bytes before the beginning of the frame
	                  if(i>0){
	                       memmove(buf,buf+i,frame_info->bytes_received-i);
	                       frame_info->bytes_received=frame_info->bytes_received-i;
	                  }
	                       frame_head_found = 1;
	                       //print_debug("gps head found\n");
	                       break;
	              }
	         }
	    }else{
	    	// not enough data
	    	return 0;
	    }

	  if(frame_head_found){
          // frame head has been found ,check if we have received the frame size field
	        if(frame_info->bytes_received < 5){
	        	//do not have enough data to extract frame size
	        	// print_debug("gps do not have enough data to extract frame size\n");
	        	return 0;
	        }
           //extract the frame size
          frame_info->frame_size = ((buf[GPS_FRAME_MASK_FRAME_SIZE]<<8) | buf[GPS_FRAME_MASK_FRAME_SIZE+1]) + GPS_FRAME_LEN_NO_DATA;

          //check if frame size is valid
          if((frame_info->frame_size >= GPS_FRAME_MINIMUM_LEN)&&(frame_info->frame_size <= GPS_FRAME_MAX_LEN)){
               if(frame_info->frame_size > frame_info->bytes_received){
          	    // do not have received whole frame
            	 //  print_debug("gps do not have received whole frame\n");
          	     return 0;
               }

               // we have received the whole frame ,so check the frame tail
               if(buf[frame_info->frame_size-1]==GPS_FRAME_END){
                   //extract the CRC value of frame
          	       frame_crc = (buf[frame_info->frame_size-3]<<8) | buf[frame_info->frame_size-2];
          	       //SBG CRC is calculated on the[CMD;LENGTH;DATA] fields
          	       if(frame_crc==sbg_crc_check(buf+GPS_FRAME_MASK_CMD, GPS_FRAME_CRC_LEN)){
          		      // we have a valid CRC
          	    	  //print_debug("gps crc ok\n");
          		      return frame_info->frame_size;
          	       }else{
                      // invalid CRC ,remove the whole frame from the buffer

          		      memmove(buf,buf+frame_info->frame_size,frame_info->bytes_received-frame_info->frame_size);
          	    	  frame_info->bytes_received -=frame_info->frame_size;
          	    	  frame_info->frame_size = 0;
          		      frame_head_found = 0;
#ifdef debug
          		      print_debug("gps crc error\n");
#else
          		      fault_status_response(GPS_FRAME_CRC_FAILED);
#endif
          	       }

               }else{
          	       //frame tail not found ,so the frame is invalid,
          	       //we should have incorrectly detected a start of frame
                   //remove the 3 frame head bytes and start searching frame head again
            	   print_debug("gps frame tail not found \n");
          	       memmove(buf,buf+3,frame_info->bytes_received-3);
          	       frame_info->bytes_received=frame_info->bytes_received-3;
          	       frame_info->frame_size = 0;
          	       frame_head_found = 0;
               }
          }else{
               // invalid frame_size ,which means wrong frame head is detected
               // we need to remove the 3 wrong frame head bytes
        	   print_debug("gps invalid frame_size\n");
               memmove(buf,buf+3,frame_info->bytes_received-3);
               frame_info->bytes_received=frame_info->bytes_received-3;
               frame_info->frame_size = 0;
               frame_head_found = 0;
          }
     }else{
          //unable to find a valid start of frame
          //so check the last 2 bytes are FRAME_START1 and FRAME_START2 in order to keep it for next time
    	  print_debug("gps invalid frame_start\n");
          if((buf[frame_info->bytes_received-2]==GPS_FRAME_START1)&&(buf[frame_info->bytes_received-1]==GPS_FRAME_START2)){
              buf[0] = GPS_FRAME_START1;
              buf[1] = GPS_FRAME_START2;
              frame_info->bytes_received = 2;
          }else if(buf[frame_info->bytes_received-1]==GPS_FRAME_START1){
        	  //check if the last byte is FRAME START1
              buf[0] = GPS_FRAME_START1;
              frame_info->bytes_received = 1;
          }else{
              //discard the whole buffer
              frame_info->bytes_received = 0;
          }
          return 0;
      }

	}
	return 0;
}
