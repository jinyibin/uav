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
#include "interface.h"

static flying_attitude_s flying_attitude;
static uint64 fa_timestamp = 0;

void gps_data_parse(unsigned char *buf, frame_info *frame_info)
{

	set_flying_attitude(buf);
	if (get_system_status() == SYS_INIT)
		set_system_status(SYS_SENSOR_READY);

}

flying_attitude_s *get_flying_attitude()
{
	return &flying_attitude;
}

int flying_attitude_sensor_is_active()
{
	return (get_current_time() - fa_timestamp) <= CONTROL_PERIOD_US;
}

void set_flying_attitude(uint8 *buf)
{
	flying_attitude_s *p;
	uint8 data[GPS_FRAME_LEN-8];
	p=&flying_attitude;

	memcpy(data,buf+5,GPS_FRAME_LEN-8);//4bytes alignment of the data

	p->roll=*((float *)(data+GPS_FRAME_EULER_MASK));
	p->pitch=*((float *)(data+4+GPS_FRAME_EULER_MASK));
	p->yaw=*((float *)(data+8+GPS_FRAME_EULER_MASK));
	p->gx=*((float *)(data+GPS_FRAME_GYRO_MASK));
	p->gy=*((float *)(data+4+GPS_FRAME_GYRO_MASK));
	p->gz=*((float *)(data+8+GPS_FRAME_GYRO_MASK));
	p->ax=*((float *)(data+GPS_FRAME_ACCEL_MASK));
	p->ay=*((float *)(data+4+GPS_FRAME_ACCEL_MASK));
	p->az=*((float *)(data+8+GPS_FRAME_ACCEL_MASK));
	p->g_time=*((unsigned int *)(data+GPS_FRAME_TIME_MASK));
	p->status=*((unsigned int *)(data+GPS_FRAME_STATUS_MASK));
	p->vn=*((int *)(data+GPS_FRAME_NAVIGATION_MASK));
	p->ve=*((int *)(data+4+GPS_FRAME_NAVIGATION_MASK));
	p->vd=*((int *)(data+8+GPS_FRAME_NAVIGATION_MASK));
	p->heading=*(( int *)(data+12+GPS_FRAME_NAVIGATION_MASK));
	p->b_h=*(( int *)(data+GPS_FRAME_BARO_MASK));
	p->lat=*((double *)(data+GPS_FRAME_POS_MASK));
	p->Long=*((double *)(data+8+GPS_FRAME_POS_MASK));
	p->g_h=*((double *)(data+16+GPS_FRAME_POS_MASK));
	p->vx=*((float *)(data+GPS_FRAME_VEL_MASK));
	p->vy=*((float *)(data+4+GPS_FRAME_VEL_MASK));
	p->vz=*((float *)(data+8+GPS_FRAME_VEL_MASK));
	p->accuracy_atti=*((float *)(data+GPS_FRAME_ATTI_ACCURACY_MASK));
	p->accuracy_pos=*((float *)(data+GPS_FRAME_POS_ACCURACY_MASK));
	p->accuracy_vel=*((float *)(data+GPS_FRAME_VEL_ACCURACY_MASK));
	p->year  = data[GPS_FRAME_UTC_MASK];
	p->month  = data[GPS_FRAME_UTC_MASK+1];
	p->day  = data[GPS_FRAME_UTC_MASK+2];
	p->hour  = data[GPS_FRAME_UTC_MASK+3];
	p->min  = data[GPS_FRAME_UTC_MASK+4];
	p->sec  = data[GPS_FRAME_UTC_MASK+5];
    gfstate.att[0]    = p->roll;
    gfstate.att[1]    = p->pitch;
    gfstate.att[2]    = p->yaw;
    gfstate.att_v[0]  = p->gx;
    gfstate.att_v[1]  = p->gy;
    gfstate.att_v[2]  = p->gz;
    gfstate.acc_xyz[0]= p->ax;
    gfstate.acc_xyz[1]= p->ay;
    gfstate.acc_xyz[2]= p->az;
    gfstate.v_neu[0]  = p->vn;
    gfstate.v_neu[1]  = p->ve;
    gfstate.v_neu[2]  = p->vd;
    gfstate.v_xyz[0]  = p->vx;
    gfstate.v_xyz[1]  = p->vy;
    gfstate.v_xyz[2]  = p->vz;
    gfstate.posi[0]   = p->lat;
    gfstate.posi[1]   = p->Long;
    gfstate.posi[2]   = p->g_h;
    gfstate.h[0]      = p->b_h;
    gfstate.h[1]      = p->heading;
	fa_timestamp = get_current_time();
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
	  unsigned int m=0;

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
#ifdef debug
          		      print_debug("gps crc error,%d\n",frame_info->frame_size);
          		      //for(m=0;m<frame_info->frame_size;){
          		      //  print_debug(" %2x %2x %2x %2x %2x %2x %2x %2x  \n",buf[m],buf[m+1],buf[m+2],buf[m+3],buf[m+4],buf[m+5],buf[m+6],buf[m+7]);
                      //  m=m+8;
          		     // }
#else
          		      fault_status_response(GPS_FRAME_CRC_FAILED);
#endif
          		      memmove(buf,buf+frame_info->frame_size,frame_info->bytes_received-frame_info->frame_size);
          	    	  frame_info->bytes_received -=frame_info->frame_size;
          	    	  frame_info->frame_size = 0;
          		      frame_head_found = 0;

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
