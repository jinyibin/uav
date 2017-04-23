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
#include "sbg_ellipse.h"

#ifdef USE_SBG_ELLIPSE
static flying_attitude_s flying_attitude;
static uint64 fa_timestamp = 0 ;


flying_attitude_s *get_flying_attitude()
{
	return &flying_attitude;
}

int flying_attitude_sensor_is_active()
{
	return (get_current_time() - fa_timestamp) <= CONTROL_PERIOD_US;
}

void ellipse_data_parse(unsigned char *buf, frame_info *frame_info)
{

	flying_attitude_s *p;
    uint8 msg_id ;
    uint8 msg_class ;
    uint8 msg_data_size;
    uint8 data[254];
    p=&flying_attitude;

    msg_id = buf[ELLIPSE_FRAME_MASK_MSG_ID];
    msg_class = buf[ELLIPSE_FRAME_MASK_MSG_CLASS];
    msg_data_size =((buf[ELLIPSE_FRAME_MASK_FRAME_SIZE+1]<<8) | buf[ELLIPSE_FRAME_MASK_FRAME_SIZE]) ;


    fa_timestamp = get_current_time();
    if(msg_class==ELLIPSE_FRAME_MSG_CLASS_LOG_ECOM0){
       switch(msg_id){
       case(ELLIPSE_FRAME_MSG_ID_STATUS):
           break;
       case(ELLIPSE_FRAME_MSG_ID_UTC_TIME):
		       memcpy(data,buf+ELLIPSE_FRAME_DATA_MASK,msg_data_size);
               p->year  = *((uint16*)(data+ELLIPSE_MSG_UTC_MASK_YEAR));
               p->month = data[ELLIPSE_MSG_UTC_MASK_MONTH];
               p->day   = data[ELLIPSE_MSG_UTC_MASK_DAY];
               p->hour  = data[ELLIPSE_MSG_UTC_MASK_HOUR];
               p->min   = data[ELLIPSE_MSG_UTC_MASK_MIN];
               p->sec   = data[ELLIPSE_MSG_UTC_MASK_SEC];

               break;
       case(ELLIPSE_FRAME_MSG_ID_IMU_DATA):

               p->ax  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_ACCEL_X));
               p->ay  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_ACCEL_Y));
               p->az  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_ACCEL_Z));
               p->gx  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_GYRO_X));
               p->gy  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_GYRO_Y));
               p->gz  = *((float*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_IMU_MASK_GYRO_Z));
               p->g_time  = *((uint32*)(buf+ELLIPSE_FRAME_DATA_MASK+ELLIPSE_MSG_MASK_TIME_STAMP));

	           gfstate.att_v[0]  = p->gx;
	           gfstate.att_v[1]  = p->gy;
	           gfstate.att_v[2]  = p->gz;
			   gfstate.acc_xyz[0]= p->ax;
			   gfstate.acc_xyz[1]= p->ay;
			   gfstate.acc_xyz[2]= p->az;
               break;
       case(ELLIPSE_FRAME_MSG_ID_EKF_EULER):
		       memcpy(data,buf+ELLIPSE_FRAME_DATA_MASK,msg_data_size);
		       p->roll  = *((float*)(data+ELLIPSE_MSG_EKF_EULER_MASK_ROLL));
		       p->pitch = *((float*)(data+ELLIPSE_MSG_EKF_EULER_MASK_PITCH));
		       p->yaw   = *((float*)(data+ELLIPSE_MSG_EKF_EULER_MASK_YAW));
			   gfstate.att[0]    = p->roll;
			   gfstate.att[1]    = p->pitch;
			   gfstate.att[2]    = p->yaw;
		       break;
       case(ELLIPSE_FRAME_MSG_ID_EKF_NAV):
		      memcpy(data,buf+ELLIPSE_FRAME_DATA_MASK,msg_data_size);
			  p->vn   = *((float*)(data+ELLIPSE_MSG_EKF_NAV_MASK_VEL_N));
			  p->ve   = *((float*)(data+ELLIPSE_MSG_EKF_NAV_MASK_VEL_E));
			  p->vd   = *((float*)(data+ELLIPSE_MSG_EKF_NAV_MASK_VEL_D));
			  p->lat  = *((double*)(data+ELLIPSE_MSG_EKF_NAV_MASK_LATI));
			  p->Long = *((double*)(data+ELLIPSE_MSG_EKF_NAV_MASK_LONG));
			  p->g_h  = *((double*)(data+ELLIPSE_MSG_EKF_NAV_MASK_ALTI));
              gfstate.v_neu[0]  = p->vn;
              gfstate.v_neu[1]  = p->ve;
              gfstate.v_neu[2]  = p->vd;
              gfstate.posi[0]   = p->lat;
              gfstate.posi[1]   = p->Long;
              gfstate.posi[2]   = p->g_h;
			   break;
       case(ELLIPSE_FRAME_MSG_ID_MAG):
    		   break;
       case(ELLIPSE_FRAME_MSG_ID_PRESSURE):
		       memcpy(data,buf+ELLIPSE_FRAME_DATA_MASK,msg_data_size);
			   p->b_h   = *((float*)(data+ELLIPSE_MSG_PRESSURE_MASK_ALTI));
               gfstate.h[0]      = p->b_h;
    		   break;
       default:
    	       break;
       }

    }
	if (get_system_status() == SYS_INIT)
		set_system_status(SYS_SENSOR_READY);

}

#endif

unsigned int serial_data_recv_ellipse(frame_info *frame_info,unsigned char *buf){

	  unsigned int nread=0;
	  unsigned int i=0;
	  unsigned int frame_head_found=0;
	  unsigned int frame_crc;


	// make sure buf never will be overflowed
  nread=read(gps_fd,buf+frame_info->bytes_received,BUF_SIZE_ELLIPSE-frame_info->bytes_received);
  frame_info->bytes_received += nread;

	while(frame_info->bytes_received > 0){
	   // start searching frame head if at least 2 bytes has been received
		//print_debug("gps bytes received %d\n",frame_info->bytes_received);
	   if(frame_info->bytes_received >= ELLIPSE_FRAME_MINIMUM_LEN){
	        for(i=0;i<frame_info->bytes_received-1;i++){
	             if((buf[i]==ELLIPSE_FRAME_START1)&&(buf[i+1]==ELLIPSE_FRAME_START2)){
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
	        if(frame_info->bytes_received < ELLIPSE_FRAME_MINIMUM_LEN){
	        	//do not have enough data to extract frame size
	        	// print_debug("gps do not have enough data to extract frame size\n");
	        	return 0;
	        }
         //extract the frame size
        frame_info->frame_size = ((buf[ELLIPSE_FRAME_MASK_FRAME_SIZE+1]<<8) | buf[ELLIPSE_FRAME_MASK_FRAME_SIZE]) + ELLIPSE_FRAME_LEN_NO_DATA;

        //check if frame size is valid
        if((frame_info->frame_size >= ELLIPSE_FRAME_MINIMUM_LEN)&&(frame_info->frame_size <= ELLIPSE_FRAME_MAX_LEN)){
             if(frame_info->frame_size > frame_info->bytes_received){
        	    // do not have received whole frame
          	     //print_debug("gps do not have received whole frame,frame size:%d\n",frame_info->frame_size);
        	     return 0;
             }

             // we have received the whole frame ,so check the frame tail
             if(buf[frame_info->frame_size-1]==ELLIPSE_FRAME_END){
                 //extract the CRC value of frame
        	       frame_crc = (buf[frame_info->frame_size-2]<<8) | buf[frame_info->frame_size-3];
        	       //SBG CRC is calculated on the[CMD;LENGTH;DATA] fields
        	       if(frame_crc==sbg_crc_check(buf+ELLIPSE_FRAME_MASK_MSG_ID, frame_info->frame_size-5)){
        		      // we have a valid CRC
        	    	  //print_debug("gps crc ok\n");
        		      return frame_info->frame_size;
        	       }else{
                    // invalid CRC ,remove the whole frame from the buffer
#ifdef debug
        		      print_debug("ellipse crc error,%d\n",frame_info->frame_size);
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
          	       print_debug("sbg frame tail not found \n");
        	       memmove(buf,buf+2,frame_info->bytes_received-2);
        	       frame_info->bytes_received=frame_info->bytes_received-2;
        	       frame_info->frame_size = 0;
        	       frame_head_found = 0;
             }
        }else{
             // invalid frame_size ,which means wrong frame head is detected
             // we need to remove the 3 wrong frame head bytes
      	     print_debug("sbg invalid frame_size\n");
             memmove(buf,buf+2,frame_info->bytes_received-2);
             frame_info->bytes_received=frame_info->bytes_received-2;
             frame_info->frame_size = 0;
             frame_head_found = 0;
        }
   }else{
        //unable to find a valid start of frame
        //so check if the last  bytes are FRAME_START1  in order to keep it for next time
  	    print_debug("sbg invalid frame_start\n");
        if(buf[frame_info->bytes_received-1]==ELLIPSE_FRAME_START1){
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
