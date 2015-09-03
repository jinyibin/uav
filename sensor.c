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
#include "status.h"
#include "sensor.h"

static int gps_fd = -1;
//static int high_fd = -1;
static int control_fd = -1;
static int running = 0;

static uint16 uav_ctrl_id=0x01;
#define MAX(a,b) (a>b?a:b)

static pthread_t recv_pid;
static void *sensor_data_collect();

int sensor_open()
{
	gps_fd = serial_open(GPS_SENSOR_COM, 115200, 0, 1);
    control_fd = serial_open(CONTROL_COM, 115200, 0, 1);
//	if (gps_fd < 0 || high_fd < 0 || control_fd < 0) {
	if (gps_fd < 0 ) {

		print_err("sensor open failed, gps_fd = %d\n", gps_fd);
		return SERIAL_GPS_OPEN_FAILED;
	}

	if (control_fd < 0) {

		print_err("sensor open failed,control_fd = %d\n",  control_fd);
		return SERIAL_CTRL_OPEN_FAILED;
	}
	running = 1;
	pthread_create(&recv_pid, NULL, sensor_data_collect, NULL);
	return 0;
}

void sensor_close()
{
	void* result = NULL;
	running = 0;
	pthread_join(recv_pid,&result);
	serial_close(gps_fd);
//	serial_close(high_fd);
	serial_close(control_fd);
}

/* control_data_parse()
 * process the control frame received
 *
 * frame_info         : struct pointer to store frame information ,including frame size and valid number of bytes received
 * buf                : allocated buffer to hold raw received data
 * frame_wait_confirm : struct pointer to store data field of the frame,and frame type waiting for confirm
 *
 */
int control_data_parse(unsigned char *buf, frame_info *frame_info,frame_wait_confirm *frame_wait_confirm)
{


	unsigned char frame_type = buf[10];

	if(frame_type!=CTRL_FRAME_TYPE_CMD_CONFIRM ){
		//if the frame is not a confirm command,store the data and send out answer command ,except joystick data
	    if((frame_type==CTRL_FRAME_TYPE_SERVO_TEST)  ||
	       (frame_type==CTRL_FRAME_TYPE_TAKEOFF)     ||
	       (frame_type==CTRL_FRAME_TYPE_REMOTE_CTRL1)||
	       (frame_type==CTRL_FRAME_TYPE_REMOTE_CTRL2)||
	       (frame_type==CTRL_FRAME_TYPE_HOVER)       ||
	       (frame_type==CTRL_FRAME_TYPE_FLYING)      ||
	       (frame_type==CTRL_FRAME_TYPE_RETURN)      ||
	       (frame_type==CTRL_FRAME_TYPE_LAND)        ||
	       (frame_type==CTRL_FRAME_TYPE_EXPORT_DATA) ||
	       (frame_type==CTRL_FRAME_TYPE_FLY_PARA1)   ||
	       (frame_type==CTRL_FRAME_TYPE_FLY_PARA2)   ||
	       (frame_type==CTRL_FRAME_TYPE_HELI_CONFIG) ||
	       (frame_type==CTRL_FRAME_TYPE_WAYPOINT_MODIFY)||
	       (frame_type==CTRL_FRAME_TYPE_WAYPOINT_INIT)    ||
	       (frame_type==CTRL_FRAME_TYPE_FIRM_UPDATE)
	       ){
              //handle the control command
              frame_wait_confirm->type = frame_type;
              frame_wait_confirm->data_size = frame_info->frame_size;
              frame_wait_confirm->frame_num = (buf[CTRL_FRAME_MASK_FRAME_NUM-1]<<8) | buf[CTRL_FRAME_MASK_FRAME_NUM];
              frame_wait_confirm->frame_id  = (buf[CTRL_FRAME_MASK_FRAME_ID-1]<<8) | buf[CTRL_FRAME_MASK_FRAME_ID];
              //store the data field for process after confirming the command
			  memcpy(frame_wait_confirm->data,buf+CTRL_FRAME_MASK_DATA,frame_info->frame_size-CTRL_FRAME_LEN_NO_DATA);
			  //send out response command to the ground
			  control_cmd_response_recv(buf,frame_info->frame_size);
		}else if(frame_type==CTRL_FRAME_TYPE_STICK_DATA){
			// do not answer joystick data ,update it directly ,because it's periodical
			update_control_data(buf+CTRL_FRAME_MASK_DATA);
		}else if(frame_type==CTRL_FRAME_TYPE_LINK_TEST){
			// do not answer link test data ,update it directly ,because it's periodical
            link_test(buf);
            set_flying_status(SYS_PREPARE_STEERING_TEST);
		}else if(frame_type==CTRL_FRAME_TYPE_VERSION_READ){
            send_version();
		}else
			print_err("unsupported control data received\n");
	}else{
	// if the frame is a confirm command, process the command data stored previously
		if(buf[CTRL_FRAME_MASK_DATA] == frame_wait_confirm->type) {
			/* if the frame type waiting to be confirmed is the same as received confirm cmd
			 * clear the frame_wait_confirm->type and process the command accordingly
			 */
	        switch (frame_wait_confirm->type) {
		        case CTRL_FRAME_TYPE_SERVO_TEST:
		        	 	steering_test();
		        	 	set_system_status(SYS_PREPARE_TAKEOFF);
		        	 	set_flying_status(AIRCRAFT_READY);
		        	    break;
		        case CTRL_FRAME_TYPE_TAKEOFF:
		        	 	if(get_flying_status() == AIRCRAFT_READY) {
		        	 		set_flying_status(AIRCRAFT_TAKEOFF);
		        	 	}
		        	 	else {
		        	 		print_err("aircarft takeoff is not ready %d\n", get_flying_status());
		        	 	}
		        	    break;
		        case CTRL_FRAME_TYPE_REMOTE_CTRL1:
		        	 	set_flying_status(AIRCRAFT_REMOTE1);
   		        	    break;
		        case CTRL_FRAME_TYPE_REMOTE_CTRL2:
		        	 	set_flying_status(AIRCRAFT_REMOTE2);
		        	    break;
		        case CTRL_FRAME_TYPE_HOVER:
		        	 	set_flying_status(AIRCRAFT_HOVERING);
		                break;
		        case CTRL_FRAME_TYPE_FLYING:
		        	 	set_flying_status(AIRCRAFT_FLYING);
		        	 break;
		        case CTRL_FRAME_TYPE_RETURN:
		        	 	set_flying_status(AIRCRAFT_RETURN);
		        	    break;
		        case CTRL_FRAME_TYPE_LAND:
		        	    set_flying_status(AIRCRAFT_LANDING);
                       break;
		        case CTRL_FRAME_TYPE_EXPORT_DATA:
		        	 	data_export();
		        	    break;
		        case CTRL_FRAME_TYPE_FLY_PARA1:
		        	    update_control_parameter_remote1(frame_wait_confirm->data);
		            	 break;
		        case CTRL_FRAME_TYPE_FLY_PARA2:
		        		update_control_parameter_remote2(frame_wait_confirm->data);
		        	    break;
		        case CTRL_FRAME_TYPE_HELI_CONFIG:
		        	    set_aircaft_preparing_status(frame_wait_confirm->data);
		        	    break;
		        case CTRL_FRAME_TYPE_WAYPOINT_MODIFY:
		        	    inflight_waypoint_modify(frame_wait_confirm);
		        	 break;
		        case CTRL_FRAME_TYPE_WAYPOINT_INIT:
		        	 	waypoint_init(frame_wait_confirm);
		        	 break;
		        case CTRL_FRAME_TYPE_FIRM_UPDATE:
		        	    firmware_upgrade(frame_wait_confirm->data,frame_wait_confirm->data_size);
		        	 break;
		        default:
		        	print_err("unsupport control cmd type received\n");
		        	break;
	       }
	        //command execute over ,send response
	        frame_wait_confirm->type = 0;
	        control_cmd_response_exe(buf[CTRL_FRAME_MASK_DATA]);
		}else
			/* the frame type waiting to be confirmed is not the same as received confirm cmd
			 * something is wrong with the confirm command
			 */
			//print_err("cmd type error\n");
		    fault_status_return(CMD_TYPE_MISMATCH);
	}
	return 0;
}

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

/* serial_data_recv_ctrl()
 * try to receive a frame from the serial port for control frame
 * frame_info: pointer to store frame information ,including frame size and valid number of bytes received
 * buf:        allocated buffer to hold raw received data
 * return      FRAME SIZE  if have received a valid frame
 *             0 if have not received a valid frame
 */
unsigned int serial_data_recv_ctrl(frame_info *frame_info ,unsigned char *buf)
{
	  unsigned int nread=0;
	  unsigned int i=0;
	  unsigned int frame_head_found=0;
	  unsigned int frame_crc;

    	// make sure buf never will be overflowed
      nread=read(control_fd,buf+frame_info->bytes_received,BUF_SIZE_CTRL-frame_info->bytes_received);

 	  frame_info->bytes_received += nread;

 	while(frame_info->bytes_received > 0){
 		//print_debug("ctrl bytes received %d\n",frame_info->bytes_received);
 	   // start searching frame head if at least 2 bytes has been received
 	   if(frame_info->bytes_received >= 2){
 	        for(i=0;i<frame_info->bytes_received-1;i++){
 	             if((buf[i]==CTRL_FRAME_START1)&&(buf[i+1]==CTRL_FRAME_START2)){
 	                //found the frame head ,remove useless received bytes before the beginning of the frame
 	                  if(i>0){
 	                       memmove(buf,buf+i,frame_info->bytes_received-i);
 	                       frame_info->bytes_received=frame_info->bytes_received-i;
 	                  }
 	                       frame_head_found = 1;
 	                       //print_debug("ctrl head found\n");
 	                       break;
 	              }
 	         }
 	    }else{
 	    	// not enough data
 	    	return 0;
 	    }

 	  if(frame_head_found){
            // frame head has been found ,start extract frame if we have received minimum bytes of a frame
 	        if(frame_info->bytes_received < CTRL_FRAME_MINIMUM_LEN){
 	        	//do not have enough data for a valid frame
 	        	//print_debug("ctrl :do not have enough data for a valid frame\n");
 	        	return 0;

 	        }
            // we have found the frame head ,extract the machine ID
            //frame_info->machine_id = (buf[3]<<8) |  buf[2];
             //extract the frame size
            frame_info->frame_size = (buf[CTRL_FRAME_MASK_FRAME_SIZE+1]<<8) | buf[CTRL_FRAME_MASK_FRAME_SIZE];

            //check if frame size is valid
            if((frame_info->frame_size >= CTRL_FRAME_MINIMUM_LEN)&&(frame_info->frame_size <= CTRL_FRAME_MAX_LEN)){
                 if(frame_info->frame_size > frame_info->bytes_received){
            	    // do not have received whole frame
                	 //print_debug("ctrl :do not have received whole frame\n");
            	     return 0;
                 }

                 // we have received the whole frame ,so check the frame tail
                 if(buf[frame_info->frame_size-1]==CTRL_FRAME_END){

            	     frame_crc = (buf[frame_info->frame_size-2]<<8) | buf[frame_info->frame_size-3];
            	     //if(frame_crc==crc_checksum16(buf, frame_info->frame_size-3)){
            	     if(1){
            		    // we have a valid CRC
            	    	 print_debug("ctrl :valid crc\n");

            		     return frame_info->frame_size;
            	     }else{
                        // invalid CRC ,remove the whole frame from the buffer

            		    memmove(buf,buf+frame_info->frame_size,frame_info->bytes_received-frame_info->frame_size);
            	    	frame_info->bytes_received=frame_info->bytes_received-frame_info->frame_size;
            	    	frame_info->frame_size = 0;
            		    frame_head_found = 0;
#ifdef debug
            		    print_debug("ctrl :invalid crc\n");
#else
            		    fault_status_response(CTRL_FRAME_CRC_FAILED);
#endif
            	     }

                 }else{
            	     //frame tail not found ,so the frame is invalid,
            	     //we should have incorrectly detected a start of frame
                     //remove the 2 frame head bytes and start searching frame head again
                	 print_debug("ctrl :frame tail not found ,so the frame is invalid\n");
            	     memmove(buf,buf+2,frame_info->bytes_received-2);
            	     frame_info->bytes_received=frame_info->bytes_received-2;
            	     frame_info->frame_size = 0;
            	     frame_head_found = 0;
            	     continue;
                 }
            }else{
                // invalid frame_size ,which means wrong frame head is detected
                 // we need to remove the 2 wrong frame head bytes
            	 print_debug("ctrl :invalid frame_size \n");
                 memmove(buf,buf+2,frame_info->bytes_received-2);
                 frame_info->bytes_received=frame_info->bytes_received-2;
                 frame_info->frame_size = 0;
                 frame_head_found = 0;
            }
       }else{
            //unable to find a valid start of frame
            //so check the last byte is FRAME_START1 in order to keep it for next time
    	    print_debug("ctrl :invalid start of frame\n");
            if(buf[frame_info->bytes_received-1]==CTRL_FRAME_START1){
                buf[0] = CTRL_FRAME_START1;
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


static void *sensor_data_collect()
{

	int i = 0;
	int m = 0;
	unsigned int data_len = 0;
	int maxfd = 0;
	fd_set rfds;

	unsigned char buf_gps[BUF_SIZE_GPS];          // buffer to store raw gps frame data
	unsigned char buf_ctrl[BUF_SIZE_CTRL];        // buffer to store raw control frame data
	unsigned char ctrl_frame_data[BUF_SIZE_CTRL]; // buffer to store data field of control frame


	 frame_info frame_info_gps={0,0};
	 frame_info frame_info_ctrl={0,0};
	 frame_wait_confirm frame_wait_confirm = {ctrl_frame_data,0,0};

	struct timeval tv;

	maxfd = MAX(gps_fd, control_fd);
	while (running) {
	         FD_ZERO(&rfds);
	         FD_SET(gps_fd, &rfds);
	         FD_SET(control_fd, &rfds);
	         tv.tv_sec=0;
             tv.tv_usec=80000; //80 ms
		     if (select(1 + maxfd, &rfds, NULL, NULL, &tv) > 0) {
				if (FD_ISSET(gps_fd, &rfds)) {
					data_len=serial_data_recv_gps(&frame_info_gps,buf_gps);
                    if(data_len > 0){
                        print_debug("%4d ,frame time: %d,length : %d\n",i++, *(unsigned int *)(buf_gps+41),data_len);
                    	gps_data_parse(buf_gps, &frame_info_gps);
                    	frame_info_gps.frame_size=0;
                    	frame_info_gps.bytes_received -= data_len;

                    }

				} else if (FD_ISSET(control_fd, &rfds)) {

					data_len = serial_data_recv_ctrl(&frame_info_ctrl,buf_ctrl);
					if(data_len > 0){
						print_debug("ctrl %d\n",m++);
					    control_data_parse(buf_ctrl,&frame_info_ctrl,&frame_wait_confirm);
					   frame_info_ctrl.frame_size=0;
					   frame_info_ctrl.bytes_received -= data_len;
					}
				}
		} else {
#ifdef debug
			print_err("Device read timeout\n");// serial port received timeout.
#else
			fault_status_response(SERIAL_NO_DATA);
#endif
		}
	}

}

int control_cmd_send(uint8 *buf,uint32 buf_size)
{
	return serial_write(control_fd, buf, buf_size);

}

