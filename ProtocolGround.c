/*
 * ProtocolGround.c
 *
 *  Created on: Dec 11, 2015
 *      Author: root
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include "datatype.h"
#include "control.h"
#include "crc.h"
#include "fpga.h"
#include "serial.h"
#include "ComManage.h"
#include "ProtocolImu.h"
#include "ProtocolGround.h"
#include "interface.h"

extern uint32 debug_enable;

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
	int i=0;
	char cmd_exe_err=0;

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
	       (frame_type==CTRL_FRAME_TYPE_FLY_PARA3)   ||
	       (frame_type==CTRL_FRAME_TYPE_FLY_PARA4)   ||
	       (frame_type==CTRL_FRAME_TYPE_HELI_CONFIG) ||
	       (frame_type==CTRL_FRAME_TYPE_WAYPOINT_MODIFY)||
	       (frame_type==CTRL_FRAME_TYPE_WAYPOINT_INIT)    ||
	       (frame_type==CTRL_FRAME_TYPE_GROUND_OK)    ||
	       (frame_type==CTRL_FRAME_TYPE_MANUAL_MODE)  ||
	       (frame_type==CTRL_FRAME_TYPE_RESET)  ||
	       (frame_type==CTRL_FRAME_TYPE_FIRM_UPDATE)
	       ){
              //handle the control command
              frame_wait_confirm->type = frame_type;
              frame_wait_confirm->data_size = frame_info->frame_size;
              frame_wait_confirm->frame_num = (buf[CTRL_FRAME_MASK_FRAME_NUM+1]<<8) | buf[CTRL_FRAME_MASK_FRAME_NUM];
              frame_wait_confirm->frame_id  = (buf[CTRL_FRAME_MASK_FRAME_ID+1]<<8) | buf[CTRL_FRAME_MASK_FRAME_ID];
              //store the data field for process after confirming the command
			  memcpy(frame_wait_confirm->data,buf+CTRL_FRAME_MASK_DATA,frame_info->frame_size-CTRL_FRAME_LEN_NO_DATA);
			  //send out response command to the ground
			  control_cmd_response_recv(buf,frame_info->frame_size);
		}else if(frame_type==CTRL_FRAME_TYPE_STICK_DATA){
			// do not answer joystick data ,update it directly ,because it's periodical
			update_control_data(buf+CTRL_FRAME_MASK_DATA);
			print_debug("joystick frame %d\n",i++);
		}else if(frame_type==CTRL_FRAME_TYPE_LINK_TEST){
			// do not answer link test data ,update it directly ,because it's periodical
            link_test(buf);
            set_system_status(SYS_PREPARE_STEERING_TEST);
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
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING){

        	 	   	    data_export();
        	 	   	    printf("servo testing...\n");
        	 	        if((fp_servo_test=fopen(SERVO_TEST_DATA,"rb"))==NULL)
        	 	             printf("can not open servo test data\n");

        	 	   	    servo_test_enable=1;
        	 	   	    set_system_status(SYS_PREPARE_STEERING_TEST);
        	 	    }
        	 	    else {
        	 		    print_err("can not test rotar now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
		        	    break;
		        case CTRL_FRAME_TYPE_TAKEOFF:
		        	 	if(get_flying_status() == AIRCRAFT_READY)
		        	 		set_flying_status(AIRCRAFT_TAKEOFF);
		        	 	else {
		        	 		print_err("aircarft takeoff is not ready %d\n", get_flying_status());
	        	 		    cmd_exe_err = INVALID_CMD;
		        	 	}
		        	    break;
		        case CTRL_FRAME_TYPE_REMOTE_CTRL1:
	        	 	if(get_flying_status() & REMOTE1_VALID)
	        	 		set_flying_status(AIRCRAFT_REMOTE1);
	        	 	else {
	        	 		print_err("aircarft can not switch to remote1 now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
   		        	    break;
		        case CTRL_FRAME_TYPE_REMOTE_CTRL2:
	        	 	if(get_flying_status() & REMOTE2_VALID)
	        	 		set_flying_status(AIRCRAFT_REMOTE2);
	        	 	else {
                        print_err("aircarft can not switch to remote2 now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
		        	    break;
		        case CTRL_FRAME_TYPE_HOVER:
	        	 	if(get_flying_status() & HOVER_VALID)
	        	 		set_flying_status(AIRCRAFT_HOVERING);
	        	 	else {
                        print_err("aircarft can not switch to hover now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
		                break;
		        case CTRL_FRAME_TYPE_FLYING:
	        	 	if(get_flying_status() & FLYING_VALID)
	        	 		set_flying_status(AIRCRAFT_FLYING);
	        	 	else {
	        	 		print_err("aircarft can not switch to flying now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
		        	 break;
		        case CTRL_FRAME_TYPE_RETURN:
	        	 	if(get_flying_status() & RETURN_VALID)
	        	 		set_flying_status(AIRCRAFT_RETURN);
	        	 	else {
	        	 		print_err("aircarft can not switch to return now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
		        	    break;
		        case CTRL_FRAME_TYPE_LAND:
	        	 	if(get_flying_status() & LAND_VALID)
	        	 		set_flying_status(AIRCRAFT_LANDING);
	        	 	else {
                 		print_err("aircarft can not switch to land now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
                       break;
		        case CTRL_FRAME_TYPE_MANUAL_MODE:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING){
        	 		    set_flying_status(AIRCRAFT_MANUAL_MODE);
        	 		    //switch pwm output to RC input
        	 		    cmd_exe_err=set_control_register(CTRL_REG_MASK_MANUAL);

        	 	    }else {
       	 		         print_err("can not switch to manual mode now %d\n", get_flying_status());
             	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
        	 		         break;
		        case CTRL_FRAME_TYPE_RESET:
    	 		    set_flying_status(AIRCRAFT_PREPARING);
    	 		    //switch pwm output to aoto mode
    	 		    cmd_exe_err=reset_control_register(CTRL_REG_MASK_MANUAL);

		        	/*
        	 	    if(get_flying_status() & RESET_VALID){
        	 		    set_flying_status(AIRCRAFT_PREPARING);
        	 		    //switch pwm output to aoto mode
        	 		    cmd_exe_err=reset_control_register(CTRL_REG_MASK_MANUAL);

        	 	    }else {
       	 		         print_err("can not switch to reset mode now %d\n", get_flying_status());
             	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
        	 	    */
        	 		         break;
		        case CTRL_FRAME_TYPE_GROUND_OK:

        	 	    if(get_flying_status() == AIRCRAFT_PREPARING) {
        	 	    	if(waypoint_check()==TRUE){
        	 		        set_system_status(SYS_PREPARE_TAKEOFF);
		        		    set_flying_status(AIRCRAFT_READY);
        	 	    	}else
        	 	    		cmd_exe_err = NO_WAYPOINT;
        	 	    } else {
        	 		    print_err("can not switch to ground ok now %d\n", get_flying_status());
             	 		cmd_exe_err = INVALID_CMD;
        	 	    }
		                break;
		        case CTRL_FRAME_TYPE_EXPORT_DATA:
	        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
	        	 		    data_export();
	        	 	    else {
        	 		         print_err("can not export data now %d\n", get_flying_status());
             	 		    cmd_exe_err = INVALID_CMD;
	        	 	    }
		        	    break;
		        case CTRL_FRAME_TYPE_FLY_PARA1:
	        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
	        	 		     update_control_parameter_remote1(frame_wait_confirm->data);
	        	 	    else {
        	 		        print_err("can not set fly para now %d\n", get_flying_status());
            	 		    cmd_exe_err = INVALID_CMD;
	        	 	    }
		            	 break;
		        case CTRL_FRAME_TYPE_FLY_PARA2:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
        	 		     update_control_parameter_remote2(frame_wait_confirm->data);
        	 	    else {
       	 		        print_err("can not set fly para now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
		        	    break;
		        case CTRL_FRAME_TYPE_FLY_PARA3:
	        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
	        	 		     update_control_parameter_remote3(frame_wait_confirm->data);
	        	 	    else {
        	 		        print_err("can not set fly para now %d\n", get_flying_status());
            	 		    cmd_exe_err = INVALID_CMD;
	        	 	    }
		            	 break;
		        case CTRL_FRAME_TYPE_FLY_PARA4:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
        	 		     update_control_parameter_remote4(frame_wait_confirm->data);
        	 	    else {
       	 		        print_err("can not set fly para now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
		        	    break;
		        case CTRL_FRAME_TYPE_HELI_CONFIG:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
        	 		     set_aircaft_preparing_status(frame_wait_confirm->data);
        	 	    else {
        	 		        print_err("can not configure heli now %d\n", get_flying_status());
            	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
		        	    break;
		        case CTRL_FRAME_TYPE_WAYPOINT_MODIFY:
	        	 	if(get_flying_status() & WP_MODIFY_VALID)
	        	 		inflight_waypoint_modify(frame_wait_confirm);
	        	 	else {
	        	 		print_err("aircarft can not modify way point now %d\n", get_flying_status());
        	 		    cmd_exe_err = INVALID_CMD;
	        	 	}
		        	 break;
		        case CTRL_FRAME_TYPE_WAYPOINT_INIT:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
        	 		     waypoint_init(frame_wait_confirm);
        	 	    else {
        	 		        print_err("can not set waypoint now %d\n", get_flying_status());
            	 		    cmd_exe_err = INVALID_CMD;
        	 	    }
		        	 break;
		        case CTRL_FRAME_TYPE_FIRM_UPDATE:
        	 	    if(get_flying_status() == AIRCRAFT_PREPARING)
        	 		     firmware_upgrade(frame_wait_confirm->data,frame_wait_confirm->data_size);
        	 	    else {
        	 		     print_err("can not update firmware now %d\n", get_flying_status());
        	 		     cmd_exe_err = INVALID_CMD;
        	 	    }
		        	 break;
		        default:
		        	print_err("unsupport control cmd type received\n");
		        	cmd_exe_err = UNSUPPORTED_CMD;
		        	break;
	       }
	        //command execute over ,send response
	        frame_wait_confirm->type = 0;
	        if(cmd_exe_err == 0)
	        	control_cmd_response_exe(buf[CTRL_FRAME_MASK_DATA]);
	        else
	            fault_status_return(cmd_exe_err);
		}else
			/* the frame type waiting to be confirmed is not the same as received confirm cmd
			 * something is wrong with the confirm command
			 */
			//print_err("cmd type error\n");
		    fault_status_return(CMD_TYPE_MISMATCH);
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
            	     if(frame_crc==crc_checksum16(buf, frame_info->frame_size-3)){
            	     //if(1){
            		    // we have a valid CRC
            	         if(debug_enable)
            	    	     print_debug("valid ctrl frame received,frame type:%x \n",buf[CTRL_FRAME_MASK_FRAME_TYPE]);

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

