#include "sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "status.h"
#include "crc.h"
#include "interface.h"
#include "fpga.h"
#include "adc.h"
#include "control.h"



static int system_status = 0;
static int waypoint_is_ready = 0;
static int prepare_setting_is_ready = 0;
aircraft_preparing_status_s  aircraft_preparing_status;
waypoint_info_s waypoint_info = {0,0};


static waypoint_list_s *waypoint_list_head = NULL;
static waypoint_list_s *waypoint_list_tail = NULL;
static waypoint_list_s *waypoint_list_current = NULL;

void set_system_status(int status)
{
	system_status = status;
}
int get_system_status()
{
	return system_status;
}

static int waypoint_list_add(uint8 *waypoint)
{
	waypoint_list_s *wp_list = NULL;
	waypoint_list_s *wp_list_tail = waypoint_list_tail;

	wp_list = malloc(sizeof(waypoint_list_s));
	if (wp_list == NULL) {
		print_err("malloc failed\n");
		return -1;
	}
	//memcpy(&(wp_list->waypoint), waypoint, sizeof(waypoint_s));
	wp_list->waypoint.id = *(uint16*)(waypoint);
	wp_list->waypoint.task = *(waypoint+2);
	wp_list->waypoint.task_para = *(waypoint+3);
	wp_list->waypoint.v = *(float*)(waypoint+4);
	wp_list->waypoint.lon = *(double*)(waypoint+8);
	wp_list->waypoint.lat = *(double*)(waypoint+16);
	wp_list->waypoint.h = *(float*)(waypoint+24);


	if (waypoint_list_head == NULL) {
		waypoint_list_head = waypoint_list_tail = wp_list;
		wp_list->prev = NULL;
		wp_list->next = NULL;
	} else {
		wp_list_tail->next = wp_list;
		wp_list->prev = waypoint_list_tail;
		wp_list->next = NULL;
		waypoint_list_tail = wp_list;
	}
	waypoint_info.received_num++;

	return 0;
}

static void waypoint_list_clear()
{
	waypoint_list_s *wp = waypoint_list_head;
	waypoint_list_s *p = wp;
	while(wp) {
		p = wp;
		wp = wp->next;
		free(p);
	}
	waypoint_list_head = waypoint_list_current = waypoint_list_tail = NULL;
}

int waypoint_delete (uint8 *waypoint, int no)
{
	int i = 0;
	waypoint_list_s *wp = waypoint_list_head;
	for(i= 0; i < no; i++) {
		if (wp == NULL){
			print_err("waypoint list node is NULL, delete failed\n");
			return -1;
		}
		wp = wp->next;
	}
	if (wp->prev)
		wp->prev->next = wp->next;
	if (wp->next)
		wp->next->prev = wp->prev;
	free(wp);
	wp = waypoint_list_head;

	waypoint_info.received_num --;
	waypoint_info.total_num --;
	//reassign id no
	for(i=0;i<waypoint_info.total_num;i++){
         wp->waypoint.id = i;
         wp = wp->next;
	}

	return 0;
}
int waypoint_insert (uint8 *waypoint, int no)
{
	int i = 0;
	waypoint_list_s *wp = waypoint_list_head;
	waypoint_list_s *prev = NULL;
	waypoint_list_s *curr = malloc(sizeof(waypoint_list_s));
	//memcpy(&curr->waypoint, waypoint, sizeof(waypoint_s));;
	curr->waypoint.id = *(uint16*)(waypoint);
	curr->waypoint.task = *(waypoint+2);
	curr->waypoint.task_para = *(waypoint+3);
	curr->waypoint.v = *(float*)(waypoint+4);
	curr->waypoint.lon = *(double*)(waypoint+8);
	curr->waypoint.lat = *(double*)(waypoint+16);
	curr->waypoint.h = *(float*)(waypoint+24);
	for(i= 0; i < no; i++) {
		if (wp == NULL){
			print_err("waypoint list node is NULL, insert failed\n");
			return -1;
		}
		wp = wp->next;
	}
	prev = wp->prev;
	curr->next = wp;
	wp->prev = curr;
	curr->prev = prev;
	prev->next = curr;


	wp = waypoint_list_head;
	waypoint_info.received_num ++;
	waypoint_info.total_num ++;
	//reassign id no
	for(i=0;i<waypoint_info.total_num;i++){
         wp->waypoint.id = i;
         wp = wp->next;
	}
	return 0;
}

int waypoint_modify (uint8 *waypoint, int no)
{
	int i = 0;
	waypoint_list_s *wp = waypoint_list_head;
	for(i= 0; i < no; i++) {
		if (wp == NULL){
			print_err("waypoint list node is NULL, modify failed\n");
			return -1;
		}
		wp = wp->next;
	}
	//memcpy(&wp->waypoint, waypoint, sizeof(waypoint_s));
	wp->waypoint.id = *(uint16*)(waypoint);
	wp->waypoint.task = *(waypoint+2);
	wp->waypoint.task_para = *(waypoint+3);
	wp->waypoint.v = *(float*)(waypoint+4);
	wp->waypoint.lon = *(double*)(waypoint+8);
	wp->waypoint.lat = *(double*)(waypoint+16);
	wp->waypoint.h = *(float*)(waypoint+24);
	return 0;
}
waypoint_list_s *get_waypoint_head()
{
	return waypoint_list_head;
}

waypoint_list_s *get_waypoint_tail()
{
	return waypoint_list_tail;
}


waypoint_list_s *get_waypoint_previous()
{
	waypoint_list_current = waypoint_list_current->prev;
	return waypoint_list_current;
}
waypoint_list_s *get_waypoint_next()
{
	waypoint_list_current = waypoint_list_current->next;
	return waypoint_list_current;
}
waypoint_list_s *get_waypoint_current()
{
	return waypoint_list_current;
}

int inflight_waypoint_modify(frame_wait_confirm *frame_wait_confirm)
{
    uint16 waypoint_id;
    int ret = -1;
    waypoint_id = frame_wait_confirm->data[CTRL_FRAME_MASK_WP_ID+1] << 8| frame_wait_confirm->data[CTRL_FRAME_MASK_WP_ID];
 	if (frame_wait_confirm->data[0] == WAYPOINT_INSERT)
 		ret=waypoint_insert(frame_wait_confirm->data + CTRL_FRAME_MASK_WP_ID, waypoint_id);
 	else if (frame_wait_confirm->data[0] == WAYPOINT_MODIFY)
 		ret=waypoint_modify(frame_wait_confirm->data + CTRL_FRAME_MASK_WP_ID, waypoint_id);
 	else if (frame_wait_confirm->data[0] == WAYPOINT_DELETE)
 		ret=waypoint_delete(frame_wait_confirm->data + CTRL_FRAME_MASK_WP_ID, waypoint_id);

 	return ret;
}


void waypoint_init(frame_wait_confirm *frame_wait_confirm)
{
	int i;
	int waypoint_num_this_frame ;
	int waypoint_total_num;
	uint8 *waypoint;
    // extract way point number of this frame
	waypoint_num_this_frame= frame_wait_confirm->data[CTRL_FRAME_MASK_WP_NUM+2];

	waypoint_total_num = frame_wait_confirm->data[CTRL_FRAME_MASK_WP_NUM+1] << 8 | frame_wait_confirm->data[CTRL_FRAME_MASK_WP_NUM];
	// if this is the first frame of way point packet, free the way point list in the memory
	if(frame_wait_confirm->frame_id == 1){
	   waypoint_list_clear();
	   // extract total number of way point
	   waypoint_info.total_num = waypoint_total_num;
	}

	if(waypoint_total_num != waypoint_info.total_num){
		/* the total way point number field in the frames of whole packet is different,
		 * which means something is wrong
		 */
		print_err("way point number err\n");
		return;
	}

	for (i = 0; i < waypoint_num_this_frame; i++) {
		waypoint = frame_wait_confirm->data + 4 + i* WAYPOINT_INFO_LEN;
		waypoint_list_add(waypoint);
	}

	if(frame_wait_confirm->frame_id == frame_wait_confirm->frame_num ){
		if(waypoint_info.received_num != waypoint_info.total_num){
		   /* the received way point is not same as total number
			* which means something is wrong
			*/
					print_err("way point missing\n");
					return;
		}


	}
    waypoint_list_current = waypoint_list_head;
}

void waypoint_return(unsigned char *buf, int buf_size)
{
	control_cmd_send(buf, buf_size);
	waypoint_is_ready = 1;
}

void link_test(uint8 *data)
{
    uint32 count;
    count = (data[14]<<24) | (data[13]<<16) | (data[12] << 8 )| data[11];
	print_debug("count %d \n",count);

}

void fault_status_return(uint8 fault)
{
	uint8  buf[15];
	uint16 crc_value;

	buf[0] = CTRL_FRAME_START1;
	buf[1] = CTRL_FRAME_START2;
	buf[2] = get_aircraft_no()&0xFF;
	buf[3] = get_aircraft_no()>>8;
	*(uint16*)(buf+4) = 15;
	buf[6] = 1;
	buf[7] = 0;
	buf[8] = 1;
	buf[9] = 0;
	buf[10] = CTRL_FRAME_TYPE_ERROR;
	buf[11] = fault;
	crc_value=crc_checksum16(buf, 12);
	buf[12] = crc_value&0xFF;
	buf[13] = crc_value>>8;
	buf[14] = CTRL_FRAME_END;
	control_cmd_send(buf, 15);
}

void flying_status_return()
{
	uint8 *fa = (uint8*)(get_flying_attitude());
	uint8  buf[145];
	uint16 crc_value;
	uint8  rc_data[20];

	buf[0] = CTRL_FRAME_START1;
	buf[1] = CTRL_FRAME_START2;
	buf[2] = get_aircraft_no()&0xFF;
	buf[3] = get_aircraft_no()>>8;
	*(uint16*)(buf+4) = 0x91;
	buf[6] = 1;
	buf[7] = 0;
	buf[8] = 1;
	buf[9] = 0;
	buf[10] = CTRL_FRAME_TYPE_FLY_STATUS;
	// compiler add 4bytes between float data and double data for flying_attitude struct
	// so we have to copy separately
	memcpy(buf+11,fa,60);
	memcpy(buf+71,fa+64,36);

	*(uint32*)(buf+107)  = get_sonar_data();

	buf[111] = gepoint.id & 0xFF ;
	buf[112] = gepoint.id >> 8 ;// next waypoint

    if(get_flying_status() == AIRCRAFT_MANUAL_MODE){
       read_rc_data((uint16*)rc_data);
       memcpy(buf+113,rc_data,14);
    }else
       memcpy(buf+113,(uint8 *)(&ppwm),20);//pwm output

    buf[133] = get_flying_status()&0xFF;
    buf[134] = get_flying_status()>>8;

    buf[135] = 0; // gps status
    buf[136] = 0; // imu status
    buf[137] = 0; // AP status :cpu1 or cpu2
    buf[138] = get_input_voltage()&0xFF;//AP power
    buf[139] = get_monitor_voltage()&0xFF;//UAV power
    buf[140] = get_cpu_temperature()/1000;
    buf[141] = 0;

	crc_value=crc_checksum16(buf, 142);
	buf[142] = crc_value&0xFF;
	buf[143] = crc_value>>8;
	buf[144] = CTRL_FRAME_END;
	fwrite(buf,145,1,fp_fly_status);
	control_cmd_send(buf, 145);
}

/*
void master_slaver_sync()
{
	static uint8 count = 0;
	regular_data_s ru_data;
	flying_attitude_s *p = get_flying_attitude();
	ru_data.header = 0x5a5a;
	ru_data.frame_type = 0x3c;
	ru_data.stop = 0x4e;
	ru_data.count = count;
	ru_data.flying_status = get_flying_status();
	ru_data.m_roll = p->roll;
	ru_data.m_pitch = p->pitch;
	ru_data.m_yaw= p->yaw;
	ru_data.m_gx = p->gx;
	ru_data.m_gy = p->gy;
	ru_data.m_gz = p->gz;
	ru_data.crc = crc_checksum16();
	count++;
	master_cmd_send(&ru_data, sizeof(regular_data_s));
}
*/

extern uint32 command;


int poweron_self_check()
{
	int ret = -1;
	int timeout = 0;

    if((fp_fly_status=fopen("fly_status.raw","wb+"))==NULL){
      printf("can not open file:fly_status\n");
    }

	ret=spi_open();
#ifndef debug
	if(ret<0)
		fault_status_return(ret);
#endif
/*
	ret=adc_init();
#ifndef debug
	if(ret<0)
		fault_status_return(ret);
#endif
*/
	ret = sensor_open();
#ifndef debug
	if (ret < 0) {
		fault_status_return(ret);
	}
#endif

	set_flying_status(AIRCRAFT_PREPARING);
	if(command==0){
	//UAV is working in normal mode
		// reset CPLD ,this should be check later ,if CPLD should be reset when CPU recovers from failure.
		reset_control_register(CTRL_REG_MASK_MANUAL);

	   // wait for flying attitude sensor data
	    while (timeout <= 600) {
		    sleep(1);
		    if(flying_attitude_sensor_is_active())
			  break;
		    timeout++;
	    }

	    if (timeout >600) {
		    print_err("flying attitude sensor is inactive, please check serial port is connected\n");
		    goto exit;
	    }
	    print_debug("Flying attitude sensor is active\n");

	}else if(command==1){
	//UAV is working in test mode
		reset_control_register(CTRL_REG_MASK_MANUAL);
		printf("uav is working in test mode,IMU is by passed,using test data instead\n");
	}else if(command==2){
    //UAV is in mannual mode ,for pwm data capture

		set_flying_status(AIRCRAFT_MANUAL_MODE);
		set_control_register(CTRL_REG_MASK_MANUAL);
		set_system_status(SYS_PREPARE_SETTING);
		printf("UAV is now working in data capturing mode\n");
	}else{
		printf(" error command,please check the usage:\n");
        printf(" usage: uav [command] [gap] ]\n");
        printf(" [command]: 0--normal mode,used when run in the air\n");
        printf(" [command]: 1--test mode,used for platform test.use this mode  \n");
        printf("               when no IMU connected ,\n");
        printf(" [command]: 2--manual mode,used when capturing PWM data\n");
        printf(" [frequency]: the frequency (ms/frame) in which UAV send fly status data\n");
     }


	while (1) {
		flying_status_return();
		usleep(500000); //500ms
		if (get_system_status() >= SYS_PREPARE_SETTING) {
			print_debug("Link is ready\n");
			return 0;
		}
		//link_testing_send();
		//usleep(20000); //20ms
	}
exit:
	sensor_close();
	return ret;	
}

static int servo_test_enable=0;
static int servo_test_cnt=0;
void steering_test()
{
   servo_test_cnt++;
   if(servo_test_enable){
	   if(servo_test_cnt==1){

	   }else if(servo_test_cnt==3){

	   }


   }

}

void set_aircaft_preparing_status(unsigned char *buf)
{
	memcpy((uint8*)(&aircraft_preparing_status), buf, sizeof(aircraft_preparing_status));
	update_setting_status(&aircraft_preparing_status);
}
/*
void aircraft_preparing_response()
{
    uint8  buf[23];
    uint16 crc_value;
    buf[0] = CTRL_FRAME_START1;
    buf[1] = CTRL_FRAME_START2;
    buf[2] = get_aircraft_no()&0xFF;
    buf[3] = get_aircraft_no()>>8;
    *(uint8*)(buf+4) = 23;
    buf[8] = CTRL_FRAME_TYPE_AP_SET_ACK;
    memcpy(buf+9,(uint8*)(&aircaft_preparing_status),sizeof(aircaft_preparing_status));
    crc_value=crc_checksum16(buf, 20);
    buf[20] = crc_value&0xFF;
    buf[21] = crc_value>>8;
    buf[22] = CTRL_FRAME_END;

	control_cmd_send(buf, 23);
	prepare_setting_is_ready = 1;
}
*/
//static control_parameter_s control_parameter_remote1;
//static control_parameter_s control_parameter_remote2;
//static control_data_s control_data;
static uint16 control_parameter_remote1[16];
static uint16 control_parameter_remote2[16];
static uint16 control_data[8];

void update_control_parameter_remote1(uint8 *buf)
{
	memcpy(control_parameter_remote1, buf, sizeof(control_parameter_remote1));
}

void update_control_parameter_remote2(uint8 *buf)
{
	memcpy(control_parameter_remote2, buf, sizeof(control_parameter_remote2));
}

void update_control_data(uint8 *buf)
{
	memcpy(control_data, buf, sizeof(control_data));
	update_joystick_data(control_data);
	//write_pwm_data(control_data);
}



/* control_cmd_response_recv()
 * send back the received command to ground after receiving
 */
 void control_cmd_response_recv(uint8 *data,uint16 data_size)
{
	uint16 crc_value;
	uint8  buf[CTRL_FRAME_MAX_LEN];
	uint16 frame_size=data_size + CTRL_FRAME_LEN_NO_DATA;

	buf[0] = CTRL_FRAME_START1;
	buf[1] = CTRL_FRAME_START2;
	buf[2] = get_aircraft_no()&0xFF;
	buf[3] = get_aircraft_no()>>8;
	*(uint16*)(buf+4) = frame_size;
	buf[6] = 1;
	buf[7] = 0;
	buf[8] = 1;
	buf[9] = 0;
	buf[10] = CTRL_FRAME_TYPE_CMD_ACK ;
	memcpy(buf+11,data,data_size);
	crc_value = crc_checksum16(buf, data_size+11);
	buf[frame_size-3] = crc_value&0xFF;
	buf[frame_size-2] = crc_value>>8;
	buf[frame_size-1] = CTRL_FRAME_END;
	control_cmd_send(buf, frame_size);
}

 /* control_cmd_response_exe()
  * send  this command to ground after command execution
  */
  void control_cmd_response_exe(uint8 data)
 {
 	uint16 crc_value;
 	uint8  buf[15];

 	buf[0] = CTRL_FRAME_START1;
 	buf[1] = CTRL_FRAME_START2;
 	buf[2] = get_aircraft_no()&0xFF;
 	buf[3] = get_aircraft_no()>>8;
 	*(uint16*)(buf+4) = 0x0F;
	buf[6] = 1;
	buf[7] = 0;
	buf[8] = 1;
	buf[9] = 0;
 	buf[10] = CTRL_FRAME_TYPE_CMD_EXE ;
 	buf[11] = data;
 	crc_value = crc_checksum16(buf, 12);
 	buf[12] = crc_value&0xFF;
 	buf[13] = crc_value>>8;
 	buf[14] = CTRL_FRAME_END;
 	control_cmd_send(buf, 15);
 }





void control_cmd_confirm()
{
	if (get_system_status() == SYS_PREPARE_SETTING) {
		if (waypoint_is_ready && prepare_setting_is_ready)
			set_system_status(SYS_PREPARE_STEERING_TEST);
		//control_cmd_send(ack_response, 13);
	} else {
		print_err("control_cmd_confirm error: system status is %d\n", get_system_status());
	}
}

void send_version()
{   uint16 crc_value;
	uint8  buf[31];

	buf[0] = CTRL_FRAME_START1;
	buf[1] = CTRL_FRAME_START2;
	buf[2] = get_aircraft_no()&0xFF;
	buf[3] = get_aircraft_no()>>8;
	*(uint16*)(buf+4) = 0x1F;
	buf[6] = 1;
	buf[7] = 0;
	buf[8] = 1;
	buf[9] = 0;
	buf[10] = CTRL_FRAME_TYPE_VERSION ;
	buf[11] = get_board_id();
	*(uint32*)(buf+12) = 0x0; // kernel version
	*(uint32*)(buf+16) = 0x0; // app version
	*(uint32*)(buf+20) = get_fpga_version();
	*(uint32*)(buf+24) = 0x0; // reserve
	crc_value = crc_checksum16(buf, 28);
	buf[28] = crc_value&0xFF;
	buf[29] = crc_value>>8;
	buf[30] = CTRL_FRAME_END;
	control_cmd_send(buf, 31);

}

void data_export()
{
	waypoint_list_s *wp = waypoint_list_head;
	int i=0;
	if(wp==NULL){
		printf("no way point data\n");
		return;
	}

	do{
       printf("%2d,%8f,%lf,%lf,%8f,%x,%x\n",wp->waypoint.id,wp->waypoint.v,wp->waypoint.lon,wp->waypoint.lat,wp->waypoint.h,wp->waypoint.task,wp->waypoint.task_para);
	   wp = wp->next;
	}while(wp!=NULL);
	printf("heli config:");
	printf("%d,%d,",aircraft_preparing_status.h_tp,aircraft_preparing_status.om);
	printf("%d,%d,",aircraft_preparing_status.fc,aircraft_preparing_status.cp_tp);
	printf("%d,%d,",aircraft_preparing_status.o_fp,aircraft_preparing_status.tg);
	printf("%d,%d,",aircraft_preparing_status.max_v,aircraft_preparing_status.g_tp);
	printf("%d,%d,",aircraft_preparing_status.radar,aircraft_preparing_status.reserved1);
	printf("%d\n",aircraft_preparing_status.reserved2);
    printf("flying para1:");
	for(i=0;i<16;i++)
    	printf("%d,",control_parameter_remote1[i]);
	 printf("\n");
	 printf("flying para2:");
		for(i=0;i<16;i++)
	    	printf("%d,",control_parameter_remote2[i]);
		 printf("\n");

		 printf("joystick data:");
			for(i=0;i<8;i++)
		    	printf("%d,",grm.c[i]);
			 printf("\n");


}


