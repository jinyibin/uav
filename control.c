
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "crc.h"
#include "interface.h"
#include "fpga.h"
#include "adc.h"
#include "control.h"
#include "serial.h"
#include <sys/time.h>
#include <time.h>
#include "ComManage.h"
#include "ProtocolImu.h"
#include "PressureSensor.h"

static int system_status = 0;
static int waypoint_is_ready = 0;
static int prepare_setting_is_ready = 0;
aircraft_preparing_status_s  aircraft_preparing_status;
waypoint_info_s waypoint_info = {0,0};


static waypoint_list_s *waypoint_list_head = NULL;
static waypoint_list_s *waypoint_list_tail = NULL;
static waypoint_list_s *waypoint_list_current = NULL;


static uint16 flying_status = 0;

extern uint32 debug_enable;

uint64 get_current_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}

static void heli_configuration_init()
{
    FILE *fp_heli_config;
    uint32 buf[11];
    int i;
	if((fp_heli_config=fopen(HELI_CONFIGURATION,"r"))==NULL){
      printf("can not open heli configuration file\n");
      return ;
    }
    for(i=0;i<11;i++){
        if(fscanf(fp_heli_config,"%u,",buf+i)==EOF){
        	print_err("heli configuration file error\n");
        	fclose(fp_heli_config);
        	return;
        }
    }
    fclose(fp_heli_config);
    for(i=0;i<11;i++)
    	*((uint8*)(&aircraft_preparing_status)+i)=(uint8)(buf[i]&0xff);
	update_setting_status(&aircraft_preparing_status);

    switch(aircraft_preparing_status.o_fp){
    case SERVO_PWM_PERIOD_20:
    	set_servo_pwm_period(20000);
    	break;
    case SERVO_PWM_PERIOD_14:
    	set_servo_pwm_period(14000);
    	break;
    case SERVO_PWM_PERIOD_7:
        set_servo_pwm_period(7000);
        break;
    case SERVO_PWM_PERIOD_3:
        set_servo_pwm_period(3031);
        break;
    default:
    	break;
    }
}
static void control_parameter_init()
{
    FILE *fp_control_para;
    uint32 buf[32];
    int i;
	if((fp_control_para=fopen(CONTROL_PARAMETER,"r"))==NULL){
      printf("can not open control parameter file\n");
      return ;
    }
    for(i=0;i<32;i++){
        if(fscanf(fp_control_para,"%u,",buf+i)==EOF){
        	print_err("control parameter file error\n");
        	fclose(fp_control_para);
        	return;
        }

    }
    fclose(fp_control_para);
    for(i=0;i<32;i++)
    	K.k[i]=buf[i];
}


void gps_time_update(uint32 g_time)
{

}
uint16 get_flying_status()
{
	return flying_status;
}

void set_flying_status(uint16 status)
{
	flying_status = status;
}

uint16 get_aircraft_no()
{
	return 0x0001;
}

void firmware_upgrade(uint8 *buf, uint32 size)
{
	FILE *fp = NULL;
	#define FILE_PATH "/data/backup/upgrade"
	fp = fopen(FILE_PATH, "w+");
	if (fp == NULL) {
		print_err("File %s open failed\n", FILE_PATH);
		return;
	}
	fwrite(buf, 1, size, fp);
	fclose(fp);
	fp = NULL;
}




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

void set_current_waypoint(int input)
{

	if(input == 0)
		waypoint_list_current = waypoint_list_head;
        //printf("%x %x %x \n",waypoint_list_current->waypoint.id,waypoint_list_head->waypoint.id,waypoint_list_tail->waypoint.id);
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
working_status_s working_status;
void flying_status_return(int transmit_data)
{
	uint8 *fa = (uint8*)(get_flying_attitude());
	uint8  buf[256];
	uint16 crc_value;
	int pressure;

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

    sonar_data=get_sonar_data();
	*(uint32*)(buf+107)  = sonar_data;

	buf[111] = gepoint.id & 0xFF ;
	buf[112] = gepoint.id >> 8 ;// next waypoint

	read_rc_data(rc_data);
    if(get_flying_status() == AIRCRAFT_MANUAL_MODE){
       memcpy(buf+113,rc_data,14);
    }else
       memcpy(buf+113,(uint8 *)(&ppwm),20);//pwm output

    buf[133] = get_flying_status()&0xFF;
    buf[134] = get_flying_status()>>8;

    buf[135] = 0; // gps status
    buf[136] = 0; // imu status
    buf[137] = 0; // AP status :cpu1 or cpu2
    working_status.input_voltage = get_input_voltage();
    working_status.engine_voltage = get_monitor_voltage();
    working_status.cpu_temprature = get_cpu_temperature();
    buf[138] = working_status.input_voltage&0xFF;//AP power
    buf[139] = working_status.engine_voltage&0xFF;//UAV power
    buf[140] = working_status.cpu_temprature/1000;
    buf[141] = 0;

    if(transmit_data){
	    crc_value=crc_checksum16(buf, 142);
	    buf[142] = crc_value&0xFF;
	    buf[143] = crc_value>>8;
	    buf[144] = CTRL_FRAME_END;
	    control_cmd_send(buf, 145);
    }
	*(uint16*)(buf+4) = 0xAF;
	memcpy(buf+142,rc_data,14);
	pressure = get_altimeter();
	*(uint16*)(buf+154)= pressure>>16;
	*(uint16*)(buf+156)= pressure&0xffff;
	*(uint16*)(buf+158)=time_estimation.data_return;
	*(uint16*)(buf+160)=time_estimation.algorithm;
	crc_value=crc_checksum16(buf, 172);
	buf[172] = crc_value&0xFF;
	buf[173] = crc_value>>8;
	buf[174] = CTRL_FRAME_END;
	fwrite(buf,175,1,fp_fly_status);
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

    //--------------spi initial------------------
	ret=spi_open();
    printf("CPLD   logic   version:%d\n",get_fpga_version());
    printf("----------------------------------------------------------------\n");
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
	//---------------imu initial-----------------
	ret = sensor_open();
#ifndef debug
	if (ret < 0) {
		fault_status_return(ret);
	}
#endif
	pressure_sensor_init();
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

	generate_file_name(log_file_name);
    if((fp_fly_status=fopen(log_file_name,"wb+"))==NULL){
      printf("can not open file:%s\n",log_file_name);
    }
    heli_configuration_init();
    control_parameter_init();

    servo_test_enable=0;

	while (1) {
		if(get_system_status() < SYS_PREPARE_STEERING_TEST){
		    flying_status_return(1);
		    usleep(500000);
		}else if(get_system_status() == SYS_PREPARE_STEERING_TEST){
        	steering_test();
        	usleep(20000);
		}else if(get_system_status() == SYS_PREPARE_TAKEOFF){
			return 0;
		}

		/*
		if (get_system_status() >= SYS_PREPARE_SETTING) {
			print_debug("Link is ready\n");
			return 0;
		}
		link_testing_send();
		usleep(20000); //20ms
		*/
	}
exit:
	sensor_close();
	return ret;	
}


void steering_test()
{
    if(fread(&ppwm,20,1,fp_servo_test)==0){
        servo_test_enable=0;
        set_system_status(SYS_PREPARE_TAKEOFF);
        fclose(fp_servo_test);
        printf("servo test over\n");
    }else{
        write_pwm_data((uint16*)&ppwm);
    }

}

void set_aircaft_preparing_status(unsigned char *buf)
{
    FILE *fp;
    int i;

    memcpy((uint8*)(&aircraft_preparing_status), buf, sizeof(aircraft_preparing_status));
	update_setting_status(&aircraft_preparing_status);

	if((fp=fopen(HELI_CONFIGURATION,"w+"))==NULL){
      printf("can not open heli configuration file\n");
      return ;
    }
    for(i=0;i<11;i++)
        fprintf(fp,"%u,",*(((uint8*)&aircraft_preparing_status)+i));
    fclose(fp);

    switch(aircraft_preparing_status.o_fp){
        case SERVO_PWM_PERIOD_20:
        	set_servo_pwm_period(20000);
        	break;
        case SERVO_PWM_PERIOD_14:
        	set_servo_pwm_period(14000);
        	break;
        case SERVO_PWM_PERIOD_7:
            set_servo_pwm_period(7000);
            break;
        case SERVO_PWM_PERIOD_3:
            set_servo_pwm_period(3031);
            break;
        default:
        	break;
        }

	printf("------------------heli config----------------------\n");
	printf("heli type:%4d ,oil_m  :%4d\n",aircraft_preparing_status.h_tp,aircraft_preparing_status.om);
	printf("fuel_cons:%4d ,  cp_tp:%4d\n",aircraft_preparing_status.fc,gsfstate.CP_tp);
	printf("servo_fre:%4d ,     tg:%4d\n",aircraft_preparing_status.o_fp,gsfstate.tg);
	printf("    max_v:%4d , gps_tp:%4d\n",gsfstate.max_v,aircraft_preparing_status.g_tp);
	printf("    radar:%4d ,   rsv1:%4d\n",gsfstate.radar,aircraft_preparing_status.reserved1);
	printf("     rsv2:%4d\n",aircraft_preparing_status.reserved2);
}


static uint16 control_data[8];

void update_control_parameter_remote1(uint8 *buf)
{
    FILE *fp;
    int i;
	memcpy(&K, buf, 32);

	if((fp=fopen(CONTROL_PARAMETER,"w+"))==NULL){
      printf("can not open control parameter file\n");
      return ;
    }
    for(i=0;i<32;i++)
        fprintf(fp,"%u,",(uint32)K.k[i]);
    fclose(fp);

    printf("---------------flying parameter--------------------");
	for(i=0;i<32;i++){
		if((i%8)==0)
	       printf("\n");
    	printf("%d,",K.k[i]);
	}
	printf("\n");
}

void update_control_parameter_remote2(uint8 *buf)
{
    FILE *fp;
    int i;
	memcpy((uint8*)(&K)+32, buf, 32);

	if((fp=fopen(CONTROL_PARAMETER,"w+"))==NULL){
      printf("can not open control parameter file\n");
      return ;
    }
    for(i=0;i<32;i++)
        fprintf(fp,"%u,",(uint32)K.k[i]);
    fclose(fp);

    printf("---------------flying parameter--------------------");
	for(i=0;i<32;i++){
		if((i%8)==0)
	       printf("\n");
    	printf("%d,",K.k[i]);
	}
	printf("\n");
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
	if(debug_enable)
	   printf("control_cmd_response_recv-----type:%x------\n",data[10]);

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
 	if(debug_enable)
 	    printf("control_cmd_response_exe----type:%x-------\n",data);
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
	printf("------------------waypoint----------------------\n");
	do{
       printf("%2d,%8f,%lf,%lf,%8f,%x,%x\n",wp->waypoint.id,wp->waypoint.v,wp->waypoint.lon,wp->waypoint.lat,wp->waypoint.h,wp->waypoint.task,wp->waypoint.task_para);
	   wp = wp->next;
	}while(wp!=NULL);
    printf("---------------flying parameter--------------------");
	for(i=0;i<32;i++){
		if((i%8)==0)
	       printf("\n");
    	printf("%d,",K.k[i]);
	}
	printf("\n");

	printf("------------------heli config----------------------\n");
		printf("heli type:%4d ,oil_m  :%4d\n",aircraft_preparing_status.h_tp,aircraft_preparing_status.om);
		printf("fuel_cons:%4d ,  cp_tp:%4d\n",aircraft_preparing_status.fc,gsfstate.CP_tp);
		printf("servo_fre:%4d ,     tg:%4d\n",aircraft_preparing_status.o_fp,gsfstate.tg);
		printf("    max_v:%4d , gps_tp:%4d\n",gsfstate.max_v,aircraft_preparing_status.g_tp);
		printf("    radar:%4d ,   rsv1:%4d\n",gsfstate.radar,aircraft_preparing_status.reserved1);
		printf("     rsv2:%4d\n",aircraft_preparing_status.reserved2);

		 printf("-----------------joystick data----------------------------\n");
			for(i=0;i<8;i++)
		    	printf("%d,",grm.c[i]);
			 printf("\n");


}

int control_cmd_send(uint8 *buf,uint32 buf_size)
{
	return serial_write(control_fd, buf, buf_size);

}

void generate_file_name(char *name)
{
	struct tm *local_time;
	time_t local_time_seconds;
	local_time_seconds=time(NULL);
	local_time=localtime(&local_time_seconds);
	strftime(name,sizeof(log_file_name),"/home/log/%Y%m%d%H%M%S.raw",local_time);
}

