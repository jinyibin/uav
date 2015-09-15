#ifndef CONTROL_H
#define CONTROL_H

#include "datatype.h"
#include "sensor.h"


#define CONTROL_DUTY 20	//20ms

/* control frame format */
#define CTRL_FRAME_MINIMUM_LEN 15
#define CTRL_FRAME_MAX_LEN     4096

#define CTRL_FRAME_START1 0xAA
#define CTRL_FRAME_START2 0x55
#define CTRL_FRAME_END 0x4E
#define CTRL_FRAME_LEN_NO_DATA 14  // total length of field other than data

#define CTRL_FRAME_MASK_FRAME_SIZE  4     // frame size field position start from 0
#define CTRL_FRAME_MASK_FRAME_NUM   6     // frame num field position start from 0
#define CTRL_FRAME_MASK_FRAME_ID    8     // frame id field position start from 0
#define CTRL_FRAME_MASK_FRAME_TYPE  10    // frame type field position start from 0
#define CTRL_FRAME_MASK_DATA        11    // data field position start from 0

#define CTRL_FRAME_MASK_WP_ID      4     // way point id position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_PARA     8     // way point parameter position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_V        8     // way point v position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_LONG     12     // way point long position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_LAT      20     // way point lat position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_H        28     // way point h position in the frame-waypoint modify
#define CTRL_FRAME_MASK_WP_TASK     6     // way point task position in the frame-waypoint modify
#define WAYPOINT_INFO_LEN     28    // byte length of way point information

#define CTRL_FRAME_MASK_WP_NUM  0      // way point number of received frame position in the frame waypoint set


/* frame from gound  */
#define CTRL_FRAME_TYPE_HELI_CONFIG      0x38        // heli configure including heli type,oil volume etc.
#define CTRL_FRAME_TYPE_WAYPOINT_MODIFY  0x8B        // modify way point
#define CTRL_FRAME_TYPE_FLY_PARA1        0x33        // set flying parameter1
#define CTRL_FRAME_TYPE_FLY_PARA2        0x3E        // set flying parameter2
#define CTRL_FRAME_TYPE_FIRM_UPDATE      0xFF        // update firmware
#define CTRL_FRAME_TYPE_WAYPOINT_INIT     0x8A        // initialize waypoint
#define CTRL_FRAME_TYPE_SERVO_TEST       0x0C        // tell AP to test servos
#define CTRL_FRAME_TYPE_TAKEOFF          0x50        // tell AP to take off automatically
#define CTRL_FRAME_TYPE_REMOTE_CTRL1     0x8E
#define CTRL_FRAME_TYPE_REMOTE_CTRL2     0xB2
#define CTRL_FRAME_TYPE_HOVER            0x88        // hovering the plane
#define CTRL_FRAME_TYPE_FLYING           0xCA        // fly according to the waypoint
#define CTRL_FRAME_TYPE_RETURN           0x5F        // fly back
#define CTRL_FRAME_TYPE_LAND             0x3C
#define CTRL_FRAME_TYPE_VERSION_READ     0x4D        // reading version of software and hardware
#define CTRL_FRAME_TYPE_CMD_CONFIRM      0x80
#define CTRL_FRAME_TYPE_STICK_DATA       0xD6        // data from joystick
#define CTRL_FRAME_TYPE_LINK_TEST        0x69
#define CTRL_FRAME_TYPE_EXPORT_DATA      0x86
#define CTRL_FRAME_TYPE_GROUND_OK        0xEF        //gound check over,ready to go
#define CTRL_FRAME_TYPE_MANUAL_MODE      0x5E        //heli is controlled manually
/* frame to the ground   */
#define CTRL_FRAME_TYPE_FLY_STATUS       0x55
#define CTRL_FRAME_TYPE_ERROR            0x41
#define CTRL_FRAME_TYPE_CMD_ACK          0x39        // command response after receiving
#define CTRL_FRAME_TYPE_CMD_EXE          0x81        // command response after execute
#define CTRL_FRAME_TYPE_VERSION          0x49        // sending version

/* waypoint process type */
#define WAYPOINT_INSERT    0x01
#define WAYPOINT_MODIFY    0x02
#define WAYPOINT_DELETE    0x04

typedef struct aircraft_preparing_status_s {
	//uint16 header; //0xaa55
	//uint16 aircraft_no;
	//uint32 frame_size;
	//uint8 frame_type; //0x38
	int8 h_tp;//0x1 0x2 0x4
	int8 om;
	int8 fc;
	int8 cp_tp;
	int8 o_fp;
	int8 tg;
	int8 max_v;
	int8 g_tp;
	int8 radar;
	int8 reserved1;
	int8 reserved2;
	//uint16 crc;
	//uint8 stop; //0x4e
} aircraft_preparing_status_s;


typedef struct control_parameter_s {
	uint16 header; //0xaa55
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x33 or 0x3e
	uint16 parameter[16];
	uint16 crc;
	uint8 stop; //0x4e
} control_parameter_s;

typedef struct control_data_s {
	uint16 header; //0xaa55
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0xd6
	uint16 data[8];
	uint16 crc;
	uint8 stop; //0x4e
} control_data_s;

typedef struct firmware_upgrade_s {
	uint16 header; //0xaa55
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //type = 0x33 or 0x3e
	uint32 size;
	uint8 *data;
	uint8 stop; //0x4e
} firmware_upgrade_s;

typedef struct waypoint_s {
	uint16 id;
	unsigned char task;
	unsigned char task_para;
	float v;
	double lon;
	double lat;
	float h;
} waypoint_s;

typedef struct flying_attitude_s {
	//uint16 header; //0xff02
	//uint8 command;
	//uint16 size; //0x0060
	float roll;
	float pitch;
	float yaw;
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	uint32 g_time;
	int vn;
	int ve;
	int vd;
	int heading;
	int b_h;
	double lat;
	double Long;
	double g_h;
	float vx;
	float vy;
	float vz;

	//uint16 crc;
	//uint8 stop; //0x03
} flying_attitude_s;

typedef struct remote_control_s {
	uint16 header; //0xaa55
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0xd6
	uint16 rm[8];
	uint16 crc;
	uint8 stop; //0x4e
} remote_control_s;

typedef struct link_testing_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x69
	int8 count;
	uint16 crc;
	uint8 stop; //0x4e
} link_testing_s;

typedef struct flying_status_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x50
	float roll;
	float pitch;
	float yaw;
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	int vn;
	int ve;
	int vd;
	int heading;
	int b_h;
	double lat;
	double Long;
	double g_h;
	uint16 g_time;
	float vx;
	float vy;
	float vz;
	float l_h;//laser
	uint16 waypoint_dest;
	uint16 pwm[10];
	uint16 status;
	uint16 crc;
	uint8 stop; //0x4e
} flying_status_s;

typedef struct working_status_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x55
	int8 gps_status;
	int8 flying_attitude;
	int8 link_status;
	int8 computer_status;
	int8 battery_status;
	int8 temp_volt;
	int8 reserved[2];
	uint16 crc;
	uint8 stop; //0x4e
} working_status_s;

typedef struct fault_status_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x38
	uint16 fault;
	/* 0x1 gps;
	0x2 gps can't calculate;
	0x4 master fault;
	0x8 slaver fault;
	0x10 aircraft fault1;
	0x20 aircraft fault2 */
	uint16 crc;
	uint8 stop; //0x4e
} fault_status_s;

typedef struct ack_response_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x80
	int8 data; //0xd8
	uint16 crc;
	uint8 stop; //0x4e
} ack_response_s;

typedef struct waypoint_setup_return_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint8 frame_type; //0x4c return on land 0xc4 return in air
	waypoint_s *waypoint;
	uint16 crc;
	uint8 stop; //0x4e
} waypoint_setup_return_s;

typedef struct status_setup_return_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint8 frame_type; //0x3d
	//????
	uint16 crc;
	uint8 stop; //0x4e
} status_setup_return_s;

typedef struct flying_data_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint8 frame_type; //0x88
	uint32 size;
	uint8 *data; //
	uint16 crc;
	uint8 stop; //0x4e
} flying_data_s;

typedef struct regular_data_s {
	uint16 header; //0x5a5a
	uint8 frame_type; //0x3c
	uint8 count;
	uint16 flying_status;
	float m_roll;
	float m_pitch;
	float m_yaw;
	float m_gx;
	float m_gy;
	float m_gz;
	uint16 pwm[5];
	uint16 crc;
	uint8 stop; //0x4e
} regular_data_s;

typedef struct unregular_data_s {
	uint16 header; //0x5a5a
	uint8 frame_type; //0x81

	int8 h_tp;//0x1 0x2 0x4
	int8 om;
	int8 fc;
	int8 cp_tp;
	int8 o_fp;
	int8 tg;
	int8 max_v;
	int8 g_tp;
	int8 radar;

	float pl_v;
	double pl_long;
	double pl_lat;
	float pl_gh;

	uint16 crc;
	uint8 stop; //0x4e
} unregular_data_s;
typedef struct waypoint_list_s waypoint_list_s;
typedef struct waypoint_list_s {
	waypoint_s waypoint;
	waypoint_list_s *prev;
	waypoint_list_s *next;
} waypoint_list_s;

typedef struct waypoint_info_s
{
	uint16 total_num;     // total number of way point
	uint16 received_num;  // number of way point that has been received
} waypoint_info_s;


enum SYSTEM_STATUS {
	SYS_INIT = 0,
	SYS_SENSOR_READY,
	SYS_PREPARE_SETTING,
	SYS_PREPARE_STEERING_TEST,
	SYS_PREPARE_TAKEOFF,
};

FILE *fp_fly_status;
//FILE *fp_way_point;


void set_system_status(int status);
int get_system_status();

int inflight_waypoint_modify(frame_wait_confirm *frame_wait_confirm);
int waypoint_delete (uint8 *waypoint, int no);
int waypoint_insert (uint8 *waypoint, int no);
int waypoint_modify (uint8 *waypoint, int no);
waypoint_list_s *get_waypoint_head();
waypoint_list_s *get_waypoint_tail();
waypoint_list_s *get_waypoint_previous();
waypoint_list_s *get_waypoint_next();
waypoint_list_s *get_waypoint_current();
void waypoint_init(frame_wait_confirm *frame_wait_confirm);

void link_test(uint8 *data);
void fault_status_return(uint8 fault);
void flying_status_return();
int poweron_self_check();
void steering_test();
void set_aircaft_preparing_status(unsigned char *buf);
void aircraft_preparing_response();
void update_control_parameter_remote1(uint8 *buf);
void update_control_parameter_remote2(uint8 *buf);
void update_control_data(uint8 *buf);

void control_cmd_confirm();
void control_cmd_response_recv(uint8 *data,uint16 data_size);
void control_cmd_response_exe(uint8 data);
void send_version();
void data_export();



#endif
