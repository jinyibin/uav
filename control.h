#ifndef CONTROL_H
#define CONTROL_H

#include "datatype.h"
#include "ComManage.h"
#include "ProtocolGround.h"

#define CONTROL_DUTY 20	//20ms
#define CONTROL_FREQUENCY 50	//20ms
#define CONTROL_PERIOD_US 20000	//20000us
#define CONTROL_PERIOD_MS 20	//20ms

/*  err tag */
#define INVALID_CMD 30
#define CMD_TYPE_MISMATCH 31
#define UNSUPPORTED_CMD 32
#define NO_WAYPOINT 33
#define SERIAL_NO_DATA 40

#define SPI_OPEN_FAILED    -2
#define SPI_SETUP_FAILED   -3
#define SPI_DUMP_FAILED    -4
#define PWM_WRITE_FAILED   -5
#define SPI_WRITE_FAILED   -6

#define ADC_TEMP_OPEN_FAILED -7
#define ADC_PS_OPEN_FAILED -8

#define SERIAL_GPS_OPEN_FAILED -9
#define SERIAL_CTRL_OPEN_FAILED -10
#define CTRL_FRAME_CRC_FAILED -11
#define GPS_FRAME_CRC_FAILED -12

#define SERVO_TEST_DATA "/home/parameter/servo_test.raw"
#define HELI_CONFIGURATION "/home/parameter/heli_configuration.csv"
#define CONTROL_PARAMETER "/home/parameter/control_parameter.csv"

typedef struct aircraft_preparing_status_s {
	int8 h_tp;  /*0x1:electric
	              0x2:gas
	              0x4:methanol */
	int8 om;    // fuel volume
	int8 fc;    // fuel mileage
	int8 cp_tp; // swashplate type  0x01:normal  0x02:triangle 0x04:cross
	int8 o_fp;  // servo frequency
	int8 tg;    // tail gyro  0x00: yes 0xff:no
	int8 max_v;
	int8 g_tp;  //gps type 0xff:differential 0x00:no
	int8 radar; //0x00:radar 0xff:no radar
	int8 reserved1;
	int8 reserved2;
} aircraft_preparing_status_s;

/*
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
*/
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
	uint32 status;
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
	float accuracy_atti;
	float accuracy_pos;
	float accuracy_vel;
	uint8  year;
	uint8  month;
	uint8  day;
	uint8  hour;
	uint8  min;
	uint8  sec;
	uint32 nano_sec;

} flying_attitude_s;

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

	int8 gps_status;
	int8 flying_attitude;
	int8 link_status;
	int  engine_voltage;
	int  input_voltage;
	int  cpu_temprature;

} working_status_s;
/*
typedef struct fault_status_s {
	uint16 header; //0x55aa
	uint16 aircraft_no;
	uint32 frame_size;
	uint8 frame_type; //0x38
	uint16 fault;

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
*/
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

struct time_estimation
{
    uint32 data_return;
    uint32 algorithm;
}time_estimation;
FILE *fp_fly_status,*fp_servo_test;
char log_file_name[50];
uint32 servo_test_enable;
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
void set_current_waypoint(int input);
void waypoint_init(frame_wait_confirm *frame_wait_confirm);

void link_test(uint8 *data);
void fault_status_return(uint8 fault);
void flying_status_return(int transmit_data);
int poweron_self_check();
void steering_test();
void set_aircaft_preparing_status(unsigned char *buf);
void aircraft_preparing_response();
void update_control_parameter_remote1(uint8 *buf);
void update_control_parameter_remote2(uint8 *buf);
void update_control_parameter_remote3(uint8 *buf);
void update_control_parameter_remote4(uint8 *buf);
void update_control_data(uint8 *buf);

void control_cmd_confirm();
void control_cmd_response_recv(uint8 *data,uint16 data_size);
int control_cmd_send(uint8 *buf,uint32 buf_size);
void control_cmd_response_exe(uint8 data);

void send_version();
void data_export();
void generate_file_name(char *name);






void firmware_upgrade(uint8 *buf, uint32 size);
uint16 get_aircraft_no();
void set_flying_status(uint16 status);
uint16 get_flying_status();
void gps_time_update(uint32 g_time);



uint64 get_current_time();



#endif
