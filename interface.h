#ifndef INTERFACE_H
#define INTERFACE_H

#include "datatype.h"
#include "control.h"

struct fstate {
	float att[3];
	float att_v[3];
	float acc_xyz[3];
	float v_neu[3];
	float h[2];
	double posi[3];
	float v_xyz[3];
	int state;
};

struct rm {
	uint16 c[8];//Should be u16
};
uint16 rc_data[7];


struct sstate {
char CP_tp;
char tg;
char max_v;
char radar;
}; 

//struct wstate {                          //2015-11-21 删除
//	double posi[3];
//	float stan_p[3];
//	unsigned char statG[5];
//};


struct spoint {

	uint16 id;
	unsigned char task;
	unsigned char task_para;
	float v;
	double lon;
	double lat;
	float h;

};
struct epoint {

	uint16 id;
	unsigned char task;
	unsigned char task_para;
	float v;
	double lon;
	double lat;
	float h;
};

struct pwm {
	uint16 c[10];
};


struct lfstate {
	float att[3];
	float att_v[3];
	float acc_xyz[3];
	float v_neu[3];
	float h[2];
	double posi[3];
	float v_xyz[3];
};

struct parameter {      //2015-11-15 王强新增
	int16 k[32];
};

struct fstate gfstate; //flying status
struct rm grm; //joystick data
struct sstate gsfstate; //setting status

//struct wstate gwstate; //work status

struct spoint gspoint; //start flying point
struct epoint gepoint; //end flying point
float    take_off_heading;

struct parameter K;//parameter of controller setting by base-station  //2015-11-15 王强新增

struct pwm ppwm; //PWM output

int route_n;//Flying line number
uint16 operation_time;
uint16 sonar_data;

struct lfstate glfstate; //flying status used last time

void auto_flying_start();
void auto_flying_stop();
void get_flyling_line_point(int forward);
int waypoint_check();

void update_setting_status( aircraft_preparing_status_s  *aps);
void update_joystick_data(uint16 *data);

#endif

