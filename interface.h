#ifndef INTERFACE_H
#define INTERFACE_H

#include "datatype.h"

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

struct sstate {
char CP_tp;
char tg;
char max_v;
char radar;
}; 

struct wstate {
	double posi[3];
	float stan_p[3];
	unsigned char statG[5];
};


struct spoint {
	uint16 id;
	float v;
	double lon;
	double lat;
	float h;
	char task;
};
struct epoint {
	uint16 id;
	float v;
	double lon;
	double lat;
	float h;
	char task;
};

struct pwm {
	uint16 c[9];
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

struct fstate gfstate; //flying status
struct rm grm; //joystick data
struct sstate gsfstate; //setting status

struct wstate gwstate; //work status

static struct spoint gspoint; //start flying point
static struct epoint gepoint; //end flying point

struct pwm ppwm; //PWM output

int route_n;//Flying line number

struct lfstate glfstate; //flying status used last time

void auto_flying_start();
void auto_flying_stop();
void get_flyling_line_point(int forward);

void update_setting_status( aircraft_preparing_status_s  *aps);
void update_joystick_data(uint16 *data);

#endif

