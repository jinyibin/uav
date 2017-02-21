#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include "control.h"
#include "interface.h"
#include "ProtocolImu.h"
#include "ac.h"
#include "fpga.h"




/* Forward +1 -1*/
void get_flyling_line_point(int forward)
{
	waypoint_list_s *wp = get_waypoint_current();
	flying_attitude_s *fa = get_flying_attitude();
	/*
	if(wp==NULL){
		printf("get_flying_line_point:please make sure waypoint have been uploaded \n");
		return;
	}
*/
	if (forward == 1) {
		if (get_flying_status() <= AIRCRAFT_TAKEOFF) {

			//memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			//memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));
			take_off_heading = fa->yaw;

			//set current position as takeoff position
            gspoint.id=0;
            gspoint.task=0;
            gspoint.task_para=0;
            gspoint.lat = fa->lat;
            gspoint.lon= fa->Long;
            gspoint.h= fa->g_h;
            gspoint.v= 0;

            memcpy(&gepoint, &gspoint, sizeof(gepoint));


		} else if (wp == get_waypoint_tail()){
			memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			wp = get_waypoint_head();
			memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));

			set_flying_status(AIRCRAFT_RETURN)  ;
			set_current_waypoint(0);//set current waypoint as head;

		} else if (get_flying_status() == AIRCRAFT_RETURN){
			memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));
		} else {
			memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			wp = get_waypoint_next();
			memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));

		}
	}else if (forward == -1) {
		if (wp == get_waypoint_head() || get_flying_status() == AIRCRAFT_LANDING) {
			memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));
		} else {
			memcpy(&gspoint, &wp->waypoint, sizeof(gspoint));
			wp = get_waypoint_previous();
			memcpy(&gepoint, &wp->waypoint, sizeof(gepoint));
		}
	}

}

int waypoint_check()
{
	waypoint_list_s *wp = get_waypoint_current();

	if(wp==NULL){
		printf("no waypoint,please make sure waypoint have been uploaded \n");
		return 0;
	}else
		return 1;
}

void update_setting_status(aircraft_preparing_status_s *aps)
{
	gsfstate.CP_tp = aps->cp_tp;
	gsfstate.tg = aps->tg;
	gsfstate.max_v = aps->max_v;
	gsfstate.radar = aps->radar;
}

void update_joystick_data(uint16 *data)
{
	memcpy(grm.c, data, sizeof(struct rm));
}

static pthread_t execute_pid;
static int running = 1;
//static pthread_cond_t timer_cond;
#define TIMER_CYCLE 20000 //20MS
/*
static void *auto_flying_execute()
{
	struct timeval tpStart,tpEnd;
	struct timespec ts;
	float timeUse;
	flying_attitude_s *fa;
	int i=0;
	uint64 current_ts, timer_ts;

	gettimeofday(&tpStart, NULL);
	timer_ts = tpStart.tv_sec * 1000000 + tpStart.tv_usec;

	while(running) {
		fa=get_flying_attitude();
	    gettimeofday(&tpStart, NULL);

		current_ts = tpStart.tv_sec * 1000000 + tpStart.tv_usec;
		if ((current_ts - timer_ts) < TIMER_CYCLE) {
			ts.tv_nsec = (tpStart.tv_usec + (current_ts - timer_ts)) * 1000;
			ts.tv_sec = tpStart.tv_sec;
			if (ts.tv_nsec > 1000000000) {
				ts.tv_nsec -= 1000000000;
				ts.tv_sec += 1;
			}
			pthread_cond_timedwait(&timer_cond, NULL, &ts);
		} else {
			print_err("Thread cost too much time\n");
		}
		timer_ts += TIMER_CYCLE;

	    print_debug("frame %4d  frame time:%d systime:%f,%d\n",i++,fa->g_time,tpStart.tv_sec*1000+tpStart.tv_usec*0.001,sizeof(flying_attitude_s));
		do {
		        gettimeofday(&tpEnd, NULL);
		        timeUse = 1000 * (tpEnd.tv_sec - tpStart.tv_sec) + 0.001 * (tpEnd.tv_usec - tpStart.tv_usec);
		} while(timeUse < 4);
	}
}
*/

extern uint32 command;
extern uint32 counter_global;
static void *auto_flying_execute()
{

	flying_attitude_s *fa;
	uint64 current_ts, timer_ts;

	while(running) {
		counter_global++;

		if(command==0){
		// running in normal mode
			//printf("running control algorithm\n");
			counter_global=0;
		}else if(command==1){
		// running in test mode.
			fa=get_flying_attitude();
			fa->roll=0+((float)(counter_global%450)/3600)*(2*PI);
			fa->pitch=0+((float)(counter_global%450)/3600)*(2*PI);
			fa->yaw=0+((float)(counter_global%3600)/3600)*(2*PI);
			fa->gx=0;
			fa->gy=0;
			fa->gz=0;
			fa->ax=0;
			fa->ay=0;
			fa->az=0;
			fa->g_time=0+counter_global*20;
			fa->vn=0;
			fa->ve=0;
			fa->vd=0;
			fa->heading=0+(counter_global%(360*10))*10000;
			fa->b_h=0+(counter_global%5000)*100;
			fa->lat=40.161499+counter_global/100000;
			fa->Long=116.260435+counter_global/100000;
			fa->g_h=66.888026+counter_global/100000;
			fa->vx=0;
			fa->vy=0;
			fa->vz=0;
			ppwm.c[0]=1000+counter_global%1000;
			ppwm.c[1]=500+counter_global%1000;
			ppwm.c[2]=1500+counter_global%1000;
			ppwm.c[3]=2000+counter_global%1000;
			ppwm.c[4]=3000+counter_global%1000;
			ppwm.c[5]=4000+counter_global%1000;
		}

		timer_ts = get_current_time();
#ifdef HELI
        negative();
#endif

#ifdef MULTIROTOR_8

        negative();
        /*
        //for test only,run in extreme low speed
        if(rc_data[2]<1170){//read remote pitch data
           ppwm.c[0]=rc_data[2];
           ppwm.c[1]=rc_data[2];
           ppwm.c[2]=rc_data[2];
           ppwm.c[3]=rc_data[2];
           ppwm.c[4]=rc_data[2];
           ppwm.c[5]=rc_data[2];
           ppwm.c[6]=rc_data[2];
           ppwm.c[7]=rc_data[2];
           ppwm.c[8]=rc_data[2];
        }
        write_pwm_data((uint16*)&ppwm);
        */

#endif

#ifdef MULTIROTOR_6

#endif
		current_ts = get_current_time();
		time_estimation.algorithm=current_ts - timer_ts;
		if (time_estimation.algorithm < TIMER_CYCLE)
		    usleep(TIMER_CYCLE-time_estimation.algorithm);
		else
			print_err("Thread cost too much time\n");

	}

}


void auto_flying_start()
{
	running = 1;
	//pthread_cond_init(&timer_cond, NULL);
	pthread_create(&execute_pid, NULL, auto_flying_execute, NULL);
}

void auto_flying_stop()
{
	void* result = NULL;
	running = 0;
	pthread_join(execute_pid,&result);
}
