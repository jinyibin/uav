#ifndef STATUS_H
#define STATUS_H
#include "datatype.h"
#include "control.h"
//flying status
#define AIRCRAFT_PREPARING 0x0
#define AIRCRAFT_READY 0x1
#define AIRCRAFT_TAKEOFF 0x2
#define AIRCRAFT_LANDING 0x4
#define AIRCRAFT_FLYING 0x8
#define AIRCRAFT_TASK 0x10 
#define AIRCRAFT_REMOTE1 0x20
#define AIRCRAFT_REMOTE2 0x40
#define AIRCRAFT_HOVERING 0x80
#define AIRCRAFT_RETURN 0x100
#define AIRCRAFT_OBS_AVOIDING 0x1000
#define AIRCRAFT_MANUAL_MODE 0x2000
#define AIRCRAFT_FAULT 0xFF

#define REMOTE1_VALID  (AIRCRAFT_FLYING | AIRCRAFT_TASK | AIRCRAFT_HOVERING | AIRCRAFT_RETURN)
#define REMOTE2_VALID  (AIRCRAFT_FLYING | AIRCRAFT_TASK | AIRCRAFT_HOVERING | AIRCRAFT_RETURN)
#define HOVER_VALID    (AIRCRAFT_FLYING | AIRCRAFT_TASK |  AIRCRAFT_RETURN)
#define RETURN_VALID   (AIRCRAFT_FLYING | AIRCRAFT_TASK | AIRCRAFT_HOVERING | AIRCRAFT_REMOTE1 | AIRCRAFT_REMOTE1)
#define LAND_VALID     (AIRCRAFT_TAKEOFF | AIRCRAFT_HOVERING | AIRCRAFT_REMOTE1 | AIRCRAFT_REMOTE1)
#define WP_MODIFY_VALID  (AIRCRAFT_FLYING | AIRCRAFT_TASK |  AIRCRAFT_READY)
#define FLYING_VALID   (AIRCRAFT_HOVERING | AIRCRAFT_REMOTE1 | AIRCRAFT_REMOTE1)
#define RESET_VALID   (AIRCRAFT_PREPARING | AIRCRAFT_READY | AIRCRAFT_MANUAL_MODE)

int flying_attitude_sensor_is_active();
void firmware_upgrade(uint8 *buf, uint32 size);
uint16 get_aircraft_no();
void set_flying_status(uint16 status);
uint16 get_flying_status();
void gps_time_update(uint32 g_time);
void set_flying_attitude(uint8 *buf);

flying_attitude_s *get_flying_attitude();
uint64 get_current_time();

#endif
