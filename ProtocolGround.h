/*
 * ProtocolGround.h
 *
 *  Created on: Dec 11, 2015
 *      Author: root
 */

#ifndef PROTOCOLGROUND_H_
#define PROTOCOLGROUND_H_

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
#define CTRL_FRAME_TYPE_RESET            0x00        //reset  in autopilot mode
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

int control_data_parse(unsigned char *buf, frame_info *frame_info,frame_wait_confirm *frame_wait_confirm);
unsigned int serial_data_recv_ctrl(frame_info *frame_info ,unsigned char *buf);

#endif /* PROTOCOLGROUND_H_ */
