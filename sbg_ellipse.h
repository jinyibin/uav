
#ifndef SBG_ELLIPSE_H_
#define SBG_ELLIPSE_H_

#define BUF_SIZE_ELLIPSE 512
#define ELLIPSE_FRAME_LEN 130

#define ELLIPSE_FRAME_START1       0xFF
#define ELLIPSE_FRAME_START2       0x5A
#define ELLIPSE_FRAME_START_LENGTH 2
#define ELLIPSE_FRAME_END          0x33
#define ELLIPSE_FRAME_MINIMUM_LEN  23
#define ELLIPSE_FRAME_MAX_LEN     100

#define ELLIPSE_FRAME_LEN_NO_DATA 9       // total length of field other than data
#define ELLIPSE_FRAME_CRC_LEN
#define ELLIPSE_FRAME_MASK_MSG_ID   2        // msg id field position start from 0
#define ELLIPSE_FRAME_MASK_MSG_CLASS 3
#define ELLIPSE_FRAME_MASK_FRAME_SIZE 4   // frame size field position start from 0


#define ELLIPSE_FRAME_MSG_CLASS_LOG_ECOM0 0x00

#define ELLIPSE_FRAME_MSG_ID_STATUS    0x01
#define ELLIPSE_FRAME_MSG_ID_UTC_TIME  0x02
#define ELLIPSE_FRAME_MSG_ID_IMU_DATA  0x03
#define ELLIPSE_FRAME_MSG_ID_EKF_EULER 0x06
#define ELLIPSE_FRAME_MSG_ID_EKF_NAV   0x08
#define ELLIPSE_FRAME_MSG_ID_MAG       0x04
#define ELLIPSE_FRAME_MSG_ID_PRESSURE  0x36

#define ELLIPSE_FRAME_DATA_MASK 6

#define ELLIPSE_MSG_IMU_MASK_ACCEL_X   (6)     // m/s2
#define ELLIPSE_MSG_IMU_MASK_ACCEL_Y   (10)    // m/s2
#define ELLIPSE_MSG_IMU_MASK_ACCEL_Z   (14)   // m/s2
#define ELLIPSE_MSG_IMU_MASK_GYRO_X   (18 )    // rad/s
#define ELLIPSE_MSG_IMU_MASK_GYRO_Y   (22)     // rad/s
#define ELLIPSE_MSG_IMU_MASK_GYRO_Z   (26)     // rad/s

#define ELLIPSE_MSG_EKF_EULER_MASK_ROLL  (4)   // rad
#define ELLIPSE_MSG_EKF_EULER_MASK_PITCH (8)   // rad
#define ELLIPSE_MSG_EKF_EULER_MASK_YAW   (12)  // rad

#define ELLIPSE_MSG_EKF_NAV_MASK_VEL_N     (4)  // m/s
#define ELLIPSE_MSG_EKF_NAV_MASK_VEL_E     (8)  // m/s
#define ELLIPSE_MSG_EKF_NAV_MASK_VEL_D     (12) // m/s
#define ELLIPSE_MSG_EKF_NAV_MASK_LATI      (28)
#define ELLIPSE_MSG_EKF_NAV_MASK_LONG      (36)
#define ELLIPSE_MSG_EKF_NAV_MASK_ALTI      (44) // m

#define ELLIPSE_MSG_PRESSURE_MASK_ALTI     (10) // m

#define ELLIPSE_MSG_MASK_TIME_STAMP     (0)
#define ELLIPSE_MSG_UTC_MASK_YEAR       (6)
#define ELLIPSE_MSG_UTC_MASK_MONTH      (8)
#define ELLIPSE_MSG_UTC_MASK_DAY        (9)
#define ELLIPSE_MSG_UTC_MASK_HOUR       (10)
#define ELLIPSE_MSG_UTC_MASK_MIN        (11)
#define ELLIPSE_MSG_UTC_MASK_SEC        (12)



unsigned int serial_data_recv_ellipse(frame_info *frame_info,unsigned char *buf);
void set_flying_attitude_ellipse(uint8 *buf);
void ellipse_data_parse(unsigned char *buf, frame_info *frame_info);
#ifdef USE_SBG_ELLIPSE
int flying_attitude_sensor_is_active();
flying_attitude_s *get_flying_attitude();
#endif

#endif
