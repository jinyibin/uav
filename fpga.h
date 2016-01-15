#ifndef FPGA_H
#define FPGA_H
#define  SPI_MODE 0
#define  SPI_BITS_WIDTH_16  16
#define  SPI_LSB      0
#define  SPI_MSB      1
#define  SPI_SPEED  3000000
#define  SPI_DELAY   2 // delay between frames per us
#define  SPI_BUF_SIZE 48


#define  PLANE_SELFTEST  0x0000
#define  PLANE_TAKEOFF   0x0003
#define  PLANE_LANDING   0x000C
#define  PLANE_CRUISE    0x000F

/* SPI write command */
#define  SPI_WRITE_CONTROL_REG      0x0003    /*  control register of FPGA
                                               * bit16 : high ,uart3 switch to CPU2
                                               * bit0  : high , PWM out switch to manual mode
                                               */
#define  CTRL_REG_MASK_MANUAL           0
#define  CTRL_REG_MASK_UART3_SWITCH     15

#define  SPI_WRITE_CPU_MODE         0x0005    /* cpu mode register,cpu have to write the reg every time plane changes mode
                                               * 4'b0000: self test
                                               * 4'b0011: takeoff
                                               * 4'b1100: landing
                                               * 4'b1111: cruise
                                               */

#define  SPI_WRITE_PWM_PERIOD        0x0010
#define  SPI_WRITE_PWM_CH1           0x0011
#define  SPI_WRITE_PWM_CH2           0x0012
#define  SPI_WRITE_PWM_CH3           0x0013
#define  SPI_WRITE_PWM_CH4           0x0014
#define  SPI_WRITE_PWM_CH5           0x0015
#define  SPI_WRITE_PWM_CH6           0x0016
#define  SPI_WRITE_PWM_LOAD          0x00FF  /* write any data to this address will
                                              * load the newly written PWM data to the
                                              * pwm counter,otherwise PWM counter will use pwm data last time
                                              */

#define  SPI_WRITE_SONAR_CONTROL     0x001A  //  sonar control register:0x00 sonar enable(default)
                                             //                             0x01 sonar close

#define  SPI_WRITE_PWM_CH7_PERIOD    0x0020  //  pwm period of channel7
#define  SPI_WRITE_PWM_CH7           0x0021  //  pwm width of channel 7
#define  SPI_WRITE_PWM_CH8_PERIOD    0x0022  //  pwm period of channel8
#define  SPI_WRITE_PWM_CH9           0x0023  //  pwm width of channel 8

/* SPI read command */
#define  SPI_READ_BOARD_ID         0xC001    // read board id
#define  SPI_READ_CONTROL_REG      0xC003    // read control register of FPGA
#define  SPI_READ_CPU_MODE         0xC005    // read cpu mode register

#define  SPI_READ_RC_PERIOD        0xC009    // read pwm period of remote controller
#define  SPI_READ_RC_CH1           0xC00A    // read pwm width of remote controller
#define  SPI_READ_RC_CH2           0xC00B
#define  SPI_READ_RC_CH3           0xC00C
#define  SPI_READ_RC_CH4           0xC00D
#define  SPI_READ_RC_CH5           0xC00E
#define  SPI_READ_RC_CH6           0xC00F

#define  SPI_READ_PWM_PERIOD        0xC010
#define  SPI_READ_PWM_CH1           0xC011
#define  SPI_READ_PWM_CH2           0xC012
#define  SPI_READ_PWM_CH3           0xC013
#define  SPI_READ_PWM_CH4           0xC014
#define  SPI_READ_PWM_CH5           0xC015
#define  SPI_READ_PWM_CH6           0xC016

#define  SPI_READ_SONAR_CONTROL     0xC01A  // read sonar control register:0x00 sonar enable(default)
                                            //                             0x01 sonar close
#define  SPI_READ_SONAR_DATA        0xC01B  // read sonar data 1cm/58us

#define  SPI_READ_VERSION_LOW       0xC01D  // read FPGA software version
#define  SPI_READ_VERSION_HIGH      0xC01E

#define  SPI_READ_PWM_CH7_PERIOD    0xC020  // read pwm period of channel7
#define  SPI_READ_PWM_CH7           0xC021  // read pwm width of channel 7
#define  SPI_READ_PWM_CH8_PERIOD    0xC022  // read pwm period of channel8
#define  SPI_READ_PWM_CH9           0xC023  // read pwm width of channel 8


int read_rc_data(uint16 *data);
int write_pwm_data(uint16 *data);
uint16 spi_read_one_word(uint16 addr);
int spi_write_one_word(uint16 addr, uint16 data);
int spi_write_one_word_no_check(uint16 addr, uint16 data);
int spi_open();
void spi_close();
uint16 get_board_id();
uint32 get_fpga_version();
uint16  get_sonar_data();
int set_control_register(int mask_bit);
int reset_control_register(int mask_bit);
void set_servo_pwm_period(uint16 data);
#endif
