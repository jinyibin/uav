#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>


#include "datatype.h"
#include "fpga.h"


static const char *spidev = "/dev/spidev0.0";
static int spi_fd = -1;
static uint8 spi_tx[SPI_BUF_SIZE];
static uint8 spi_rx[SPI_BUF_SIZE];
struct spi_ioc_transfer spi_tr;

static int spi_setup(int fd)
{
	int ret = -1;
	unsigned int mode = SPI_MODE;
	unsigned int bits = SPI_BITS_WIDTH_16;
	unsigned int lsb = SPI_LSB;
	unsigned int speed = SPI_SPEED;



	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		print_err("can't set spi mode");
		return SPI_SETUP_FAILED;
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		print_err("can't set bits per word");
		return SPI_SETUP_FAILED;
	}

	ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb);
	if (ret == -1) {
		print_err("can't set lsb first");
		return SPI_SETUP_FAILED;
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		print_err("can't set max speed hz");
		return SPI_SETUP_FAILED;
	}

	return ret;
}


static int dumpstat(const char *name, int fd)
{
	unsigned int mode, lsb, bits;
	unsigned int speed;

	if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		print_err("cannot get spi mode");
		return SPI_DUMP_FAILED;
	}
	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
		print_err("cannot get lsb first");
		return SPI_DUMP_FAILED;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
		print_err("cannot get bits per word");
		return SPI_DUMP_FAILED;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		print_err("cannot get spi max speed");
		return SPI_DUMP_FAILED;
	}


	printf("SPI mode: ........ 0x%02X\n", mode);
	printf("Bits per word: ... %d\n", bits);
	printf("LSB First: ....... %d\n", lsb);
	printf("Max speed: ....... %d Hz \n", speed);

	return 1;
}


int spi_open()
{
	int ret=-1;
	spi_fd = open(spidev, O_RDWR);
	if (spi_fd < 0) {
		print_err("open %s failed\n", spidev);
		return SPI_OPEN_FAILED;
	}
	ret=spi_setup(spi_fd);

	if(ret<0)
		//spi setup failed
		return ret;
	else
		ret=dumpstat(spidev,spi_fd);

	if(ret<0)
		//spi dump failed
		return ret;

		spi_tr.tx_buf = (unsigned long)spi_tx;
		spi_tr.rx_buf = (unsigned long)spi_rx;
		spi_tr.len = 0;
	    spi_tr.delay_usecs = SPI_DELAY;
		spi_tr.speed_hz = SPI_SPEED;
		spi_tr.bits_per_word = SPI_BITS_WIDTH_16;


	return spi_fd;
}


void spi_close()
{
	if (spi_fd > 0)
		close(spi_fd);
	spi_fd = -1;
}


uint16 spi_read_one_word(uint16 addr)
{
	int ret;

	spi_tr.len = 4;

	spi_tx[0] = addr & 0xff ;
	spi_tx[1] = addr >> 8;
	spi_tx[2] = 0;
	spi_tx[3] = 0;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_tr);
	if (ret < 1){
		print_err("can't send spi message");
		return 0;
	}
	else{
        return *(uint16*)(spi_rx+2);
	}

}


int spi_write_one_word(uint16 addr, uint16 data)
{
	int ret;

	spi_tr.len = 8;

	// write data
    spi_tx[0] = addr & 0xff ;
    spi_tx[1] = addr >> 8;
    spi_tx[2] = data & 0xff;
    spi_tx[3] = data >> 8;
    // send read command to read back the data for checking
    spi_tx[4] = (0xC000|addr) & 0xff ;
    spi_tx[5] = (0xC000|addr) >> 8;
    spi_tx[6] = 0;
    spi_tx[7] = 0;

 	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_tr);
	if (ret < 1){
		print_err("can't send spi word\n");
	    return -1;
	}
	// compare the data read back with the original data
    if(data != (*(uint16*)(spi_rx+6))){
#ifdef debug
    	   print_err("spi word write error \n");
#else
    	   fault_status_return(SPI_WRITE_FAILED);
#endif

    	 return SPI_WRITE_FAILED;
    }
    return ret;
}

 int spi_write_one_word_no_check(uint16 addr, uint16 data)
{
	int ret;

    spi_tr.len = 4;
	// write data
	spi_tx[0] = addr & 0xff ;
	spi_tx[1] = addr >> 8;
	spi_tx[2] = data & 0xff;
	spi_tx[3] = data >> 8;

 	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_tr);
	if (ret < 1){
		print_err("can't send spi word\n");
		ret = -1;
	}

    return ret;
}

int write_pwm_data(uint16 *data)
{
	int ret;

	uint16 *p;
	uint16 *d;
	uint8 i;

	spi_tr.len = 48;
	p=(uint16*)spi_tx;
	d=(uint16*)spi_rx;
	// write data
     *p      = SPI_WRITE_PWM_CH1;
     *(p+1)  = data[0];
     *(p+2)  = SPI_WRITE_PWM_CH2;
     *(p+3)  = data[1];
     *(p+4)  = SPI_WRITE_PWM_CH3;
     *(p+5)  = data[2];
     *(p+6)  = SPI_WRITE_PWM_CH4;
     *(p+7)  = data[3];
     *(p+8)  = SPI_WRITE_PWM_CH5;
     *(p+9)  = data[4];
     *(p+10) = SPI_WRITE_PWM_CH6;
     *(p+11) = data[5];
    // send read command to read back the data for checking
     *(p+12) = SPI_READ_PWM_CH1;
     *(p+13) = 0;
     *(p+14) = SPI_READ_PWM_CH2;
     *(p+15) = 0;
     *(p+16) = SPI_READ_PWM_CH3;
     *(p+17) = 0;
     *(p+18) = SPI_READ_PWM_CH4;
     *(p+19) = 0;
     *(p+20) = SPI_READ_PWM_CH5;
     *(p+21) = 0;
     *(p+22) = SPI_READ_PWM_CH6;
     *(p+23) = 0;
 	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_tr);
	if (ret < 1){
		print_err("can't send pwm data\n");
	    return -1;
	}
	// compare the data read back with the original data
	for(i=1;i<12;i=i+2){
		//print_debug("%4x --- %4x \n",p[i],d[i+12]);
       if(p[i] != d[i+12]){
#ifdef debug
    	   print_err("pwm write error \n");
#else
    	   fault_status_return(PWM_WRITE_FAILED);
#endif
    	   return PWM_WRITE_FAILED;
       }
	}
	// check success ,write pwm data load command
    ret = spi_write_one_word_no_check(SPI_WRITE_PWM_LOAD,0);
    return ret;
}

/* read_spi_rc_data()
 * read remote controller data captured by FPGA
 * return   -1 send command failed
 *          >0 number of bytes sent
 */
int read_rc_data(uint16 *data)
{
	int ret;

	uint16 *p;
	uint16 *d;

    spi_tr.len =24;

	p=(uint16*)spi_tx;
	d=(uint16*)spi_rx;
     *p      = SPI_READ_RC_CH1;
     *(p+1)  = 0;
     *(p+2)  = SPI_READ_RC_CH2;
     *(p+3)  = 0;
     *(p+4)  = SPI_READ_RC_CH3;
     *(p+5)  = 0;
     *(p+6)  = SPI_READ_RC_CH4;
     *(p+7)  = 0;
     *(p+8)  = SPI_READ_RC_CH5;
     *(p+9)  = 0;
     *(p+10) = SPI_READ_RC_CH6;
     *(p+11) = 0;
     *(p+12) = SPI_READ_RC_PERIOD;
     *(p+13) = 0;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_tr);
	if (ret < 1){
		print_err("can't send rc read command \n");
		ret = -1;
	}
	else{
       *data     = *(d+1);  //rc channel1 data
       *(data+1) = *(d+3);  //rc channel2 data
       *(data+2) = *(d+5);  //rc channel3 data
       *(data+3) = *(d+7);  //rc channel4 data
       *(data+4) = *(d+9);  //rc channel5 data
       *(data+5) = *(d+11); //rc channel6 data
       *(data+6) = *(d+13); //rc period data
	}
	return ret;
}
void joystick_execute(int *pwm)
{
	int i = 0;
	uint16 base_addr = 0x11;
	for (i = 0; i < 8; i++) {
		print_debug("0x%04x ", pwm[i]);
		//spi_write(base_addr + i, pwm[i]);//???
	}
	print_debug("\n");
}
uint16 get_board_id()
{
   return spi_read_one_word(SPI_READ_BOARD_ID);
}

uint32 get_fpga_version()
{
	uint16 low;
	uint16 high;
   low=spi_read_one_word(SPI_READ_VERSION_LOW);
   high=spi_read_one_word(SPI_READ_VERSION_HIGH);
   return (high<<16) | low;
}

float get_sonar_data()
{
  uint16 raw_data_58us;
  float  data_cm;
  raw_data_58us=spi_read_one_word(SPI_READ_SONAR_DATA);
  data_cm = ((float)raw_data_58us)/58;// the raw data is 1cm per 58us;
  return data_cm;

}
