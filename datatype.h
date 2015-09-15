#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef _DATA_TYPE_H
#define _DATA_TYPE_H

typedef signed char int8;
typedef unsigned char uint8;
typedef signed short int16;
typedef unsigned short uint16;
typedef signed int int32;
typedef unsigned int uint32;
typedef signed long long int64;
typedef unsigned long long uint64;

#define print_err printf
#define print_debug printf

#define debug


/*  err tag */
#define INVALID_CMD 30
#define CMD_TYPE_MISMATCH 31
#define UNSUPPORTED_CMD 32
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

#endif

