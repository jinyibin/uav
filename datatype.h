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
#define PI 3.1415926

#define debug
#define TRUE 1
#define FAULSE 0
//#define HELI
#define MULTIROTOR_8
//#define MULTIROTOR_6
//#define ATTITUDE_POSITION_SEPERATE


uint32 command;
uint32 frequency;

uint32  debug_enable;
uint32 test_airline;

#endif

