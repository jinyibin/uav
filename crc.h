#ifndef CRC_H
#define CRC_H
#include "datatype.h"
unsigned short crc_checksum16(unsigned char *message, unsigned int len);
unsigned short sbg_crc_check(const void *pFrame, uint16 bufferSize);
#endif
