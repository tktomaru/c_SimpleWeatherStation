#ifndef CalcCRC_h
#define CalcCRC_h

#include "Arduino.h"

unsigned short calcCRC16(unsigned short crc, unsigned char *ptr, unsigned int len);
uint32_t calcCRC32(const uint8_t *data, size_t length);

#endif
