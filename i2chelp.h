#ifndef i2chelp_h_
#define i2chelp_h_

#include <i2c_t3.h>

void writeCommand(uint8_t address, uint8_t command);
void writeByte   (uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte (uint8_t address, uint8_t subAddress);
void readBytes   (uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

#endif
