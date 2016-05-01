#include "i2chelp.h"

void writeCommand(uint8_t address, uint8_t command)
{
	Wire.beginTransmission(address);	// initialize the Tx buffer
	Wire.write(command);				// put command in Tx buffer
	Wire.endTransmission();				// send the Tx buffer
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);	// initialize the Tx buffer
	Wire.write(subAddress);				// put slave register address in Tx buffer
	Wire.write(data);					// put data in Tx buffer
	Wire.endTransmission();				// send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	Wire.beginTransmission(address);		// initialize the Tx buffer
	Wire.write(subAddress);					// put slave register address in Tx buffer
	Wire.endTransmission(false);			// send the Tx buffer, but send a restart to keep connection alive

	Wire.requestFrom(address, (uint8_t) 1);	// read one byte from slave register address 

	return Wire.read();						// fill Rx buffer with result
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);	// initialize the Tx buffer
	Wire.write(subAddress);				// put slave register address in Tx buffer
	Wire.endTransmission(false);		// send the Tx buffer, but send a restart to keep connection alive

	Wire.requestFrom(address, count);	// read bytes from slave register address 

	uint8_t i = 0;

	while (Wire.available())
	{
		dest[i++] = Wire.read();		// put read results in the Rx buffer
	}         
}
