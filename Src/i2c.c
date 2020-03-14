
#include <i2c.h>

void ReadTemperature(I2C_Handle_t *I2C_handle)
{
	uint8_t txBuffer[1] = {MCP9808_REG_AMBIENT_TEMP_REG};
	uint8_t rxBuffer[2] = {0};
	uint16_t temperature = 0;

	uint8_t txSize = sizeof(txBuffer)/sizeof(txBuffer[0]);
	uint8_t rxSize = sizeof(rxBuffer)/sizeof(rxBuffer[0]);

	// specify the register address where temperature values will be read from
	HAL_I2C_Master_Transmit(I2C_handle, txBuffer, txSize);

	// request the data from the sensor
	HAL_I2C_Master_Receive (I2C_handle, rxBuffer, rxSize);

	// printing raw bytes
	for (int i = 0; i < rxSize; i++) {
		printf ("%d\n", rxBuffer[i]);
	}

	// process data
	uint8_t upperByte = rxBuffer[0] & 0x1F; // mask out the 3 bits
	uint8_t signBit = upperByte & 0x10;

	if (signBit)
	{
		upperByte = upperByte & 0xF; 	// clear out the sign bit
		temperature = 256 - (upperByte << 4 | rxBuffer[1] >> 4);
	}
	else
	{
		temperature = upperByte << 4 | rxBuffer[1] >> 4;
	}

	printf ("Temperature value: %d\n", temperature);
}
