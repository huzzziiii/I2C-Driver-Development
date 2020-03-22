
#include <i2c.h>

int ProcessData (uint8_t *rxBuffer, uint8_t startIndex) {
	printf ("\nUpper byte: %d, Lower byte: %d\n", rxBuffer[startIndex], rxBuffer[startIndex+1]);
	uint16_t temperature;

	// process data
	uint8_t upperByte = rxBuffer[startIndex] & 0x1F; // mask out the 3 bits
	uint8_t signBit = upperByte & 0x10;

	if (signBit)
	{
		upperByte = upperByte & 0xF; 	// clear out the sign bit
		temperature = 256 - (upperByte << 4 | rxBuffer[startIndex+1] >> 4);
	}
	else
	{
		temperature = upperByte << 4 | rxBuffer[startIndex+1] >> 4;
	}
	printf ("Temperature value: %d\n\n", temperature);
}

void ReadTemperature(I2C_Handle_t *I2C_handle, const uint8_t bytesToRead)
{
	uint8_t txBuffer[1] = {MCP9808_REG_AMBIENT_TEMP_REG};
	uint8_t rxBuffer[bytesToRead];
	uint8_t startRxIndex = 0;

	uint8_t txSize = sizeof(txBuffer)/sizeof(txBuffer[0]);

	// specify the register address where temperature values will be read from
	HAL_I2C_Master_Transmit(I2C_handle, txBuffer, txSize);

	// request the data from the sensor
	for (int i = 0; i < bytesToRead/2; i++, startRxIndex+=2) {
		HAL_I2C_Master_Receive (I2C_handle, rxBuffer, BYTES_PER_TRANSACTION, startRxIndex);
		for (int j=0; j<bytesToRead; j++) printf ("%d\n", rxBuffer[j]);
	}

	printf ("Printing raw bytes:\n");
	for (int i = 0; i < bytesToRead; i+=2) {
		printf ("%d,%d\n", rxBuffer[i], rxBuffer[i+1]);
		ProcessData(rxBuffer, i);
	}

//	ProcessData(rxBuffer, 0);

}
