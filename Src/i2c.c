
#include <i2c.h>

/*
 * @ProcessData: Converts raw bytes into readable data
 */
void ProcessData (uint8_t *rxBuffer, uint8_t startIndex) {
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

/*
 * @ReadTemperatureInterrupt: Reads temperature values via I2C using interrupts
 */
void ReadTemperatureInterrupt(I2C_Handle_t *I2C_handle)
{
	setRef(I2C_handle);   // todo - sets the static pointer in HAL file to I2C_handle

	// Start I2C transaction
	while (HAL_I2C_StartInterrupt(I2C_TX_BUSY) != I2C_READY);

	I2C_handle->I2C_State = I2C_INIT;

	// request the data from the sensor
	for (int i = 0; i < I2C_handle->rxBufferSize/2; i++)
	{
		I2C_handle->I2C_State = I2C_INIT;
		while (HAL_I2C_StartInterrupt(I2C_RX_BUSY) != I2C_READY);
	}

	printf ("Printing processed data...\n");
	for (int i = 0; i < I2C_handle->rxBufferSize; i+=2)
	{
		printf ("%d, %d\n", I2C_handle->pRxBuffer[i], I2C_handle->pRxBuffer[i+1]);
		ProcessData(I2C_handle->pRxBuffer, i);
	}

   	while (1);
}

/*
 * @ReadTemperature: Read temperature via I2C using polling approach
 */
void ReadTemperature(I2C_Handle_t *I2C_handle, const uint8_t bytesToRead)
{
	// todo - maybe use the following data straight from I2C_handle
	uint8_t txBuffer[1] = {MCP9808_REG_AMBIENT_TEMP_REG};
	uint8_t rxBuffer[bytesToRead];
	uint8_t startRxIndex = 0;

	uint8_t txSize = sizeof(txBuffer)/sizeof(txBuffer[0]);

	// specify the register address where temperature values will be read from
	HAL_I2C_Master_Transmit(I2C_handle, txBuffer, txSize);

	// request the data from the sensor
	for (int i = 0; i < bytesToRead/2; i++, startRxIndex+=2)
	{
		HAL_I2C_Master_Receive (I2C_handle, rxBuffer, BYTES_PER_TRANSACTION, startRxIndex);

		for (int j=0; j<bytesToRead; j++)
		{
			printf ("%d\n", rxBuffer[j]);
		}
	}

	printf ("Printing raw bytes:\n");
	for (int i = 0; i < bytesToRead; i+=2)
	{
		printf ("%d,%d\n", rxBuffer[i], rxBuffer[i+1]);
		ProcessData(rxBuffer, i);
	}

//	ProcessData(rxBuffer, 0);

}
