/*
 * stm32f4xx_hal_i2c.c
 */
////#include "stm32f4xx_hal_i2c.h"
//#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// static functions' declarations
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx);
static void I2C_WriteSlaveAddress(I2C_Handle_t *I2C_handle, uint8_t operation);
static void I2C_ControlAcking(I2C_TypeDef *pI2Cx, uint8_t enable);
static HAL_StatusTypeDef WaitTillTimeout (uint8_t timeout);

void I2C_PeripheralClkControl(I2C_TypeDef *pI2Cx) {
	if (pI2Cx == I2C1) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}

	else if (pI2Cx == I2C2) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}

	else if (pI2Cx == I2C3) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	}
}

void I2C_Init(I2C_Handle_t *I2C_handle) {

	I2C_PeripheralClkControl(I2C_handle->pI2Cx);

	// control ACK bit
	I2C_handle->pI2Cx->CR1 |= I2C_handle->I2C_Config.I2C_AckControl << 10;

	// device address
	I2C_handle->pI2Cx->OAR1 |= I2C_handle->I2C_Config.I2C_DeviceAddress << 1;
	I2C_handle->pI2Cx->OAR1 |= 1 << 14; 		// setting 14th bit to 1 as per datasheet

	// peripheral clock frequency
	uint32_t peripheralClkFreq = (SystemCoreClock / 1000000) & 0x3f;
	I2C_handle->pI2Cx->CR2 |= peripheralClkFreq;

	//setting CCR register
	if (I2C_handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		uint16_t ccr = SystemCoreClock / (2 * I2C_handle->I2C_Config.I2C_SCLSpeed);
			//uint16_t ccr = t_high / t_pclk;
		I2C_handle->pI2Cx->CCR |= ccr;
	}

	// TRise configuration
	if(I2C_handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		I2C_handle->pI2Cx->TRISE |= ( (SystemCoreClock / 1000000 + 1) & 0x3f); // max rise time in SmMode = 1000ns - 0011 1111
	}

	// control peripheral clk and PE
	I2C_handle->pI2Cx->CR1 |= ENABLE;
}

uint8_t GetFlagStatus(I2C_TypeDef *pI2Cx, uint16_t flag) {
	if (pI2Cx->SR1 & flag)
		return FLAG_SET;
	return FLAG_NOT_SET;
}

void GenerateStartCondition(I2C_Handle_t *I2C_handle) {
	I2C_handle->pI2Cx->CR1 |= I2C_CR1_START;
}

static void GenerateStopCondition(I2C_Handle_t *I2C_handle) {
	I2C_handle->pI2Cx->CR1 |= I2C_CR1_STOP;
}

static void I2C_WriteSlaveAddress(I2C_Handle_t *I2C_handle, uint8_t operation) {
	uint8_t slaveAddress = I2C_handle->I2C_Config.I2C_DeviceAddress;
	slaveAddress <<= 1;
	slaveAddress = operation == WRITE ? (slaveAddress & ~1) : (slaveAddress | 1);

	I2C_handle->pI2Cx->DR = slaveAddress;
}

static void I2C_WriteToDevice(I2C_Handle_t *I2C_handle, uint8_t data, uint8_t operation) {
	data <<= 1;
	data = operation == WRITE ? (data & ~1) : (data | 1);
	I2C_handle->pI2Cx->DR = data;
}

// read SR1 and SR2 registers to clear ADDR flag
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx) {
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
}
static void I2C_ControlAcking(I2C_TypeDef *pI2Cx, uint8_t enable)
{
//	pI2Cx->CR1 = enable ? (pI2Cx->CR1 | (1 << I2C_CR1_ACK)) : (pI2Cx->CR1 & ~(1 << I2C_CR1_ACK));
	if (enable)
	{
		pI2Cx->CR1 |= I2C_CR1_ACK;
	}
	else
	{
		pI2Cx->CR1 &= ~(I2C_CR1_ACK);
	}
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit (I2C_Handle_t *I2C_handle, uint8_t *data, uint8_t size)
{
	// generate start condition
	GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB) && WaitTillTimeout(5));

	// write slave address along with write bit
	I2C_WriteSlaveAddress(I2C_handle, WRITE);

	// wait for address to be sent
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR) && WaitTillTimeout(5));

	// clear address flag
	I2C_ClearADDRFlag(I2C_handle->pI2Cx);

	/* write to the SDA line */

	// wait for TXE bit to set
//	while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_TXE) && WaitTillTimeout(5)); // todo remove!

	for (; size > 0; size--)
	{
		// making sure data register is empty prior to writing to it
		while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_TXE) && WaitTillTimeout(5));

		I2C_handle->pI2Cx->DR = *data++;

		while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) && WaitTillTimeout(5));

//		if (GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) == SET)
//		{
//			I2C_handle->pI2Cx->DR = *data++;
//			--size;
//		}
//
//		if (GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) == RESET) {
//			return HAL_ERROR;
//		}
	}

	GenerateStopCondition(I2C_handle);

	return HAL_OK;
}

void HAL_I2C_Master_Receive (I2C_Handle_t *I2C_handle, uint8_t *rxBuffer, uint8_t size, uint8_t startIndex)
{
	// generate start condition
	GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB));

	// write slave address
	I2C_WriteSlaveAddress(I2C_handle, READ);

	// wait for address to be sent
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR) && WaitTillTimeout(5));

	switch (size) {
		case 1:
			I2C_ControlAcking(I2C_handle->pI2Cx, RESET);	// disable ACK
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);			// clear ADDR flag
			GenerateStopCondition(I2C_handle);				// generate STOP condition
			break;

		case 2:
			I2C_ControlAcking(I2C_handle->pI2Cx, RESET);	// disable ACK
			I2C_handle->pI2Cx->CR1 |= 1 << 11;				// set POS
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);			// clear ADDR flag
			break;

		default:
			I2C_ControlAcking(I2C_handle->pI2Cx, SET);		// Enable ACK
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);			// clear ADDR flag

	}

	while (size > 0) {
		if (size <= 3) {
			if (size == 1) {
					// disable ACK
					I2C_ControlAcking(I2C_handle->pI2Cx, RESET);

					// clear the ADDR flag
					I2C_ClearADDRFlag(I2C_handle->pI2Cx);

					// wait till RXNE = 1 (Data is sent from SR to DR)
					while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_RXNE));

					// generate stop
					GenerateStopCondition(I2C_handle);

					// read data
					rxBuffer = I2C_handle->pI2Cx->DR;

			}

			else if (size == 2) {
				// wait till BTF is set (last byte is received) - shift reg=1, DR=1
				while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) && WaitTillTimeout(5));

				GenerateStopCondition(I2C_handle);
				printf ("Start_index: %d\n", startIndex);

				rxBuffer[startIndex++] = (uint8_t) I2C_handle->pI2Cx->DR;
				--size;

				rxBuffer[startIndex] = (uint8_t) I2C_handle->pI2Cx->DR;
				--size;
			}
			else if (size == 3) {
				// wait for the second last byte to be put in SR while DR is full (RxNE=1)
				while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) && WaitTillTimeout(5));

				// disable ACK so NACK is sent upon reception of the last byte
				I2C_ControlAcking(I2C_handle->pI2Cx, DISABLE);

				// read the first byte
				*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
				rxBuffer++;

				size--;

				// wait for the last byte to be put in SR while DR is full (RxNE=1)
				while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) && WaitTillTimeout(5));

				GenerateStopCondition(I2C_handle);

				// read the second byte
				*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
				rxBuffer++;

				size--;

				// read the third byte
				*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
				rxBuffer++;

				size--;

			}

		}
		// > 3 bytes
		else {
			while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_RXNE) && WaitTillTimeout(5));

			// reading the byte
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;

			size--;

		}
	}
}

static HAL_StatusTypeDef WaitTillTimeout (uint8_t timeout)
{
	uint8_t prevTicks = HAL_GetTick(); // current ticks in ms
	while ((HAL_GetTick() - prevTicks) < timeout);
	return HAL_OK;
}





