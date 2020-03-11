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
		pI2Cx->CR1 |= 1U << I2C_CR1_ACK;
	}
	else
	{
		pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK);
	}
}
HAL_StatusTypeDef I2C_MasterRequestWrite (I2C_Handle_t *I2C_handle, uint8_t *data, uint8_t size)
{
	// generate start condition
	GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB));

	// write slave address along with write bit
	I2C_WriteSlaveAddress(I2C_handle, WRITE);

	// wait for address to be sent
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR));

	// clear address flag
	I2C_ClearADDRFlag(I2C_handle->pI2Cx);

	/* write to the SDA lie */

	// wait for TXE bit to set
	while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_TXE));

	for (; size > 0; size--)
	{
		// making sure data register is not empty prior to reading from it
		while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_TXE));

		I2C_handle->pI2Cx->DR = *data++;

		while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF));

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


	while(! GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_TXE) );


	while(! GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF) );

	GenerateStopCondition(I2C_handle);

	return HAL_OK;
}

HAL_StatusTypeDef I2C_MasterTransmitData (I2C_Handle_t *I2C_handle, uint8_t* data, uint8_t size)
{
//	if (I2C_MasterRequestWrite(I2C_handle, data, size) != HAL_OK)
//	{
//		return HAL_ERROR;
//	}

	I2C_MasterRequestWrite(I2C_handle, data, size);

}

void I2C_MasterReceiveData (I2C_Handle_t *I2C_handle, uint8_t *rxBuffer, uint8_t size)
{
//	uint8_t temperatureRegister[1] = {0x5};
//	uint8_t regSize = sizeof(temperatureRegister)/sizeof(temperatureRegister[0]);
//
//	I2C_MasterTransmitData(I2C_handle, temperatureRegister, regSize);

	// -----------
	// generate start condition
	GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB));

	// write slave address
	I2C_WriteSlaveAddress(I2C_handle, READ);

	// wait for address to be sent
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR));

	if (size > 1) {

//		I2C_ClearADDRFlag(I2C_handle->pI2Cx);

		if (size == 2) {
			I2C_ControlAcking(I2C_handle->pI2Cx, RESET);
			I2C_handle->pI2Cx->CR1 |= 1 << 11;				//POS
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);

			while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF));
			GenerateStopCondition(I2C_handle);
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
		}
		else if (size == 3) {
			I2C_ControlAcking(I2C_handle->pI2Cx, SET);
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);

			while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF));

			// disable ACK so NACK is sent upon reception of the last byte
			I2C_ControlAcking(I2C_handle->pI2Cx, DISABLE);

			// read the first byte
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;

			size--;

			while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF));
			GenerateStopCondition(I2C_handle);

			// read the second byte
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;

			size--;

			// read the third byte
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;

			size--;
//			break;

		}

	}

//	if (size > 1) {
//
//		I2C_ClearADDRFlag(I2C_handle->pI2Cx);
//
////		for (uint8_t i = size; i > 0; i--)
////		{
//			if (size == 2) {
//				I2C_ControlAcking(I2C_handle->pI2Cx, RESET);
//				I2C_handle->pI2Cx->CR1 |= 1 << 11;		//POS
//				I2C_ClearADDRFlag(I2C_handle->pI2Cx);
//				while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_BTF));
//				GenerateStopCondition(I2C_handle);
//				rxBuffer = I2C_handle->pI2Cx->DR;
//				rxBuffer++;
//				rxBuffer = I2C_handle->pI2Cx->DR;
//			}
////			// wait till RXNE = 1
////			while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_RXNE));
////
////			rxBuffer = (uint8_t*) I2C_handle->pI2Cx->DR;
////			rxBuffer++;
////
////			if (size == 2) {
////				I2C_ControlAcking(I2C_handle->pI2Cx, RESET);	// disable ACK
////				GenerateStopCondition(I2C_handle);				// generate stop condition
////			}
//
////		}
//	}

	else if (size == 1) {
		// disable ACK
		I2C_ControlAcking(I2C_handle->pI2Cx, RESET);

		// clear the ADDR flag
		I2C_ClearADDRFlag(I2C_handle->pI2Cx);

		// wait till RXNE = 1
		while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_RXNE));

		// generate stop
		GenerateStopCondition(I2C_handle);

		// read data
		rxBuffer = I2C_handle->pI2Cx->DR;

	}


}





void I2C_MasterSendData(I2C_Handle_t *I2C_handle) {
//	// generate the start condition
//	GenerateStartCondition(I2C_handle);
//
//	// validate the completion of start condition
//	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB));
//
//	// write slave address
//	I2C_WriteSlaveAddress(I2C_handle);
//
//	// verify address phase is completed
//	while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR));
//
//	// clear ADDR flag
//	I2C_ClearADDRFlag(I2C_handle->pI2Cx);
}


