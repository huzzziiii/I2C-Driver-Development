/*
 * stm32f4xx_hal_i2c.c
 */
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal.h"

// static functions' declarations
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx);
static void I2C_WriteSlaveAddress(I2C_Handle_t *I2C_handle);


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
	pI2Cx->CR1 = ENABLE;
}

void I2C_Init(I2C_Handle_t *I2C_handle) {

	// control peripheral clk
	I2C_PeripheralClkControl(I2C_handle->pI2Cx);

	// control ACK bit
	I2C_handle->pI2Cx->CR1 |= I2C_handle->I2C_Config.I2C_AckControl << 10;

	// device address					   I2C_Config.I2C_DeviceAddress
	I2C_handle->pI2Cx->OAR1 |= I2C_handle->I2C_Config.I2C_DeviceAddress << 1;
	I2C_handle->pI2Cx->OAR1 |= 1 << 14; 		// setting 14th bit to 1 as per datasheet

	// peripheral clock frequency
	uint32_t peripheralClkFreq = (SystemCoreClock / 1000000) & 0x3f;
	I2C_handle->pI2Cx->CR2 |= peripheralClkFreq;

	//setting CCR register
//	uint16_t t_high = (1/I2C_handle->I2C_Config->I2C_SCLSpeed); // /2
//	uint16_t t_pclk = 1/SystemCoreClock;
//	uint16_t ccr = SystemCoreClock / (2 * I2C_handle->I2C_Config->I2C_SCLSpeed);

	if (I2C_handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		uint16_t ccr = SystemCoreClock / (2 * I2C_handle->I2C_Config.I2C_SCLSpeed);
//		uint16_t ccr = t_high / t_pclk;
		I2C_handle->pI2Cx->CCR |= ccr;
	}

	// TRise configuration
	if(I2C_handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		I2C_handle->pI2Cx->TRISE |= ( (SystemCoreClock / 1000000 + 1) & 0x3f);
	}
}

uint8_t GetFlagStatus(I2C_TypeDef *pI2Cx, uint16_t flag) {
	if (pI2Cx->SR1 & flag)
		return FLAG_SET;
	return FLAG_NOT_SET;
}

void GenerateStartCondition(I2C_Handle_t *I2C_handle) {
	I2C_handle->pI2Cx->CR1 |= I2C_CR1_START;
}

static void I2C_WriteSlaveAddress(I2C_Handle_t *I2C_handle) {
	uint8_t slaveAddress = I2C_handle->I2C_Config.I2C_DeviceAddress;
	slaveAddress <<= 1;
	slaveAddress &= ~1;
	I2C_handle->pI2Cx->DR = slaveAddress;
}

static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx) {
	// read SR1 and SR2 registers to clear ADDR flag
	volatile uint16_t dummyRead;
	dummyRead= pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
//	(void) dummyRead;
}

void I2C_MasterSendData(I2C_Handle_t *I2C_handle) {
	// generate the start condition
	GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	while (!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_SB));

	// write slave address
	I2C_WriteSlaveAddress(I2C_handle);

	// verify address phase is completed
	while(!GetFlagStatus(I2C_handle->pI2Cx, I2C_SR1_ADDR));

	// clear ADDR flag
	I2C_ClearADDRFlag(I2C_handle->pI2Cx);
}


