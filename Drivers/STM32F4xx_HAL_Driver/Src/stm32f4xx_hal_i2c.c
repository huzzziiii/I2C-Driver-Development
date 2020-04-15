/*
 * stm32f4xx_hal_i2c.c
 */
#include "stm32f4xx_hal_i2c.h"

/* Private function prototypes -----------------------------------------------*/
static void I2C_GenerateStartCondition(volatile I2C_Handle_t *I2C_handle);
static void I2C_GenerateStopCondition(volatile I2C_Handle_t *I2C_handle);
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx);
static void I2C_WriteSlaveAddress(volatile I2C_Handle_t *I2C_handle, uint8_t operation);
static void I2C_ControlAcking(I2C_TypeDef *pI2Cx, uint8_t enable);
//static HAL_StatusTypeDef WaitTillTimeout (uint8_t timeout);
static void I2C_SetCtrlBits(void);
static void I2C_StopTransmission(void);
static void I2C_WaitForCompletion(I2C_TypeDef *pI2Cx, uint16_t i2cRegister);
volatile static I2C_Handle_t *I2C_handle_p = NULL;		// static pointer to the struct that's only used by the I2C driver
// **********************************************************************************************

/* Function definitions -----------------------------------------------------*/

/*
 * @I2C_PeripheralClkControl: Enables the peripheral clock for a respective I2C interface
 */
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

/*
 * @I2C_Init: Populates I2C struct
 */
void I2C_Init(I2C_Handle_t *I2C_handle) {

	// initializing static pointer
	I2C_handle_p = I2C_handle;

	// enable I2C clock in RCC register
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
	{
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

void I2C_GenerateStartCondition(volatile I2C_Handle_t *I2C_handle) {
	I2C_handle->pI2Cx->CR1 |= I2C_CR1_START;
}

static void I2C_GenerateStopCondition(volatile I2C_Handle_t *I2C_handle) {
//	printf ("STOP condition...\n");
	I2C_handle->pI2Cx->CR1 |= I2C_CR1_STOP;
}

static void I2C_WaitForCompletion(I2C_TypeDef *pI2Cx, uint16_t i2cRegister)
{
	while (!GetFlagStatus(pI2Cx, i2cRegister) && WaitTillTimeout(5));
}

/*
 * @I2C_WriteSlaveAddress: Writes slave address to DR
 */
static void I2C_WriteSlaveAddress(volatile I2C_Handle_t *I2C_handle, uint8_t operation) {
//	printf ("Writing slave address: %d\n", I2C_handle->I2C_Config.I2C_DeviceAddress);
	uint8_t slaveAddress = I2C_handle->I2C_Config.I2C_DeviceAddress;
	slaveAddress <<= 1;
	slaveAddress = operation == WRITE ? (slaveAddress & ~1) : (slaveAddress | 1);

	I2C_handle->pI2Cx->DR = slaveAddress;
}

/*
 * @I2C_ClearADDRFlag: Reads SR1 and SR2 registers to clear ADDR flag
 * During RXing: when 2 bytes are to be read, reset the ACK and set the POS bit
 */
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx) {
	uint32_t dummyRead;

	if (I2C_handle_p->I2C_State == I2C_RX_BUSY)
	{
		if (I2C_handle_p->rxBufferLength == 2)
		{
			I2C_ControlAcking(I2C_handle_p->pI2Cx, RESET);
			I2C_handle_p->pI2Cx->CR1 |= 1 << I2C_CR1_POS_Pos;
		}
	}

	dummyRead = pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
}

/*
 * @I2C_ControlAcking: enables/disables the ACK bit for I2C
 */
static void I2C_ControlAcking(I2C_TypeDef *pI2Cx, uint8_t enable)
{
	if (enable)
	{
		pI2Cx->CR1 |= I2C_CR1_ACK;
	}
	else
	{
		pI2Cx->CR1 &= ~(I2C_CR1_ACK);
	}
}

/*
 * @HAL_I2C_StartInterrupt: Generates the START condition and enables I2C control bits
 * usage: called to "enable" I2C transaction via interrupts
 */
I2C_State HAL_I2C_StartInterrupt(I2C_State expectedState)
{
	char *command = expectedState == I2C_TX_BUSY ? "TXing...\n" : "RXing...\n";

	if (I2C_handle_p->I2C_State == I2C_INIT)
	{
		printf ("%s", command);

		// set transaction state
		I2C_handle_p->I2C_State = expectedState;

		I2C_GenerateStartCondition(I2C_handle_p);

		// enable i2c control bits
		I2C_SetCtrlBits();
	}
	return I2C_handle_p->I2C_State;
}


/*
 * I2C1_EV_IRQHandler: Interrupt handler for I2C
 */
void I2C1_EV_IRQHandler (void)
{
	uint8_t eventInterrupt = (I2C_handle_p->pI2Cx->CR2 & I2C_CR2_ITEVTEN) >> I2C_CR2_ITEVTEN_Pos;
	uint8_t bufferInterrupt = (I2C_handle_p->pI2Cx->CR2 & I2C_CR2_ITBUFEN) >> I2C_CR2_ITBUFEN_Pos;
	uint8_t temp;			// stores register values

	if (eventInterrupt)
	{
		//	validate the completion of START condition
		temp =  (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_SB) >> I2C_SR1_SB_Pos;
		if (temp)
		{
			if (I2C_handle_p->I2C_State == I2C_TX_BUSY)
			{
				I2C_WriteSlaveAddress(I2C_handle_p, WRITE);		// write slave address along with write bit
			}
			else if (I2C_handle_p->I2C_State == I2C_RX_BUSY)
			{
				I2C_WriteSlaveAddress(I2C_handle_p, READ);		// write slave address along with read bit
			}
		}

		// ADDR
		temp = (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_ADDR) >> I2C_SR1_ADDR_Pos;
		if (temp)
		{
			I2C_ClearADDRFlag(I2C_handle_p->pI2Cx);				// clear address flag
		}

		// TXE, RXNE
		if (bufferInterrupt)
		{
			// TXing
			temp = (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_TXE) >> I2C_SR1_TXE_Pos;

			if (temp && I2C_handle_p->I2C_State == I2C_TX_BUSY)
			{
				I2C_TXE_Interrupt();
			}

			// RXing
			temp = (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_RXNE) >> I2C_SR1_RXNE_Pos;

			if (temp && I2C_handle_p->I2C_State == I2C_RX_BUSY)
			{
//				I2C_RXNE_Interrupt();
			}
		}

		//BTF
		temp = (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_BTF) >> I2C_SR1_BTF_Pos;
		if (temp)
		{
			if (I2C_handle_p->I2C_State == I2C_TX_BUSY)					// TXE=1, BTF=1
			{
				if (!I2C_handle_p->txBufferLength)						// if there are no more TX bytes to be sent
				{
					I2C_GenerateStopCondition(I2C_handle_p);
					I2C_StopTransmission();
				}
			}
			else if (I2C_handle_p->I2C_State == I2C_RX_BUSY)			// RXNE=1, BTF=1, LEN=0 --> STOP
			{
				if (I2C_handle_p->rxBufferLength == 2)
				{
					I2C_GenerateStopCondition(I2C_handle_p);

					I2C_handle_p->pRxBuffer[I2C_handle_p->rxStartIndex++] = (uint8_t) I2C_handle_p->pI2Cx->DR; // read second last byte
					I2C_handle_p->rxBufferLength--;

					I2C_handle_p->pRxBuffer[I2C_handle_p->rxStartIndex++] = (uint8_t) I2C_handle_p->pI2Cx->DR; // read last byte
					I2C_handle_p->rxBufferLength--;

					I2C_StopTransmission();
				}
			}
		}
	}
}

/*
 * @I2C_HandleInterruptEvents: Called upon getting an error interrupt - resets the bit and generate stop condition
 */
void I2C_HandleInterruptEvents (uint16_t errorRegister, I2C_ErrorEvents errorEvent)
{
	I2C_handle_p->pI2Cx->SR1 &= ~(errorRegister);

	if (errorEvent == I2C_ERROR_AF)
	{
		printf ("Received ACK failure...\n");
		I2C_StopTransmission();
		I2C_GenerateStopCondition(I2C_handle_p);
	}
}

/*
 * @I2C1_ER_IRQHandler: Interrupt handler for I2C errors
 */
void I2C1_ER_IRQHandler(void)
{
	printf ("ERROR IRQ handler...\n");
	I2C_handle_p->I2C_State = I2C_ERROR;

	uint8_t errorInterrupt = (I2C_handle_p->pI2Cx->CR2 & I2C_CR2_ITERREN) >> I2C_CR2_ITERREN_Pos;
	if (errorInterrupt)
	{
		if (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_AF)
		{
			I2C_HandleInterruptEvents(I2C_SR1_AF, I2C_ERROR_AF);
		}
		else if (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_OVR)
		{
			I2C_HandleInterruptEvents(I2C_SR1_AF, I2C_ERROR_OVR);
		}
		else if (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_PECERR)
		{
			I2C_HandleInterruptEvents(I2C_SR1_AF, I2C_ERROR_PECERR);
		}
		else if (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_TIMEOUT)
		{
			I2C_HandleInterruptEvents(I2C_SR1_AF, I2C_ERROR_TIMEOUT);
		}
		else if (I2C_handle_p->pI2Cx->SR1 & I2C_SR1_BERR)
		{
			I2C_HandleInterruptEvents(I2C_SR1_AF, I2C_ERROR_BERR);
		}
	}
}

/*
 * @I2C_TXE_Interrupt: Writes the respective byte to the DR
 * data register = empty = TXE
 */
void I2C_TXE_Interrupt (void)
{
	if (I2C_handle_p->txBufferLength)
	{
//		printf ("Writing TX data: %d\n", *I2C_handle_p->txBuffer);
		I2C_handle_p->pI2Cx->DR = (*I2C_handle_p->txBuffer)++;
		I2C_handle_p->txBufferLength--;
	}
}


//void I2C_RXNE_Interrupt() {
	//	if (I2C_handle_p->rxBufferLength > 0)
	//	{
	//		I2C_handle_p->pRxBuffer[I2C_handle_p->rxStartIndex++] = (uint8_t) I2C_handle_p->pI2Cx->DR;
	//		I2C_handle_p->rxBufferLength--;
	//	}

	//	if (!I2C_handle_p->rxBufferLength)				   // no more bytes to read
	//	{
	//		I2C_GenerateStopCondition(I2C_handle_p);
	//		I2C_StopTransmission();
	//	}



	//	// handle RXing
	//	if (I2C_handle_p->rxBufferLength)
	//	{
	//		if (I2C_handle_p->rxBufferLength == 2)
	//		{
	//			I2C_ControlAcking(I2C_handle_p->pI2Cx, RESET);		// Enable NACK for the last byte to be RX'd
	//			I2C_handle_p->pI2Cx->CR1 |= 1 << I2C_CR1_POS_Pos;
	//		}
	//		I2C_handle_p->pRxBuffer[I2C_handle_p->rxStartIndex++] = (uint8_t) I2C_handle_p->pI2Cx->DR;
	////		*I2C_handle_p->pRxBuffer++ = I2C_handle_p->pI2Cx->DR;
	//		I2C_handle_p->rxBufferLength--;
	//	}
	//
	//	if (!I2C_handle_p->rxBufferLength)						// no more bytes to read
	//	{
	//		I2C_GenerateStopCondition(I2C_handle_p);
	//		I2C_StopTransmission();
	//	}

//}


/*
 * HAL_I2C_Master_Transmit: polling approach for TXing bytes to the slave
 */
HAL_StatusTypeDef HAL_I2C_Master_Transmit (I2C_Handle_t *I2C_handle, uint8_t *data, uint8_t size)
{
	// generate start condition
	I2C_GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_SB);

	// write slave address along with write bit
	I2C_WriteSlaveAddress(I2C_handle, WRITE);

	// wait for address to be sent
	I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_ADDR);

	// clear address flag
	I2C_ClearADDRFlag(I2C_handle->pI2Cx);

	// write to the SDA line
	for (; size > 0; size--)
	{
		// making sure data register is empty prior to writing to it
		I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_TXE);

		I2C_handle->pI2Cx->DR = *data++;

		I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_BTF);
	}
	I2C_GenerateStopCondition(I2C_handle);

	return HAL_OK;
}

/*
 * @HAL_I2C_Master_Receive: Polling approach for RXing bytes from slave
 */
void HAL_I2C_Master_Receive (I2C_Handle_t *I2C_handle, uint8_t *rxBuffer, uint8_t size, uint8_t startIndex)
{
	// generate start condition
	I2C_GenerateStartCondition(I2C_handle);

	// validate the completion of start condition
	I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_SB);

	// write slave address
	I2C_WriteSlaveAddress(I2C_handle, READ);

	// wait for address to be sent
	I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_ADDR);

	switch (size) {
		case 1:
			I2C_ControlAcking(I2C_handle->pI2Cx, RESET);	// disable ACK
			I2C_ClearADDRFlag(I2C_handle->pI2Cx);			// clear ADDR flag
			I2C_GenerateStopCondition(I2C_handle);				// generate STOP condition
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
					I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_RXNE);

					// generate stop
					I2C_GenerateStopCondition(I2C_handle);

					// read data
					rxBuffer[startIndex++] = I2C_handle->pI2Cx->DR;

			}

			else if (size == 2) {
				// wait till BTF is set (last byte is received) - shift reg=1, DR=1
				I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_BTF);

				I2C_GenerateStopCondition(I2C_handle);
				printf ("Start_index: %d\n", startIndex);

				rxBuffer[startIndex++] = (uint8_t) I2C_handle->pI2Cx->DR;
				--size;

				rxBuffer[startIndex] = (uint8_t) I2C_handle->pI2Cx->DR;
				--size;
			}
			else if (size == 3) {
				// wait for the second last byte to be put in SR while DR is full (RxNE=1)
				I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_BTF);

				// disable ACK so NACK is sent upon reception of the last byte
				I2C_ControlAcking(I2C_handle->pI2Cx, DISABLE);

				// read the first byte
				*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
				rxBuffer++;

				size--;

				// wait for the last byte to be put in SR while DR is full (RxNE=1)
				I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_BTF);

				I2C_GenerateStopCondition(I2C_handle);

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
			I2C_WaitForCompletion(I2C_handle->pI2Cx, I2C_SR1_RXNE);

			// reading the byte
			*rxBuffer = (uint8_t) I2C_handle->pI2Cx->DR;
			rxBuffer++;

			size--;

		}
	}
}

/*
 * @I2C_StopTransmission: Disables I2C control bits and sets I2C struct to initial values
 */
static void I2C_StopTransmission(void)
{
	printf ("Stopping transmission...\n\n");

	// disable control bits
	I2C_handle_p->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN_Pos);
	I2C_handle_p->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN_Pos);

	// restore struct
	I2C_handle_p->I2C_State = I2C_READY;
	I2C_handle_p->rxBufferLength = BYTES_PER_TRANSACTION;
}

/*
 * @I2C_SetCtrlBits: Sets I2C control bits
 */
static void I2C_SetCtrlBits(void)
{
	I2C_handle_p->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
	I2C_handle_p->pI2Cx->CR2 |= I2C_CR2_ITEVTEN;
	I2C_handle_p->pI2Cx->CR2 |= I2C_CR2_ITERREN;
}

HAL_StatusTypeDef WaitTillTimeout (uint8_t timeout)
{
	uint8_t prevTicks = HAL_GetTick(); // current ticks in ms
	while ((HAL_GetTick() - prevTicks) < timeout);
	return HAL_OK;
}





