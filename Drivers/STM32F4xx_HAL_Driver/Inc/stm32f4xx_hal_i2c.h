/*
 * stm32f4xx_hal_i2c.h
 */
#include "stm32f401xe.h"
#include "stm32f4xx_hal.h"

/*
 * @ I2C states for interrupt
 */
typedef enum {
	I2C_INIT,
	I2C_READY,
	I2C_RX_BUSY,
	I2C_TX_BUSY,
	I2C_ERROR
} I2C_State;

typedef enum {
	I2C_ERROR_AF,
	I2C_ERROR_OVR,
	I2C_ERROR_PECERR,
	I2C_ERROR_TIMEOUT,
	I2C_ERROR_BERR,
	I2C_TX_DONE,
	I2C_RX_DONE
} I2C_ErrorEvents;


typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
	I2C_TypeDef *pI2Cx;
	I2C_Config_t I2C_Config;
	I2C_State I2C_State;
	uint8_t *txBuffer;
	uint8_t *pRxBuffer;
	uint8_t rxStartIndex;
	uint8_t rxBufferSize;
	uint8_t txBufferLength;
	uint8_t rxBufferLength;
}I2C_Handle_t;

HAL_StatusTypeDef WaitTillTimeout (uint8_t timeout);
void I2C_PeripheralClkControl(I2C_TypeDef *pI2Cx);
void I2C_Init(I2C_Handle_t *I2C_handle);
//void GenerateStartCondition(I2C_Handle_t *I2C_handle);
void I2C_MasterSendData(I2C_Handle_t *I2C_handle);
HAL_StatusTypeDef HAL_I2C_Master_Transmit (I2C_Handle_t *I2C_handle, uint8_t *data, uint8_t size);
void HAL_I2C_Master_Receive (I2C_Handle_t *I2C_handle, uint8_t *rxBuffer, uint8_t size, uint8_t startIndex);

// I2C interrupt headers
I2C_State HAL_MasterTransmitInterrupt(void);
I2C_State HAL_MasterReceiveInterrupt(void);
I2C_State HAL_I2C_StartInterrupt(I2C_State expectedState);
void I2C_TXE_Interrupt(void);
void I2C_RXNE_Interrupt(void);
void StopTransmission (void);



/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 				100000
#define I2C_SCL_SPEED_FM				400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE 					1
#define I2C_ACK_DISABLE					0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       			0
#define I2C_FM_DUTY_16_9    			1

#define MCP9808_ADDR 					0x18
#define MCP9808_REG_AMBIENT_TEMP_REG	0x5
#define BYTES_PER_TRANSACTION			2

#define FLAG_SET						1
#define FLAG_NOT_SET					0

#define WRITE 							0
#define READ 							1



