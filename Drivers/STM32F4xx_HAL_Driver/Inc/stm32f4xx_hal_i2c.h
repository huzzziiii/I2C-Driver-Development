/*
 * stm32f4xx_hal_i2c.h
 */
#include "stm32f401xe.h"

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint16_t I2C_FMDutyCycle;

} I2C_Config_t;

typedef struct {
	I2C_TypeDef *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;

void I2C_PeripheralClkControl(I2C_TypeDef *pI2Cx);
void I2C_Init(I2C_Handle_t *I2C_handle);
void GenerateStartCondition(I2C_Handle_t *I2C_handle);
void I2C_MasterSendData(I2C_Handle_t *I2C_handle);

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM	400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE 		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

#define FLAG_SET		1
#define FLAG_NOT_SET	0



