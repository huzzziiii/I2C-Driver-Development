#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal.h"

void ReadTemperature(I2C_Handle_t *I2C_handle, const uint8_t bytesToRead);
void ReadTemperatureInterrupt(I2C_Handle_t *I2C_handle, const uint8_t bytesToRead);
