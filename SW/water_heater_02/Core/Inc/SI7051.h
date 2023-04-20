/*
 * SI7051.h
 *
 *  Created on: 12. 9. 2019
 *      Author: Jozo
 */

#ifndef SI7051_H_
#define SI7051_H_

#define I2C_ADR 0x80

#include "stm32f3xx_hal.h"

void SI7051_start_conversion (I2C_HandleTypeDef *i2c);
int16_t SI7051_read_temperature(I2C_HandleTypeDef *i2c);



#endif /* SI7051_H_ */
