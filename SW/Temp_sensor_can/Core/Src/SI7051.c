/*
 * SI7051.c
 *
 *  Created on: 12. 9. 2019
 *      Author: Jozo
 */


#include "SI7051.h"

/*Spusti iba konverziu teploty. Konverzia trva 7 az 11ms.*/
void SI7051_start_conversion (I2C_HandleTypeDef *i2c)
{
	uint8_t TxBuffer[]={0xF3};

	HAL_I2C_Master_Transmit(i2c, I2C_ADR, TxBuffer, 1, 2);
}

/*Vrati float teplotu v �C*/
int16_t SI7051_read_temperature(I2C_HandleTypeDef *i2c)
{
	uint8_t RxBuffer[2];
	uint16_t RAW_value = 0;
	int32_t temperature;
	HAL_I2C_Master_Receive(i2c, I2C_ADR, RxBuffer, 2, 10);
	RAW_value = ((uint16_t)RxBuffer[0] << 8) + RxBuffer[1];
	temperature =((17572*(int32_t)RAW_value)>>16)-4685;
	return ((int16_t)temperature)/10;
}
