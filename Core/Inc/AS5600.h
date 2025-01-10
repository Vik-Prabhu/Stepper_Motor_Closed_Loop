/*
 * AS5600.h
 *
 *  Created on: Jan 3, 2025
 *      Author: Asus
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_


#include "stm32f4xx_hal.h"

void AS5600_Init(I2C_HandleTypeDef *hi2c);
float AS5600_ReadCorrectedAngle(void);

#endif /* INC_AS5600_H_ */
