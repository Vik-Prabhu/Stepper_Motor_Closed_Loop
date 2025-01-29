/*
 * Stepper_Driver.h
 *
 *  Created on: Jan 4, 2025
 *      Author: Asus
 */

#ifndef INC_STEPPER_DRIVER_H_
#define INC_STEPPER_DRIVER_H_


#include "stm32f4xx_hal.h"

// PID parameters
#define KP 2.0f
#define KI 0.055f
#define KD 0.9f
#define STEPS_PER_REVOLUTION 400
#define ERROR_THRESHOLD 1.0f

float AS5600_ReadCorrectedAngle(I2C_HandleTypeDef *hi2c);
void MotorControl_Update(float angle , I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim );

#endif /* INC_STEPPER_DRIVER_H_ */
