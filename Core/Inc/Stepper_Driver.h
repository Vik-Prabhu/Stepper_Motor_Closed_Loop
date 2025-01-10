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
#define KP 5.1f
#define KI 0.055f
#define KD 0.7f
#define STEPS_PER_REVOLUTION 400
#define ERROR_THRESHOLD 1.0f

void MotorControl_Init(TIM_HandleTypeDef *htim);
void MotorControl_Update(float angle);

#endif /* INC_STEPPER_DRIVER_H_ */
