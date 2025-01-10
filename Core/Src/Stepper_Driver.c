/*
 * Stepper_Driver.c
 *
 *  Created on: Jan 4, 2025
 *      Author: Vikram Prabhu
 */
#include "main.h"
#include "Stepper_Driver.h"
#include "AS5600.h" // for reading angle
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

static TIM_HandleTypeDef *motor_htim;
volatile uint32_t pulse_counter = 0;
volatile uint32_t target_pulse_count = 0;
static float integral = 0;
static float previousError = 0;
static float targetAngle = 0;
static uint32_t lastStepperMoveTime = 0;

void MotorControl_Init(TIM_HandleTypeDef *htim) {
    motor_htim = htim;
    HAL_TIM_Base_Start(motor_htim);
    HAL_TIM_Base_Start_IT(motor_htim);
}

void MotorControl_Update(float angle) {
    uint32_t currentMillis = HAL_GetTick();
    float correctedAngle = AS5600_ReadCorrectedAngle();
    targetAngle = angle;

    if (currentMillis - lastStepperMoveTime >= 100) {
        lastStepperMoveTime = currentMillis;
        float angleError = targetAngle - correctedAngle;

        if (angleError > 180) angleError -= 360;
        if (angleError < -180) angleError += 360;

        integral += angleError;
        float derivative = angleError - previousError;
        float output = KP * angleError + KI * integral + KD * derivative;

        int stepsToMove = (int)fminf(fmaxf(output, -STEPS_PER_REVOLUTION), STEPS_PER_REVOLUTION);

        if (fabs(angleError) > ERROR_THRESHOLD) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (stepsToMove > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            TIM2_PWM_Config(abs(stepsToMove));
        }

        previousError = angleError;
        printf("Corrected Angle: %.2f, Steps to Move: %d\n", correctedAngle, stepsToMove);
    }
}

void TIM2_PWM_Config( uint32_t pulse_count) {
    target_pulse_count = pulse_count;
    pulse_counter = 0;

    HAL_TIM_PWM_Start(motor_htim, TIM_CHANNEL_1);
	TIM2->CCR1 = 500;
    __HAL_TIM_ENABLE_IT(motor_htim, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(motor_htim, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(motor_htim, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(motor_htim, TIM_IT_UPDATE);
            pulse_counter++;
            if (pulse_counter >= target_pulse_count) {
            	TIM2->CCR1 = 0;
            }
        }
    }
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if (htim->Instance == TIM2)
//	    {
//	        pulse_counter++;  // Increment pulse counter
//
//	        if (pulse_counter >= target_pulse_count)  // Stop after 1000 pulses
//	        {
//	        	TIM2->CCR1 = 0;
//	        }
//	    }
//}

