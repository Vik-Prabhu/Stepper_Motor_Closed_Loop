/*
 * Stepper_Driver.c
 *
 *  Created on: Jan 4, 2025
 *      Author: Vikram Prabhu
 */
#include "main.h"
#include "Stepper_Driver.h"
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
static uint32_t lastStepperMoveTime1 = 0;
static uint32_t lastStepperMoveTime2 = 0;

extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim5;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;



#include <math.h>

static I2C_HandleTypeDef *sensor_hi2c;
#define RAW_ANGLE_LOW 0x0D
#define RAW_ANGLE_HIGH 0x0C

float AS5600_ReadCorrectedAngle(I2C_HandleTypeDef *hi2c) {
    sensor_hi2c = hi2c;
    uint8_t buffer[2];
    int lowbyte, highbyte, rawAngle;
    float degAngle;

    HAL_I2C_Mem_Read(sensor_hi2c, 0x36 << 1, RAW_ANGLE_LOW, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(sensor_hi2c, 0x36 << 1, RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, HAL_MAX_DELAY);

    lowbyte = buffer[0];
    highbyte = buffer[1] << 8;
    rawAngle = highbyte | lowbyte;

    degAngle = rawAngle * 0.087890625;

    return degAngle;
}

void MotorControl_Update(float angle , I2C_HandleTypeDef *hi2c , TIM_HandleTypeDef *htim) {
    sensor_hi2c = hi2c;
    motor_htim = htim;
    HAL_TIM_Base_Start(motor_htim);
    uint32_t currentMillis = HAL_GetTick();
    float correctedAngle = AS5600_ReadCorrectedAngle(hi2c);
    targetAngle = angle;

    uint32_t *lastStepperMoveTime = (hi2c == &hi2c1) ? &lastStepperMoveTime1 : &lastStepperMoveTime2;

    if (currentMillis - *lastStepperMoveTime >= 10) {
        *lastStepperMoveTime = currentMillis;
        float angleError = targetAngle - correctedAngle;

        if (angleError > 180) angleError -= 360;
        if (angleError < -180) angleError += 360;

        integral += angleError;
        float derivative = angleError - previousError;
        float output = KP * angleError + KI * integral + KD * derivative;

        int stepsToMove = (int)fminf(fmaxf(output, -STEPS_PER_REVOLUTION), STEPS_PER_REVOLUTION);

        if (fabs(angleError) > ERROR_THRESHOLD  && sensor_hi2c == &hi2c1 ) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (stepsToMove > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            TIM2 -> CCR3 = 0;
            HAL_TIM_PWM_Start(motor_htim, TIM_CHANNEL_1);
            TIM2 -> CCR1 = 500;
            printf("Corrected Angle1: %.2f, Steps to Move1: %d\n", correctedAngle, stepsToMove);
            HAL_Delay(1);
        }

        if (fabs(angleError) > ERROR_THRESHOLD && sensor_hi2c == &hi2c2 )
        {
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (stepsToMove > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            TIM2 -> CCR1 = 0;
        	HAL_TIM_PWM_Start(motor_htim, TIM_CHANNEL_3);
        	TIM2 -> CCR3 = 500;
        	printf("Corrected Angle2: %.2f, Steps to Move2: %d\n", correctedAngle, stepsToMove);
        	HAL_Delay(1);
        }
        TIM2 -> CCR1 = 0;
        TIM2 -> CCR3 = 0;


        previousError = angleError;
        HAL_TIM_Base_Stop(motor_htim);

    }
}

//void TIM5_IRQHandler(void) {
//    if (__HAL_TIM_GET_FLAG(motor_htim, TIM_FLAG_UPDATE) != RESET) {
//        if (__HAL_TIM_GET_IT_SOURCE(motor_htim, TIM_IT_UPDATE) != RESET) {
//            __HAL_TIM_CLEAR_IT(motor_htim, TIM_IT_UPDATE);
//            pulse_counter++;
//            if (pulse_counter >= target_pulse_count) {
//            	TIM2->CCR3 = 0;
//            }
//        }
//    }
//}
//
//void TIM2_IRQHandler(void) {
//    if (__HAL_TIM_GET_FLAG(motor_htim, TIM_FLAG_UPDATE) != RESET) {
//        if (__HAL_TIM_GET_IT_SOURCE(motor_htim, TIM_IT_UPDATE) != RESET) {
//            __HAL_TIM_CLEAR_IT(motor_htim, TIM_IT_UPDATE);
//            pulse_counter++;
//            if (pulse_counter >= target_pulse_count) {
//            	TIM2->CCR1 = 0;
//            }
//        }
//    }
//}

