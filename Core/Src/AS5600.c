/*
 * AS5600.c
 *
 *  Created on: Jan 3, 2025
 *      Author: Asus
 */
#include "AS5600.h"
#include <math.h>

static I2C_HandleTypeDef *sensor_hi2c;
#define RAW_ANGLE_LOW 0x0D
#define RAW_ANGLE_HIGH 0x0C

void AS5600_Init(I2C_HandleTypeDef *hi2c) {
    sensor_hi2c = hi2c;
}

float AS5600_ReadCorrectedAngle(void) {
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
