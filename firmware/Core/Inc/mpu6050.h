#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32u5xx_hal.h"
#include <stdint.h>

#define MPU6050_I2C_ADDRESS (0x68 << 1)
#define WHO_AM_I_REG 0x75

#define INT_PIN_CFG_REG 0x37
#define INT_ENABLE_REG 0x38
#define INT_STATUS_REG 0x3A
#define ACCEL_XOUT_H_REG 0x3B
#define ACCEL_XOUT_L_REG 0x3C
#define ACCEL_YOUT_H_REG 0x3D
#define ACCEL_YOUT_L_REG 0x3E
#define ACCEL_ZOUT_H_REG 0x3F
#define ACCEL_ZOUT_L_REG 0x40

#define PWR_MGMT_1 0x6B // 0x40 default
#define PWR_MGMT_2 0x6C

typedef struct
{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint8_t raw_data[6];
	I2C_HandleTypeDef *i2c_handler;
} MPU6050_Typedef;

void MPU6050_Init(MPU6050_Typedef *device, I2C_HandleTypeDef *i2c_handler);
void MPU6050_Read_Accel(MPU6050_Typedef *device);
void MPU6050_Read_Accel_IT(MPU6050_Typedef *device);
void MPU6050_Read_Accel_DMA(MPU6050_Typedef *device);

#endif // __MPU6050_H__