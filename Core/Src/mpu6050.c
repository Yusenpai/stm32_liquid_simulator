#include "mpu6050.h"

#define I2C_MEM_WRITE()

void MPU6050_Init(MPU6050_Typedef *device, I2C_HandleTypeDef *i2c_handler)
{
	uint8_t data = 0x00;
	device->i2c_handler = i2c_handler;
	device->accel_x = 0;
	device->accel_y = 0;
	device->accel_z = 0;
	HAL_I2C_Mem_Write(device->i2c_handler, MPU6050_I2C_ADDRESS, PWR_MGMT_1, 1, &data, 1, 100);
}

void MPU6050_Read_Accel(MPU6050_Typedef *device)
{
	uint8_t raw_data[4];
	uint8_t reg_addr = ACCEL_XOUT_H_REG;
	HAL_I2C_Mem_Read(device->i2c_handler, MPU6050_I2C_ADDRESS, reg_addr, 1, raw_data, 4, 100);
	// Combine high and low bytes to get raw accelerometer values
	device->accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	device->accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
}

void MPU6050_Read_Accel_IT(MPU6050_Typedef *device)
{
	uint8_t reg_addr = ACCEL_XOUT_H_REG;
	HAL_I2C_Mem_Read_IT(device->i2c_handler, MPU6050_I2C_ADDRESS, reg_addr, 1, device->raw_data, 4);
}

void MPU6050_Read_Accel_DMA(MPU6050_Typedef *device){
	HAL_I2C_Mem_Read_DMA(device->i2c_handler, MPU6050_I2C_ADDRESS, ACCEL_XOUT_H_REG, 1, device->raw_data, 4);
}