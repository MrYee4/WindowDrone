/*
 * Accelerometer_Interface.h
 *
 *  Created on: Dec 8, 2023
 *      Author: dogeb
 */
#ifndef INC_ACCELEROMETER_INTERFACE_H_
#define INC_ACCELEROMETER_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

// I2C addresses for the MPU6050
#define MPU_6050_ADDR (0x68	<< 1)// I2C Address shifted 1 bit to the left per HAL I2C 8 bit requirement
#define POWER_CONFIG_ADDR 0x6B   // Power config for reset and clock select
#define GYRO_CONFIG_ADDR 0x1B    // Gyroscope config register
#define ACCEL_CONFIG_ADDR 0x1C   // Accelerometer config register
#define I2C_CONFIG_ADDR 0x24	 // I2C config register
#define GYRO_ADDR 0x43			 // Gyroscope Data starting register 0x43 - 0x48
#define ACCEL_ADDR 0x3B			 // Accelerometer Data starting register 0x3B - 0x40

// MPU Power Managment 1 setup defines
#define PM1_No_Reset 0x00
#define PM1_With_Reset 0x80

// MPU Accelerometer sensitivity defines
#define Accel_2g_Sens 0x00
#define Accel_4g_Sens 0x08
#define Accel_8g_Sens 0x10
#define Accel_16g_Sens 0x18
#define Accel_2g_LSB_Divide 16384.0
#define Accel_4g_LSB_Divide 8192.0
#define Accel_8g_LSB_Divide 4096.0
#define Accel_16g_LSB_Divide 2048.0

// MPU Gyroscope sensitivity defines
#define Gyro_250_Sens 0x00
#define Gyro_500_Sens 0x08
#define Gyro_1000_Sens 0x10
#define Gyro_2000_Sens 0x18
#define Gyro_2g_LSB_Divide 131.0
#define Gyro_4g_LSB_Divide 65.5
#define Gyro_8g_LSB_Divide 32.8
#define Gyro_16g_LSB_Divide 16.4

#define Number_of_Calibrations 100

#ifdef __cplusplus
}
#endif

#endif /* INC_ACCELEROMETER_INTERFACE_H_ */
