/*
 * MPU6050.h
 *
 *  Created on: May 27, 2025
 *      Author: Tobin
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>


//MPU6050 Definitions
#define MPU6050_ADDRESS 0x68

#define FS_GYRO_250		0
#define FS_GYRO_500		8
#define FS_GYRO_1000	9
#define FS_GYRO_2000	10
#define GYRO_SCALE_250  131
#define GYRO_SCALE_500	65.5
#define GYRO_SCALE_1000	32.8
#define GYRO_SCALE_2000	16.4

#define FS_ACC_2G		0
#define FS_ACC_4G		8
#define FS_ACC_8G		9
#define FS_ACC_16G		10
#define ACC_SCALE_2G	16384
#define ACC_SCALE_4G	8192
#define ACC_SCALE_8G	4096
#define ACC_SCALE_16G	2048

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC	28
#define REG_USR_CTRL	107
#define REG_DATA		59

#define ACCEL_XOUT_H 	REG_DATA
#define ACCEL_YOUT_H	REG_DATA + 2
#define ACCEL_ZOUT_H	REG_DATA + 4
#define GYRO_XOUT_H		REG_DATA + 8
#define GYRO_YOUT_H		REG_DATA + 10
#define GYRO_ZOUT_H		REG_DATA + 12

//Calculation Definitions
#define RAD_TO_DEG				180/M_PI
#define KALMAN_INITAL_ANGLE		0
#define KALMAN_PNOISE			4	//gyroscope error [deg/sec]
#define KALMAN_SNOISE			1	//accelerometer error[deg]
#define KALMAN_ANGULAR_COVAR 	1 	//Initial angular uncertainty
#define US_TO_S(tus)			(tus/1000000.0f)
#define G						9.80665	//[m/s^2/g]

typedef enum {A2G, A4G, A8G, A16G} Accel_Resolution_e;
typedef enum {G250DPS, G500DPS, G1000DPS, G2000DPS} Gyro_Resolution_e;

//Public
typedef struct
{
	//State variables
	float angle; //[deg]

	//Uncertainty variables
	float angleCovar;
	float pNoise; //[deg/sec]

	//Kalman Filter variables
	float sNoise; //[deg]
	float kGain;
}kalman1D_TypeDef;

typedef struct
{
	I2C_HandleTypeDef* hi2cx;
	Accel_Resolution_e accelResolution;
	float accelScale;
	Gyro_Resolution_e gyroResolution;
	float gyroScale;
	kalman1D_TypeDef kalmanFilter;

}MPU6050_TypeDef;

//Public
void MPU6050_init(MPU6050_TypeDef* mpu, I2C_HandleTypeDef* hi2cx, Accel_Resolution_e gyroMode, Gyro_Resolution_e accelMode);
Accel_Resolution_e MPU6050_getGyroResolution(MPU6050_TypeDef* mpu);
bool MPU6050_setAccelResolution(MPU6050_TypeDef* mpu, Accel_Resolution_e resolution);
Gyro_Resolution_e MPU6050_getAccelResolution(MPU6050_TypeDef* mpu);
bool MPU6050_setGyroResolution(MPU6050_TypeDef* mpu, Gyro_Resolution_e resolution);
float MPU6050_getAccX(MPU6050_TypeDef* mpu);
float MPU6050_getAccY(MPU6050_TypeDef* mpu);
float MPU6050_getAccZ(MPU6050_TypeDef* mpu);
float MPU6050_getGyroX(MPU6050_TypeDef* mpu);
float MPU6050_getGyroY(MPU6050_TypeDef* mpu);
float MPU6050_getGyroZ(MPU6050_TypeDef* mpu);
float MPU6050_getAccAngleDegX(MPU6050_TypeDef* mpu);
float MPU6050_getAccAngleDegY(MPU6050_TypeDef* mpu);
float MPU6050_getAccAngleDegZ(MPU6050_TypeDef* mpu);
float MPU6050_getGyroAngleDegX(MPU6050_TypeDef* mpu, uint16_t dt_us);
float MPU6050_getGyroAngleDegY(MPU6050_TypeDef* mpu, uint16_t dt_us);
float MPU6050_getGyroAngleDegZ(MPU6050_TypeDef* mpu, uint16_t dt_us);
float MPU6050_getKalmanAngleDeg(MPU6050_TypeDef* mpu, uint16_t t_us, float measuredAngle, float measuredVelocity);

//Private
void _MPU6050_init_kalman(MPU6050_TypeDef* mpu, float angle, float angle_covar, float p_noise, float s_noise);

#endif /*INC_MPU6050_H_*/
