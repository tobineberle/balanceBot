#include <mpu6050.h>

void MPU6050_init(MPU6050_TypeDef* mpu, I2C_HandleTypeDef* hi2cx, Accel_Resolution_e accelReso, Gyro_Resolution_e gyroReso){

	  //Check MPU6050 connection over I2C
	  HAL_StatusTypeDef conn = HAL_I2C_IsDeviceReady(hi2cx, (MPU6050_ADDRESS <<1) + 0, 1, 100);
	  if(conn == HAL_OK)
	  {
		  printf("MPU6050 Connected\n");
	  }
	  else
	  {
		  printf("MPU6050 Not Found\n");
		  return;
	  }
	  mpu->hi2cx = hi2cx;
	  uint8_t pwrMode = 0;
	  HAL_I2C_Mem_Write(hi2cx,(MPU6050_ADDRESS <<1) + 0, REG_USR_CTRL,1,&pwrMode, 1, 100);
	  MPU6050_setAccelResolution(mpu, accelReso);
	  MPU6050_setGyroResolution(mpu, gyroReso);
	  _MPU6050_init_kalman(mpu, 0, KALMAN_ANGULAR_COVAR, KALMAN_PNOISE, KALMAN_SNOISE);
}

bool MPU6050_setAccelResolution(MPU6050_TypeDef* mpu, Accel_Resolution_e resolution){
	uint8_t accelFSMode;
	 switch(resolution){
		  case A2G:
			  mpu->accelScale = ACC_SCALE_2G;
			  accelFSMode = FS_ACC_2G;
			  break;

		  case A4G:
			  mpu->accelScale = ACC_SCALE_4G;
			  accelFSMode = FS_ACC_4G;
			  break;

		  case A8G:
			  mpu->accelScale = ACC_SCALE_8G;
			  accelFSMode = FS_ACC_8G;
			  break;

		  case A16G:
			  mpu->accelScale = ACC_SCALE_16G;
			  accelFSMode = FS_ACC_16G;
			  break;
		  }
	  HAL_StatusTypeDef conn = HAL_I2C_Mem_Write((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 0, REG_CONFIG_ACC, 1, &accelFSMode, 1, 100);
	  if(conn == HAL_OK){
		  return true;
	  }
	  else{
		  return false;
	  }
}

bool MPU6050_setGyroResolution(MPU6050_TypeDef* mpu, Gyro_Resolution_e resolution){
	uint8_t gyroFSMode;
	  switch(resolution){
		  case G250DPS:
			  mpu->gyroScale = GYRO_SCALE_250;
			  gyroFSMode = FS_GYRO_250;
			  break;

		  case G500DPS:
			  mpu->gyroScale = GYRO_SCALE_500;
			  gyroFSMode = FS_GYRO_500;
			  break;

		  case G1000DPS:
			  mpu->gyroScale = GYRO_SCALE_1000;
			  gyroFSMode = FS_GYRO_1000;
			  break;

		  case G2000DPS:
			  mpu->gyroScale = GYRO_SCALE_2000;
			  gyroFSMode = FS_GYRO_2000;
			  break;
		  }
	  HAL_StatusTypeDef conn = HAL_I2C_Mem_Write((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 0, REG_CONFIG_GYRO, 1, &gyroFSMode, 1, 100);
	  if(conn == HAL_OK){
		  return true;
	  }
	  else{
		  return false;
	}
}

float MPU6050_getAccX(MPU6050_TypeDef* mpu)
{

	uint8_t data[2];
	int16_t x_acc;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, ACCEL_XOUT_H, 1, data, 2, 100);
	x_acc = ((int16_t)data[0] << 8) + data[1];
	return (float)x_acc/mpu->accelScale;
}

float MPU6050_getAccY(MPU6050_TypeDef* mpu)
{
	uint8_t data[2];
	int16_t y_acc;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, ACCEL_YOUT_H, 1, data, 2, 100);
	y_acc = ((int16_t)data[0] << 8) + data[1];
	return (float)y_acc/mpu->accelScale;
}

float MPU6050_getAccZ(MPU6050_TypeDef* mpu)
{
	uint8_t data[2];
	int16_t z_acc;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, ACCEL_ZOUT_H, 1, data, 2, 100);
	z_acc = ((int16_t)data[0] << 8) + data[1];
	return (float)z_acc/mpu->accelScale;
}

float MPU6050_getGyroX(MPU6050_TypeDef* mpu)
{
	uint8_t data[2];
	int16_t x_gyro;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, GYRO_XOUT_H, 1, data, 2, 100);
	x_gyro = ((int16_t)data[0] << 8) + data[1];
	return (float)x_gyro/mpu->gyroScale;
}

float MPU6050_getGyroY(MPU6050_TypeDef* mpu)
{
	uint8_t data[2];
	int16_t y_gyro;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, GYRO_YOUT_H, 1, data, 2, 100);
	y_gyro = ((int16_t)data[0] << 8) + data[1];
	return (float)y_gyro/mpu->gyroScale;
}

float MPU6050_getGyroZ(MPU6050_TypeDef* mpu)
{
	uint8_t data[2];
	int16_t z_gyro;
	HAL_I2C_Mem_Read((mpu->hi2cx), (MPU6050_ADDRESS <<1) + 1, GYRO_ZOUT_H, 1, data, 2, 100);
	z_gyro = ((int16_t)data[0] << 8) + data[1];
	return (float)z_gyro/mpu->gyroScale;
}

float MPU6050_getAccAngleDegX(MPU6050_TypeDef* mpu)
{
	float x =  MPU6050_getAccX(mpu);
	float y =  MPU6050_getAccY(mpu);
	float z =  MPU6050_getAccZ(mpu);
	return (float)atan(x/(sqrt(pow(y,2) + pow(z, 2))))*RAD_TO_DEG;
}

float MPU6050_getAccAngleDegY(MPU6050_TypeDef* mpu)
{
	float x =  MPU6050_getAccX(mpu);
	float y =  MPU6050_getAccY(mpu);
	float z =  MPU6050_getAccZ(mpu);
	return (float)atan(y/(sqrt(pow(x,2) + pow(z, 2))))*RAD_TO_DEG;
}

float MPU6050_getAccAngleDegZ(MPU6050_TypeDef* mpu)
{
	float x =  MPU6050_getAccX(mpu);
	float y =  MPU6050_getAccY(mpu);
	float z =  MPU6050_getAccZ(mpu);
	return (float)atan((sqrt(pow(x,2) + pow(y, 2))/z))*RAD_TO_DEG;
}

void _MPU6050_init_kalman(MPU6050_TypeDef* mpu, float angle, float angle_covar, float p_noise, float s_noise)
{
	mpu->kalmanFilter.angle = angle;
	mpu->kalmanFilter.angleCovar = angle_covar;
	mpu->kalmanFilter.pNoise = p_noise;
	mpu->kalmanFilter.sNoise = s_noise;
	mpu->kalmanFilter.kGain = 0;
}

float MPU6050_getKalmanAngleDeg(MPU6050_TypeDef* mpu, uint16_t t_us, float measuredAngle, float measuredVelocity)
{
	float t_s = (float)US_TO_S(t_us);
	kalman1D_TypeDef* k = &(mpu->kalmanFilter);

	/**
	 * 1. a) State Equation
	 * angle(k)[deg] = angle(k-1)[deg] + t[s]*velocity[deg/s]
	 */
	k->angle += t_s * measuredVelocity;

	/**
	 * 1. b) Covariance Equation (Uncertainty)
	 * P(k) = P(k-1) + process_noise [deg^2]
	 */
	k->angleCovar += pow(t_s,2) * pow(k->pNoise, 2);

	/**
	 * 2. a) Kalman Gain
	 * K(k) = P(k)/(P(k) + r) [unitless]
	 */
	k->kGain = (k->angleCovar)/(k->angleCovar + pow(k->sNoise,2));

	/**
	 * 2. b) State Correction
	 * angle`(k) = angle(k) + K(k)*(angle_measured - angle(k))
	 */
	k->angle += k->kGain * (measuredAngle - k->angle);

	/**
	 * 2. c) Covariance Correction
	 * P`(k) = (1- K(k))*P(k)
	 */
	k->angleCovar *= (1 - k->kGain);

	return k->angle;
}

float MPU6050_getGyroAngleDegX(MPU6050_TypeDef* mpu, uint16_t dt_us){
	float t_s = (float)US_TO_S(dt_us);
	return (float)(MPU6050_getGyroX(mpu) * t_s);
}

float MPU6050_getGyroAngleDegY(MPU6050_TypeDef* mpu, uint16_t dt_us){
	float t_s = (float)US_TO_S(dt_us);
	return (float)(MPU6050_getGyroY(mpu) * t_s);
}

float MPU6050_getGyroAngleDegZ(MPU6050_TypeDef* mpu, uint16_t dt_us){
	float t_s = (float)US_TO_S(dt_us);
	return (float)(MPU6050_getGyroZ(mpu) * t_s);
}

//float _MPU6050_compensateAccelX(MPU6050_TypeDef* mpu, float accX, float gyroX, float gyroY, float gyroZ)
//{
//    float a_centripetal_x =
//        -gyroY * gyroY * mpu->xOffset +
//         gyroY * gyroX * mpu->yOffset +
//         gyroZ * gyroZ * mpu->xOffset -
//         gyroZ * gyroX * mpu->zOffset;
//
//    return accX - a_centripetal_x;
//}
//float _MPU6050_compensateAccelY(MPU6050_TypeDef* mpu, float accY, float gyroX, float gyroY, float gyroZ)
//{
//    float a_centripetal_y =
//        -gyroX * gyroY * mpu->xOffset -
//         gyroX * gyroX * mpu->yOffset +
//         gyroZ * gyroY * mpu->zOffset +
//         gyroZ * gyroZ * mpu->yOffset;
//
//    return accY - a_centripetal_y;
//}
//float _MPU6050_compensateAccelZ(MPU6050_TypeDef* mpu,float accZ, float gyroX, float gyroY, float gyroZ)
//{
//    float a_centripetal_z =
//         gyroX * gyroZ * mpu->xOffset +
//         gyroY * gyroZ * mpu->yOffset -
//         gyroX * gyroX * mpu->zOffset -
//         gyroY * gyroY * mpu->zOffset;
//
//    return accZ - a_centripetal_z;
//}


