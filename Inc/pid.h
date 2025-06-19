/*
 * pid.h
 *
 *  Created on: Jun 5, 2025
 *      Author: tobin
 */
#include <stdlib.h>
#include <stdint.h>
#include <mpu6050.h>

#ifndef INC_PID_H_
#define INC_PID_H_

#define I_DECAY_FACTOR 0.98


typedef struct{
	float kp;
	float ki;
	float kd;

	float target;
	float prevError;

	float outputLimitMin;
	float outputLimitMax;

	float integral;
//	float integralLimitMin;
//	float integralLimitMax;

}PID_t;

void PID_init(PID_t* pid, float newTarget, float kp, float ki, float kd, float outputLimitMin, float outputLimitMax);
void PID_reset(PID_t* pid);
float PID_update(PID_t* pid, float currentVal, float t_ms, float enableDFilter);

void PID_setPGain(PID_t* pid, float kp);
void PID_setIGain(PID_t* pid, float ki);
void PID_setDGain(PID_t* pid, float kd);
void PID_setNewTarget(PID_t* pid, float newTarget);
float PID_lowPassFilter(float new_val, float old_val, float alpha);

#endif /* INC_PID_H_ */
