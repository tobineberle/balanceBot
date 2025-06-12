/*
 * pid.c
 *
 *  Created on: Jun 5, 2025
 *      Author: tobin
 */


#include "pid.h"


void PID_init(PID_t* pid, float newTarget, float kp, float ki, float kd, float outputLimitMin, float outputLimitMax)
{
	pid->target = newTarget;
	pid->prevError = 0;
	pid->integral = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->outputLimitMin = outputLimitMin;
	pid->outputLimitMax = outputLimitMax;
}

/**
 * @brief	sets P gain
 *
 * @param	pid pointer
 * @param	kp gain
 */
void PID_setPGain(PID_t* pid, float kp)
{
	pid->kp = kp;
}

/**
 * @brief	sets I gain
 *
 * @param	pid pointer
 * @param	ki gain
 */
void PID_setIGain(PID_t* pid, float ki)
{
	pid->ki = ki;

}

/**
 * @brief	sets D gain
 *
 * @param	pid pointer
 * @param	kd gain
 */
void PID_setDGain(PID_t* pid, float kd)
{
	pid->kd = kd;

}

/**
 * @brief	Sets the target that the PID loop will try to reach
 *
 * @param	pid pointer
 * @param	a new target value
 */
void PID_setNewTarget(PID_t* pid, float newTarget)
{
	pid->target = newTarget;
}

void PID_reset(PID_t* pid){
	pid->integral = 0;
	pid->prevError = 0;
}

float PID_update(PID_t* pid, float currentVal, float t_us)
{
	float dt = US_TO_S(t_us);
	float error= pid->target - currentVal;

	//Proportional Term
	float p = error * pid->kp;

	//Integral Term
	pid->integral += error * dt;
	float i = pid ->integral;
	pid->integral *= I_DECAY_FACTOR;

	//Derivative Term
	float d = pid->kd * (error - pid->prevError) / dt;
	pid->prevError = error;

	error = p + i + d;
	error = error > pid->outputLimitMax ? pid->outputLimitMax : error;
	error = error < pid->outputLimitMin ? pid->outputLimitMin : error;

	return error;
}
