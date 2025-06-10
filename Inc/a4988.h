/*
 * a4988.h
 *
 *  Created on: Jun 2, 2025
 *      Author: tobin
 *
 *		NON BLOCKING STEPPER CONTROL WITH A4988 DRIVERS
 *		Adapted from code by Patrick Schwarzenecker
 *
 *      A4988 Specifications:
 *      VDD = 5V
 *      EN = ACTIVE LOW or STM GPIO OUTPUT
 *      MS1 = LOW or STM GPIO OUTPUT
 *      MS2 = LOW or STM GPIO OUTPUT
 *      MS3 = LOW or STM GPIO OUTPUT
 *      RST = ACTIVE LOW or STM GPIO OUTPUT
 *      SLP = ACTIVE LOW or STM GPIO OUTPUT
 *      STEP = TIM OUTPUT COMPARE CHANNELx/GPIO with HSI and global interrupt
 *      DIR = STM GPIO OUTPUT
 *
 *		Pin setup:
 *		1. SET: timer to output compare mode
 *		2. ENABLE: global interrupt flag
 *		3. INSERT: your motors as A4988_t motorName in main.c under USER CODE BEGIN PV
 *
 *      Interrupt setup:
 *		1. GO TO: stm32f4xx_it.c
 *		2. INSERT: #include "A4988_PS.h"
 *		3. INSERT: extern A4988_t motorName;
 *		4.  LOOK FOR: void TIMx_IRQHandler(void)
 *		4.1 INSERT: A4988_IRQHandler(&motorName);
 */

#include <stdbool.h>
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <math.h>

#ifndef INC_A4988_H_
#define INC_A4988_H_

#define STABILIZE_TIME		10
#define MIN_PULSE_LENGTH	10	//[us], note 1us min pulse length minimum, we give 10 for margin
#define MIN(a,b)			(b < a ? b : a)
#define MAX(a,b)			(b > a ? b : a)

enum Direction {FORWARD, BACKWARD};
enum State {IDLE, RUNNING, SLEEPING, STABILIZING};
enum Mode {CONSTANT_SPEED, LINEAR_SPEED};
enum MotionState {ACCEL, CRUISE, DECEL};
typedef enum {FULL, HALF, QUARTER, EIGHTH, SIXTEENTH} A4988_Resolution_e;

typedef struct
{
	uint8_t _mode;
	uint8_t _motionState;
	uint16_t _accelRate;
	uint16_t _decelRate;

}SpeedProfile_t;

typedef struct
{
	/*
	 * Driver Config
	 */
	GPIO_TypeDef* _dirPort;
	uint16_t _dirPin;

	//Not required to be configured
	GPIO_TypeDef* _MS1Port;
	GPIO_TypeDef* _MS2Port;
	GPIO_TypeDef* _MS3Port;
	uint16_t _MS1Pin;
	uint16_t _MS2Pin;
	uint16_t _MS3Pin;
	GPIO_TypeDef* _SLPPort;
	GPIO_TypeDef* _RSTPort;
	GPIO_TypeDef* _ENPort;
	uint16_t _SLPPin;
	uint16_t _RSTPin;
	uint16_t _ENPin;

	/*
	 * Timer Variables
	 */
	TIM_TypeDef* _timer ;
	uint8_t _timerChannel ;
	IRQn_Type _timerInt;

	/*
	 * Motor Variables
	 */
	volatile uint16_t _microStepsPerRev;
	uint16_t _rpm;						//Speed
	A4988_Resolution_e _resolution;		//Microstepping resolution
	volatile uint8_t _state;			//State of the motor
	volatile uint16_t _stabilizeCounter;//For direction change stabilization
	volatile uint32_t _position;		//Current position cumulative
	volatile uint32_t _totalSteps;		//Current total count
	volatile uint16_t _stepsRemaining;	//To complete current move
	volatile uint16_t _accelSteps;		//To reach cruise
	volatile uint16_t _decelSteps;		//To reach brake
	volatile uint16_t _cruiseSteps;		//Total cruise steps
	SpeedProfile_t _speedProfile;		//Profile to configure accel, decel, and modes
}A4988_t;

//Remove or change for your program
extern A4988_t motA;
extern A4988_t motB;
//

/**
 * Functions
 */

//Public Init
A4988_t A4988_init(GPIO_TypeDef* dirPort, uint16_t dirPin, uint8_t mode);
void A4988_timer_init(A4988_t* motor, TIM_TypeDef* timer, uint8_t timerChannel, IRQn_Type timerInt, uint32_t APBClockFreq);
void A4988_microstepping_init(A4988_t* motor, GPIO_TypeDef* MS1Port, uint16_t MS1Pin, GPIO_TypeDef* MS2Port, uint16_t MS2Pin, GPIO_TypeDef* MS3Port, uint16_t MS3Pin);
void A4988_alt_states_init(A4988_t* motor, GPIO_TypeDef* RSTPort, uint16_t RSTPin, GPIO_TypeDef* SLPPort, uint16_t SLPPin, GPIO_TypeDef* ENPort, uint16_t ENPin);
void A4988_enableTIM_IRQ(A4988_t* motor);

//Private Motor Control
void _A4988_start(A4988_t* motor);
void _A4988_stop(A4988_t* motor);
bool _A4988_setDir(A4988_t* motor, bool dir);
uint32_t _A4988_computeArrFromRPM(A4988_t* motor, uint16_t rpm);
uint32_t _A4988_computeArrFromSpS2(A4988_t* motor, uint16_t deltaSteps, uint16_t rate);
void _A4988_setARR(A4988_t* motor, uint32_t arr);
void _A4988_setMotionState(A4988_t* motor, uint8_t motionState);

//Public
void A4988_IRQ_Handler(A4988_t* motor);
void A4988_move(A4988_t* motor, int32_t steps);
bool A4988_getDir(A4988_t* motor);
void A4988_setRPM(A4988_t* motor, uint16_t rpm);
volatile uint16_t A4988_getRPM(A4988_t* motor);
volatile uint16_t A4988_getPosition(A4988_t* motor);
volatile int16_t A4988_getAngleDegree(A4988_t* motor); //Needs work
volatile uint8_t A4988_getState(A4988_t* motor);
volatile uint8_t A4988_getMode(A4988_t* motor);
void A4988_setMode(A4988_t* motor, uint8_t mode);
volatile uint8_t A4988_getMotionState(A4988_t* motor);
void A4988_setMicrostepResolution(A4988_t* motor, A4988_Resolution_e resolution);
A4988_Resolution_e A4988_getMicrostepResolution(A4988_t* motor);
void A4988_setAccel(A4988_t* motor, uint16_t accel);
void A4988_setDecel(A4988_t* motor, uint16_t decel);
void A4988_setLinearSpeedProfile(A4988_t* motor, SpeedProfile_t profile);

//TODO
//void A4988_nextAngleDegCommand(A4988_t* motor, int32_t angle);
//void A4988_sleep(A4988_t* motor);
//void A4988_wakeup(A4988_t* motor);

#endif /* INC_A4988_H_ */
