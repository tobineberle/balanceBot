/*
 * A4988.c
 *
 *  Created on: Jun 2, 2025
 *      Author: tobin
 */

#include "A4988.h"

/**
 * @brief 	Initialize the minimum requirements to use the motor. Defaults to no microstepping
 * @note	MUST call A4988_timer_init to finish motor setup, and must call init_microstepping to enable microstepping
 *
 * @param	dirPort is the port for direction
 * @param	dirPin is the pin for the dirPort
 * @param	mode can be either CONSTANT_SPEED or LINEAR_SPEED
 * @retval	An initialized motor object
 */
void A4988_init(A4988_t* motor, GPIO_TypeDef* dirPort, uint16_t dirPin, uint8_t mode)
{
	//Pins
	motor->_dirPort = dirPort;
	motor->_dirPin = dirPin;
	//Default no microstepping
	motor->_microStepsPerRev = 200;
	motor->_rpm = 0;
	motor->_sps = 0;
	motor->_resolution = FULL;
	motor->_state = IDLE;
	motor->_stabilizeCounter = 0;
	motor->_position = 0;
	motor->_totalSteps = 0;
	motor->_stepsRemaining = 0;
	motor->_accelSteps = 0;
	motor->_decelSteps = 0;
	motor->_cruiseSteps = 0;
	motor->_speedProfile._mode = mode;
	motor->_speedProfile._accelRate = 500;
	motor->_speedProfile._decelRate = 500;

	switch(mode){
	case CONSTANT_SPEED:
		motor->_irqHandler = _A4988_IRQ_constantSpeed;
		break;
	case LINEAR_SPEED:
		motor->_irqHandler = _A4988_IRQ_linearSpeed;
		break;
	case CONTINUOUS_SPEED:
		motor->_irqHandler = _A4988_IRQ_continuousSpeed;
		break;
	case STEP_QUEUE:
		motor->_irqHandler = _A4988_IRQ_stepQ;
		break;
	}
	_A4988_enableTIM_IRQ(motor);
}

/**
 * @brief	Initializes the timer to drive the step pin
 *
 * @param	motor object initialized
 * @param 	timer chosen for running the motor (eg TIM2)
 * @param	timerChannel for the timer (1)
 * @param 	timerInt is for interrupt, needs the IRQn (eg TIM2_IRQn)
 * @param	APBClockFreq is clock frequency of your APB bus
 */
void A4988_timer_init(A4988_t* motor, TIM_TypeDef* timer, uint8_t timerChannel, IRQn_Type timerInt, uint32_t APBClockFreq)
{
	//Init timer
	motor->_timer = timer;
	motor->_timerChannel = timerChannel;
	motor->_timerInt = timerInt;

	//Configuring timer
	//Disable counter
	motor->_timer->CR1 &=~ TIM_CR1_CEN;
	//Set pre-scaler for 1us tick
	motor->_timer->PSC = (APBClockFreq / 1000000) - 1;
	//Initialize ARR, this value doesn't matter and will change
	motor->_timer->ARR = 5000;
	//The value to be compared to TIMx counter and signaled on OC1 output
	//This is active duty cycle and needs to be longer than our A4988 settling time of 1us
	motor->_timer->CCR1 = MIN_PULSE_LENGTH;

	//Sets 0x6, enabling PWM mode 1
	//Channel x is active as long as TIMx->CNT < CCR1
	if(motor->_timerChannel == 1)      motor->_timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	else if(motor->_timerChannel == 2) motor->_timer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	else if(motor->_timerChannel == 3) motor->_timer->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	else if(motor->_timerChannel == 4) motor->_timer->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

	//Clear interrupt flags
	motor->_timer->SR = 0;
	//Capture compare interrupt enable
	motor->_timer->DIER |= 1 << motor->_timerChannel;
	//Enable timer
	motor->_timer->CR1 |= TIM_CR1_CEN;
	}

/**
 * @brief	Initializes the timer to drive the step pin for step queue mode
 * @note	DO NOT CALL RPM SETTING OR MOVE FUNCTIONS AFTER INITIALZING THIS WAY AS YOU WILL RESET ARR AND THROW OFF TIMER IRQ CALLS
 * @note	DONT ASK HOW LONG IT TOOK TO FIGURE THAT ONE OUT
 *
 * @param	motor object initialized
 * @param 	timer chosen for running the motor (eg TIM2)
 * @param 	timerInt is for interrupt, needs the IRQn (eg TIM2_IRQn)
 * @param	APBClockFreq is clock frequency of your APB bus
 * @param	motorClockPeriod in us
 * @param	port for stepping
 * @param	pin for stepping
 */
void A4988_StepQ_timer_init(A4988_t* motor, TIM_TypeDef* timer, IRQn_Type timerInt, uint32_t APBClockFreq, uint32_t motorClockPeriod, GPIO_TypeDef* stepPort, uint16_t stepPin){
	//Init timer
	motor->_timer = timer;
	motor->_timerChannel = 0;
	motor->_timerInt = timerInt;
	motor->_timer->CR1 &=~ TIM_CR1_CEN;
	motor->_timer->PSC = (APBClockFreq / 1000000) - 1;
	motor->_timer->ARR = motorClockPeriod - 1;
//	motor->_timer->PSC = 0;
//	motor->_timer->ARR = 1599;
	motor->_timer->EGR = TIM_EGR_UG;
	//IRQ on ARR value
	motor->_timer->DIER |= TIM_DIER_UIE;
	motor->_timer->CR1 |= TIM_CR1_CEN;
	_A4988_enableTIM_IRQ(motor);

	//Init Step pin
	//Medium speed for faster response, the switching speed should be 25MHz
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	motor->_stepPort = stepPort;
	motor->_stepPin = stepPin;
	HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = stepPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(stepPort, &GPIO_InitStruct);
}

/**
 * @brief	Initializes microstepping, to use microstepping call set_microsteps
 *
 * @param	motor object initialized
 * @param	MSxPort port for x microstepping pin
 * @param	MSxPin	pin corresponding to the MSxPort
 */
void A4988_microstepping_init(A4988_t* motor, GPIO_TypeDef* MS1Port, uint16_t MS1Pin, GPIO_TypeDef* MS2Port, uint16_t MS2Pin, GPIO_TypeDef* MS3Port, uint16_t MS3Pin)
{
	motor->_MS1Port = MS1Port;
	motor->_MS1Pin = MS1Pin;
	motor->_MS2Port = MS2Port;
	motor->_MS2Pin = MS2Pin;
	motor->_MS3Port = MS3Port;
	motor->_MS3Pin = MS3Pin;
}

/**
 * @brief 	Sets up the reset, sleep, and enable pins
 * @note	This could probably be broken up into individual inits
 *
 * @param	motor object initialized
 * @param	xxxPort port for the corresponding function
 * @param 	xxxPin pin for the corresponding port
 */
void A4988_alt_states_init(A4988_t* motor, GPIO_TypeDef* RSTPort, uint16_t RSTPin, GPIO_TypeDef* SLPPort, uint16_t SLPPin, GPIO_TypeDef* ENPort, uint16_t ENPin)
{
	motor->_RSTPort = RSTPort;
	motor->_RSTPin = RSTPin;
	motor->_SLPPort = SLPPort;
	motor->_SLPPin = SLPPin;
	motor->_ENPort = ENPort;
	motor->_ENPin = ENPin;
}

/**
 * @brief 	Enables the interrupt for the timer
 *
 * @param	motor object initialized
 */
void _A4988_enableTIM_IRQ(A4988_t* motor)
{
	NVIC_EnableIRQ(motor->_timerInt);
}

/**
 * @brief	Starts the motor after calling a move.
 * 			Will only start when motor is IDLE and after STABILIZING is completed
 * @note	This sets the CCxE bit of TIMx which toggles whether the signal is output on the corresponding output pin
 *
 * @param	motor pointer
 */
void _A4988_start(A4988_t* motor)
{
	motor->_state = RUNNING;
	//Reset timer and force ARR to update
	//Question: will this allow moves to overwrite?
	//CNT/EGR might need to be moved back to the run method
	motor->_timer->CNT = 0;
	motor->_timer->EGR |= TIM_EGR_CC1G;
	//Connect timer to output pin (ARMED)
	motor->_timer->CCER |= 1 << (motor->_timerChannel - 1) * 4;
}

/**
 * @brief	Stops the motor
 * @note	This sets the CCxE bit of TIMx which toggles whether the signal is output on the corresponding output pin
 * 			Does not stop the timer
 *
 * @param	motor pointer
 */
void _A4988_stop(A4988_t* motor)
{
	motor->_state = IDLE;
	//Disconnect timer from output pin (DISARMED)
	motor->_timer->CCER &=~ 1 << (motor->_timerChannel - 1) * 4;
}

/**
 * @brief	IRQ handler for motor control, controls the state of the motor and width of motor pulses
 * @note	At the end of every timer cycle the IRQ will be called and handled by this function. This is what
 * 			lets us set the state and the pulse of the next clock cycle. MAKE SURE this function is called in your timer
 * 			interrupt initializer in stm32f4xx_it.c
 * 			The actual handling is broken into smaller functions to minimize handling time
 *
 * @param	motor pointer
 */
void A4988_IRQ_Handler(A4988_t* motor){
	motor->_irqHandler(motor);
}

/**
 * @brief	IRQ handler for constant speed mode
 *
 * @param	motor pointer
 */
void _A4988_IRQ_constantSpeed(A4988_t* motor){
	if(!(motor->_timer->SR & TIM_SR_CC1IF)) return;
	motor->_timer->SR &=~ TIM_SR_CC1IF;
	switch(A4988_getState(motor)){
	case RUNNING:
		if(motor->_stepsRemaining > 0){
			motor->_stepsRemaining--;
			if(A4988_getDir(motor) == FORWARD){
				motor->_position++;
			}
			else{
				motor->_position--;
			}
		}
		else{
			_A4988_stop(motor);
		}
		break;

	case STABILIZING:
		_A4988_IRQ_stabilizing(motor);
		break;
	}
}

/**
 * @brief	IRQ handler for linear speed mode
 *
 * @param	motor pointer
 */
void _A4988_IRQ_linearSpeed(A4988_t* motor){
	if(!(motor->_timer->SR & TIM_SR_CC1IF)) return;
	motor->_timer->SR &=~ TIM_SR_CC1IF;
	uint16_t arr = 0 ;
	switch(A4988_getState(motor)){
	case RUNNING:
		if(motor->_stepsRemaining > 0){
			motor->_stepsRemaining--;
			if(A4988_getDir(motor) == FORWARD){
				motor->_position++;
			}
			else{
				motor->_position--;
			}
			//Determine motion sub-state (accel, cruise, decel)
			switch(A4988_getMotionState(motor)){
			case ACCEL:
				motor->_accelSteps--;
				//Checks triangular speed profile
				if(motor->_accelSteps == 0 && motor->_cruiseSteps == 0){
					motor->_speedProfile._motionState = DECEL;
				}
				else if (motor->_accelSteps == 0){
					motor->_speedProfile._motionState = CRUISE;
					arr = _A4988_computeArrFromRPM(motor, motor->_rpm);
					_A4988_setARR(motor, arr);
				}
				else{
					arr = _A4988_computeArrFromSpS2(motor, (motor->_totalSteps - motor->_stepsRemaining), motor->_speedProfile._accelRate);
					_A4988_setARR(motor, arr);
				}
				break;

			case CRUISE:
				if(motor->_stepsRemaining == motor->_decelSteps){
				motor->_speedProfile._motionState = DECEL;
				}
				break;

			case DECEL:
				motor->_decelSteps--;
				arr = _A4988_computeArrFromSpS2(motor, motor->_stepsRemaining, motor->_speedProfile._decelRate);
				_A4988_setARR(motor, arr);
				break;
			}
		}
		//No steps remain, reset to idle
		else{
			_A4988_stop(motor);
		}
		break;

	case STABILIZING:
		_A4988_IRQ_stabilizing(motor);
		break;
	}

}

/**
 * @brief	IRQ handler for continuous speed mode
 *
 * @param	motor pointer
 */
void _A4988_IRQ_continuousSpeed(A4988_t* motor){
	if(!(motor->_timer->SR & TIM_SR_CC1IF)) return;
	motor->_timer->SR &=~ TIM_SR_CC1IF;
	switch(A4988_getState(motor)){
	case RUNNING:
		if(A4988_getDir(motor) == FORWARD){
				motor->_position++;
			}
			else{
				motor->_position--;
			}
		break;

	case STABILIZING:
		_A4988_IRQ_stabilizing(motor);
		break;
	}
}

/**
 * @brief	Reusable function that waits for motors to stabilize before enabling PWM.
 * 			Used for IRQs
 *
 * @param	motor pointer
 */
void _A4988_IRQ_stabilizing(A4988_t* motor){
	//Waiting for DIR pin to stabilize
	if(motor->_stabilizeCounter > 0) motor->_stabilizeCounter--;
	else
	{
		motor->_state = RUNNING;
		//Enable PWM Mode 1
		if(motor->_timerChannel == 1)      motor->_timer->CCMR1 |= TIM_CCMR1_OC1M_2;
		else if(motor->_timerChannel == 2) motor->_timer->CCMR1 |= TIM_CCMR1_OC2M_2;
		else if(motor->_timerChannel == 3) motor->_timer->CCMR2 |= TIM_CCMR2_OC3M_2;
		else if(motor->_timerChannel == 4) motor->_timer->CCMR2 |= TIM_CCMR2_OC4M_2;
	}
}

/**
 * @brief	IRQ handler for continuous stepQ.
 * @note	Needs to be FAST
 *
 * @param	motor pointer
 */
void _A4988_IRQ_stepQ(A4988_t* motor)
{
    if (!(motor->_timer->SR & TIM_SR_UIF)) return;
    motor->_timer->SR &= ~TIM_SR_UIF;
    motor->_totalSteps += 1;
    if (motor->_position == 0) return;

    bool dir = (motor->_position> 0);
    motor->_position += dir ? -1 : 1;

    // DIR setup
    motor->_dirPort->ODR = (motor->_dirPort->ODR & ~motor->_dirPin)|(dir ? motor->_dirPin : 0);
    _A4988_NOP_Delay(16);
    // STEP pulse — keep high for ~1 µs
    motor->_stepPort->BSRR = motor->_stepPin;        // STEP HIGH
    _A4988_NOP_Delay(16);
    motor->_stepPort->BSRR = (uint32_t)motor->_stepPin << 16;         // STEP LOW
}

/**
 * @brief	Provides a blocking delay on the order of 62.5ns
 *
 * @param 	iters, multiplier for ns wait (e.g. 62.5ns * iters = delay time)
 */
void _A4988_NOP_Delay(uint16_t iters){
	for(uint16_t i = 0; i < iters; i++){
		//62.5ns delay (@16MHz)
		__NOP();
	}
}

/**
* @brief	Function for queuing steps
* 			Uses atomic IRQ disable and re-enable to prevent overlap between queuing steps and reading them
*
* @param	motor pointer
* @param	steps to queue
*/
void A4988_queueSteps(A4988_t* motor, int32_t steps){
	__disable_irq();
	motor->_position += steps;
	__enable_irq();
}


/**
 * @brief	Move the motor a specified number of steps. Will use your current microstepping resolution
 *
 * @param 	motor pointer
 * @param	steps to move (positive or negative)
 */
void A4988_move(A4988_t* motor, int32_t steps){
	//Check mode profile to setup runtime parameters
	motor->_totalSteps = abs(steps);
	motor->_stepsRemaining =abs(steps);

	switch(A4988_getMode(motor)){
	case LINEAR_SPEED:
		//Calculate speed in [steps/s] for determining accel decel steps
		uint16_t speed = motor->_rpm * motor->_microStepsPerRev/60;
		motor->_accelSteps = speed * speed / (2 * motor->_speedProfile._accelRate);
		motor->_decelSteps = motor->_accelSteps * (motor->_speedProfile._accelRate/motor->_speedProfile._decelRate);
		motor->_cruiseSteps = motor->_totalSteps - motor->_accelSteps - motor->_decelSteps;

		//Check if our total steps is more than accel/decel step count
		if(motor->_totalSteps < (motor->_accelSteps + motor->_decelSteps)){
			//Make a triangular speed profile, accelSteps = totalsteps * (decelRate/(accelRate+decelRate)
			motor->_accelSteps = motor->_totalSteps * motor->_speedProfile._decelRate/(motor->_speedProfile._accelRate + motor->_speedProfile._decelRate);
			motor->_decelSteps = motor->_totalSteps - motor->_accelSteps;
			motor->_cruiseSteps = 0;
		}
		//Setup pulse here
		motor->_speedProfile._motionState = ACCEL;
		break;

	case CONSTANT_SPEED:
		motor->_accelSteps = 0;
		motor->_decelSteps = 0;
		motor->_speedProfile._motionState = CRUISE;
		break;
	}
	//Check direction change
    bool dirChanged = _A4988_setDir(motor,(steps > 0 ? FORWARD : BACKWARD));
	//Start movement if in same direction
	if(!dirChanged)_A4988_start(motor);
	else{
		//Start PWM when stabilized
		//Connect timer channel to output pin (armed not firing)
		motor->_timer->CCER |= 1 << (motor->_timerChannel -1) * 4;
	}
}

/**
 *  @brief	Runs the motor at continuous steps per second
 *
 * @param	motor pointer
 * @param	steps per second (positive or negative direction)
 */
void A4988_run(A4988_t* motor, int16_t sps){
	//No updating if sps is constant
	if(sps == motor->_sps)return;

	//Stopping if less than a complete step/second
	uint16_t a_sps = abs(sps);
//	if (a_sps < pow(2, motor->_resolution)){
	if(a_sps == 0){
		motor->_rpm = 0;
		_A4988_stop(motor);
		return;
	}
	//Set the rpm
	uint32_t arr = 0;
	motor->_sps = sps;
	motor->_rpm = a_sps * 60 / motor->_microStepsPerRev;

	if(motor->_rpm > MAX_RPM){
		motor->_rpm = MAX_RPM;
		motor->_sps = MAX_RPM * motor->_microStepsPerRev / 60;
		arr = _A4988_computeArrFromRPM(motor, MAX_RPM);
	}
	else{
		 arr = _A4988_computeArrFromSpS(motor, a_sps);
	}
	//Set ARR
	_A4988_setARR(motor, arr);
//	motor->_timer->CNT = 0;
//	motor->_timer->EGR |= TIM_EGR_CC1G;

	//Check direction change
    bool dirChanged = _A4988_setDir(motor,(sps > 0 ? FORWARD : BACKWARD));

	//Start movement if in same direction
	if(!dirChanged)
	{
		_A4988_start(motor);
	}
	//Start PWM when stabilized
	else
	{
		//Connect timer channel to output pin (armed not firing)
		motor->_timer->CCER |= 1 << (motor->_timerChannel -1) * 4;
	}
}

/**
 * @brief	Private function to set the direction of the motor based on steps or rotation angle
 * @note	Motor needs to stabilize after direction change, this time can be adjusted with STABILIZE_TIME
 *
 * @param	motor pointer
 * @param	boolean indicating FORWARD or BACKWARD (use enum)
 * @retval	boolean indicating whether direction has changed
 */
bool _A4988_setDir(A4988_t* motor, bool dir)
{
	//If no change in direction return false
	if(A4988_getDir(motor) == dir){
		return false;
	}
	//Otherwise, wait for stabilizing
	motor->_state = STABILIZING;
	//Total wait time = STABILIZE_TIME * ARR duration [us]. Because min is 1[us] a low STABILIZE_TIME should be acceptable
	motor->_stabilizeCounter = STABILIZE_TIME;
	//Sets capture/compare mode register's (CCMRx) output compare mode (OCxM) to inactive on match, disabling the timer output
	if(			motor->_timerChannel == 1) motor->_timer->CCMR1 &=~ TIM_CCMR1_OC1M_2;
		else if(motor->_timerChannel == 2) motor->_timer->CCMR1 &=~ TIM_CCMR1_OC2M_2;
		else if(motor->_timerChannel == 3) motor->_timer->CCMR2 &=~ TIM_CCMR2_OC3M_2;
		else if(motor->_timerChannel == 4) motor->_timer->CCMR2 &=~ TIM_CCMR2_OC4M_2;

		HAL_GPIO_WritePin(motor->_dirPort, motor->_dirPin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
		return true;
}

/**
 * @brief	Returns the read value of the motor direction
 *
 * @param	motor pointer
 * @retval	boolean for motor direction (FORWARD 0, BACKWARD 1)
 */
bool A4988_getDir(A4988_t* motor)
{
	return HAL_GPIO_ReadPin(motor->_dirPort, motor->_dirPin);
}

/**
 * @brief	Private function to compute the auto reload register time from RPM
 * @note	This is basically a conversion from speed to pulse width by adjusting the timer length
 *
 * @param	motor pointer
 * @param	rpm speed
 * @retval	arr value to be fed into the timer ARR
 */
uint32_t _A4988_computeArrFromRPM(A4988_t* motor, uint16_t rpm)
{
	//Frequency [1/s] = microStepPerRev [steps/rev] * rpm [rev/min] / 60[s/min]
	float freq = (motor->_microStepsPerRev * rpm)/60.0;
	//Avoid DB0
	if (freq <0.1f) return 0xFFFF;
	//ARR Time [us] = 1000000[us/s] / frequency[1/s]
//	printf("%f\n", freq);
	return (uint32_t)1000000/freq;
}

/**
 * @brief	computes ARR from steps per second. Useful if running the motors with a gyroscope
 *
 * @param	motor pointer
 * @parma 	sps is steps per second value
 * @retval	arr for pwm frequency
 */
uint32_t _A4988_computeArrFromSpS(A4988_t* motor, uint16_t sps){
	//Frequency [1/s] = steps/sec
	float freq = (float)sps;
	//Avoid DB0
	if (freq <0.1f) return 0xFFFF;
	//ARR Time [us] = 1000000[us/s] / frequency[1/s]
	return (uint32_t)1000000/freq;
}

/**
 * @brief	Private function to compute the auto reload register time from steps per second^2
 * @note	Used to convert the accleration and distance covered into a new velocity which can be converted to a
 * 			motor PWM frequency
 *
 * @param	motor pointer
 * @param	deltaSteps is the total amount of steps taken
 * @param	acceleration rate [steps/s^2]
 * @retval	arr value to be fed into the timer ARR
 */
uint32_t _A4988_computeArrFromSpS2(A4988_t* motor, uint16_t deltaSteps, uint16_t rate)
{
	//Frequency
	float freq = sqrt(2 * rate * deltaSteps);
	//Avoid DBO
	if (freq <0.1f) return 0xFFFF;
//	printf("%f\n", freq);
	//ARR Time [us] = 1000000[us/s] / frequency[1/s]
	return (uint32_t)1000000/freq;
}

/**
 * @brief	Sets the auto reload register
 * @note	Setting the ARR determines the length of each clock cycle. By setting this we can control
 * 			motor speed by varying the length of step pulses
 *
 * @param	motor pointer
 * @param	arr value to be set
 */
void _A4988_setARR(A4988_t* motor, uint32_t arr)
{
	//Set ARR timer for duty cycle
	//Safety so that the timer doesn't wrap during CCR1 pulse, this should never theoretically happen
	if((arr - 1) < MIN_PULSE_LENGTH){
		arr = MIN_PULSE_LENGTH + 1;
	}
	motor->_timer->ARR = arr - 1;
}

/**
 * @brief 	Used to set RPM when motor is idled
 *
 * @param	motor pointer
 * @param	rpm
 */
void A4988_setRPM(A4988_t* motor, uint16_t rpm)
{
	uint16_t _rpm = rpm > MAX_RPM ? MAX_RPM : rpm;
	//Deactive timer
	motor->_timer->CR1 &=~ TIM_CR1_CEN;
	uint32_t arr = _A4988_computeArrFromRPM(motor, _rpm);
	_A4988_setARR(motor, arr);
	motor->_rpm = _rpm;
	//Reactivate timer
	motor->_timer->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief	Get motor speed [rpm]
 *
 * @param	motor pointer
 * @retval	speed in rpm
 */
volatile uint16_t A4988_getRPM(A4988_t* motor)
{
	return motor->_rpm;

}

/**
 * @brief	Get position (net value over program running time)
 *
 * @param	motor pointer
 * @retval	net position in steps
 */
volatile uint16_t A4988_getPosition(A4988_t* motor)
{
	return motor->_position;
}

volatile int16_t A4988_getAngleDeg(A4988_t* motor) //Needs work
{
	 return (int16_t)(( (double)motor->_position / motor->_microStepsPerRev ) * 360.0) % 360;
}

/**
 * @brief	Gets the current state of the motor. Can use this in main.c to determine when the motor is ready for next move
 * @note	Needs updating to include queueing steps so that we dont have to go to idle inbetween steps of same direction
 * 			Explore this later
 *
 * @param	motor pointer
 * @retval	state, check enum for values
 */
volatile uint8_t A4988_getState(A4988_t* motor)
{
	return motor->_state;
}

/**
 * @brief	Gets the mode of the motor
 *
 * @param	motor pointer
 * @retval	mode of the motor (CONSTANT or LINEAR speed)
 */
volatile uint8_t A4988_getMode(A4988_t* motor)
{
	return motor->_speedProfile._mode;
}

/**
 * @brief	Gets the motion state of the motor
 *
 * @param	motor pointer
 * @retval	motion state
 */
volatile uint8_t A4988_getMotionState(A4988_t* motor)
{
	return motor->_speedProfile._motionState;
}

/**
 * @brief	Sets the motor mode (constant/linear/continuous)
 *
 * @param	motor pointer
 * @param	mode, constant, linear, continuous
 */
void A4988_setMode(A4988_t* motor, uint8_t mode)
{
	motor->_speedProfile._mode = mode;
	switch(mode){
	case CONSTANT_SPEED:
		motor->_irqHandler = &_A4988_IRQ_constantSpeed;
		break;
	case LINEAR_SPEED:
		motor->_irqHandler = &_A4988_IRQ_linearSpeed;
		break;
	case CONTINUOUS_SPEED:
		motor->_irqHandler = &_A4988_IRQ_continuousSpeed;
		break;
	}
}

/**
 * @brief	Private function to set the motor state based on the mode
 *
 * @param	motor pointer
 * @param	motion state to put the motor into
 */
void _A4988_setMotionState(A4988_t* motor, uint8_t motionState)
{
	motor->_speedProfile._motionState = motionState;
}

/**
 * @brief	Gets the current motor resolution
 *
 * @param	motor pointer
 * @retval	motor resolution
 */
A4988_Resolution_e A4988_getMicrostepResolution(A4988_t* motor)
{
	return motor->_resolution;
}

/**
 * @brief	Set the motor acceleration
 *
 * @param	motor pointer
 * @param	motor acceleration in steps/s^2
 */
void A4988_setAccel(A4988_t* motor, uint16_t accel)
{
	motor->_speedProfile._accelRate = accel;
}

/**
 * @brief	Set the motor deceleration
 *
 * @param	motor pointer
 * @param	motor deceleration in steps/s^2
 */
void A4988_setDecel(A4988_t* motor, uint16_t decel)
{
	motor->_speedProfile._decelRate = decel;
}

void A4988_setLinearSpeedProfile(A4988_t* motor, SpeedProfile_t profile)
{
	motor->_speedProfile._mode = LINEAR_SPEED;
	motor->_speedProfile._accelRate = profile._accelRate;
	motor->_speedProfile._accelRate = profile._decelRate;
}

/**
 * @brief	Sets full step
 */
static void setFullStep(A4988_t* motor){
	HAL_GPIO_WritePin(motor->_MS1Port, motor->_MS1Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->_MS2Port, motor->_MS2Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->_MS3Port, motor->_MS3Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Sets Half step
 */
static void setHalfStep(A4988_t* motor){
	HAL_GPIO_WritePin(motor->_MS1Port, motor->_MS1Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS2Port, motor->_MS2Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->_MS3Port, motor->_MS3Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Sets quarter step
 */
static void setQuarterStep(A4988_t* motor){
	HAL_GPIO_WritePin(motor->_MS1Port, motor->_MS1Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->_MS2Port, motor->_MS2Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS3Port, motor->_MS3Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Sets eighth step
 */
static void setEighthStep(A4988_t* motor){
	HAL_GPIO_WritePin(motor->_MS1Port, motor->_MS1Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS2Port, motor->_MS2Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS3Port, motor->_MS3Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Sets sixteenth step
 */
static void setSixteenthStep(A4988_t* motor){
	HAL_GPIO_WritePin(motor->_MS1Port, motor->_MS1Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS2Port, motor->_MS2Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->_MS3Port, motor->_MS3Pin, GPIO_PIN_SET);
}

/**
 * @brief	Sets a new microstep resolution, defaults to FULL step
 *
 * @param	motor pointer
 * @param 	resolution to be set
 */
void A4988_setMicrostepResolution(A4988_t* motor, A4988_Resolution_e resolution)
{
	switch(resolution){
	case FULL:
		setFullStep(motor);
		motor->_microStepsPerRev = 200;
		break;

	case HALF:
		setHalfStep(motor);
		motor->_microStepsPerRev = 400;
		break;

	case QUARTER:
		setQuarterStep(motor);
		motor->_microStepsPerRev = 800;
		break;

	case EIGHTH:
		setEighthStep(motor);
		motor->_microStepsPerRev = 1600;
		break;

	case SIXTEENTH:
		setSixteenthStep(motor);
		motor->_microStepsPerRev = 3200;
		break;

	default:
		setFullStep(motor);
		motor->_resolution = FULL;
		motor->_microStepsPerRev = 200;
		return;
	}

	//Scaling up the resolution
	if(resolution > motor->_resolution){
		motor->_position *= pow(2, (resolution - motor->_resolution));
		motor->_sps *= pow(2, (resolution - motor->_resolution));
	}
	//Scaling down resolution
	else{
		motor->_position /= pow(2, (resolution - motor->_resolution));
		motor->_sps /= pow(2, (resolution - motor->_resolution));
	}
	motor->_resolution = resolution;
	if(motor->_speedProfile._mode != STEP_QUEUE){
		A4988_setRPM(motor, motor->_rpm);
	}
}

/**
 * DEFUNCT
 */
//void _A4988_IRQ_stepQ(A4988_t* motor){
//	//Clear UIF flags (required by software)
//	if(!(motor->_timer->SR & TIM_SR_UIF)) return;
//	motor->_timer->SR &=~ TIM_SR_UIF;
//	//Tracking IRQ calls
//    motor->_totalSteps += 1;
//	if(motor->_position == 0) return;
//
//	bool dir = (motor->_position > 0);
//    HAL_GPIO_WritePin(motor->_dirPort, motor->_dirPin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
//
//    //Use a step
//    _A4988_NOP_Delay(32);
//    HAL_GPIO_WritePin(motor->_stepPort, motor->_stepPin, GPIO_PIN_SET);
//    _A4988_NOP_Delay(32);
//    HAL_GPIO_WritePin(motor->_stepPort, motor->_stepPin, GPIO_PIN_RESET);
//
//    motor->_position += dir ? -1 : 1;
//}
