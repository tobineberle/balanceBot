# balanceBot-v0
Using STM32 NucleoF401RE board to protoype a self balancing robot. Writing drivers for A4988 motor controller, MPU6050 IMU sensor, and others from scratch. 

## A4988 Motor Driver 
- Non-blocking motor controller code with speed profile
- Motors can be set into CONSTANT_SPEED (continous RPM) or LINEAR_SPEED (trapezoidal speed with accleration and deceleration) speed profiles
- Currently uses output compare mode (OCM) with varying auto reload register (ARR) times to control motor PWM. May update this to direct PWM in the future of for constant velocity control
- Ideas to add move queues (this can remove ramp down time if next move is in same direction) as well as time based motor movements
- Working on constant velocity mode to use with cascading PID control

## MPU6050 IMU
- I2C interface and initialization
- Current X/Y/Z angle functions using accelerometer data
- Current X/Y/Z angle functions using gyro and time delta data
- 1D kalman filter for smoothing of current angle data (removes gyro drift and accelerometer noise)

## PID Control
- Basic outline for PID control blocks with configurable target (set points)
- Configurable output limiting for physical device constraints
