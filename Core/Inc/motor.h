/*
 * motor.h
 *
 *  Created on: Jun 17, 2025
 *      Author: ms
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include <stdint.h>

void StepMotor_SendPulses(uint32_t steps);

void StepMotor_homing(uint32_t speed);

#endif /* INC_MOTOR_H_ */
