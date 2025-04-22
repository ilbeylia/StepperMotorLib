/*
 * StepMotor_lib.c
 *
 *  Created on: Apr 22, 2025
 *      Author: ilbeyli
 */


#include "StepMotor_lib.h".h"

void stepper_motor_init(m_cnfg_s *motor_conf, TIM_HandleTypeDef *tim_handle,
		uint32_t tim_channel, GPIO_TypeDef *dir_portx, uint32_t dir_pin){
	motor_conf->tim_handle = tim_handle;
	motor_conf->tim_channel= tim_channel;
	motor_conf->dir_portx = dir_portx;
	motor_conf->dir_pin = dir_pin;
	motor_conf->m_set.state = MOTOR_OFF;
	motor_conf->m_set.counter = 0;
	motor_conf->m_set.step = 0;
	}

void startMotor(m_cnfg_s* motor_conf, direction_e direction, int step){
	if(motor_conf->m_set.state == MOTOR_OFF){
		motor_conf->m_set.step = step;
		motor_conf->m_set.state = MOTOR_ON;
		switch(direction){
			case CW:
				HAL_GPIO_WritePin(motor_conf->dir_portx, motor_conf->dir_pin, 1);
				break;
			case CCW:
				HAL_GPIO_WritePin(motor_conf->dir_portx, motor_conf->dir_pin, 0);
				break;
		}
		if (0 != step) {HAL_TIM_PWM_Start_IT(motor_conf->tim_handle, motor_conf->tim_channel);}
	}
}

void stopMotor(m_cnfg_s *motor_conf){
	if (motor_conf->m_set.counter == motor_conf->m_set.step){
		HAL_TIM_PWM_Stop_IT(motor_conf->tim_handle, motor_conf->tim_channel);
		motor_conf->m_set.counter= 0;
		motor_conf->m_set.step= 0;
		motor_conf->m_set.state= MOTOR_OFF;
	}
}
