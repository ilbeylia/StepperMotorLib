/*
 * Step_Lib.c
 * @brief Stepper motor control library for STM32 and arduino. Use for motor drivers such as DRV8825, A4988, TB6600.
 *  Created on: Jul 12, 2023
 *      Author: ilbeylia
 */

#include "Step_Lib.h"

void step_init(	step_config* step_config,
				#if defined (STM32F4)
					TIM_HandleTypeDef*	Timer_handle,
					uint32_t			Timer_channel,
					GPIO_TypeDef*		dir_GPIOx,
					uint16_t			dir_GPIO_Pin
				#elif defined (ARDUINO)
					int					Step_pin,
					int 				Dir_pin,

				#endif
				)
{
	#if defined (STM32F4)
		step_config->Timer_handle 	= Timer_handle;
		step_config->Timer_channel 	= Timer_channel;
		step_config->dir_GPIOx		= dir_GPIOx;
		step_config->dir_GPIO_Pin	= dir_GPIO_Pin;
	#elif defined (ARDUINO)
		step_config->Step_pin 		= Step_pin;
		step_config->Dir_pin 		= Dir_pin;

		pinMode(Step_pin, OUTPUT);
		pinMode(Dir_pin, OUTPUT);
	#endif
}

void step(step_config* step_config, uint32_t adim, uint32_t yon)
{
	#if defined (STM32F4)
		if (step_config->step_set.durum == Step_DURDU){
			step_config->step_set.adim = adim;
			step_config->step_set.durum = Step_BASLADI;
			if(yon == Step_ILERI ){
				HAL_GPIO_WritePin(step_config->dir_GPIOx, step_config->dir_GPIO_Pin, 1);

			}
			else {
				HAL_GPIO_WritePin(step_config->dir_GPIOx, step_config->dir_GPIO_Pin, 0);

			}

			if (0 != adim){
				HAL_TIM_PWM_Start_IT(step_config->Timer_handle, step_config->Timer_channel);
				__HAL_TIM_SET_COMPARE(step_config->Timer_handle, step_config->Timer_channel, 100);
			}
		}
	#elif defined (ARDUINO)
		if (step_config->step_set.durum == Step_DURDU)
			step_config->step_set.adim = adim;
			step_config->step_set.durum = Step_BASLADI;

			digitalWrite(step_config->Dir_pin, yon);
			if (adim != 0 ){
				for (uint32_t i = 0; i < adim; i++){
					digitalWrite(step_config->Step_pin, HIGH);
                    delayMicroseconds(500);  
                    digitalWrite(step_config->Step_pin, LOW);
                    delayMicroseconds(500);  
				}
			}

	#endif
}

void step_durdur(step_config* step_config){

	#if defined (STM32F4)
		if (step_config->step_set.sayac == step_config->step_set.adim){
			HAL_TIM_PWM_Stop_IT(step_config->Timer_handle, step_config->Timer_channel);
			step_config->step_set.durum = Step_DURDU;
			step_config->step_set.sayac =0;
		}
	#elif defined (ARDUINO)
        if (step_config->step_set.sayac == step_config->step_set.adim){
            step_config->step_set.durum = Step_DURDU;
            step_config->step_set.sayac = 0;
        }
	#endif
}
