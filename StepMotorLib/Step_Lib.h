/*
 * Step_Lib.h
 * @brief Stepper motor control library for STM32 and arduino. Use for motor drivers such as DRV8825, A4988, TB6600.
 *
 *  Created on: Jul 12, 2023
 *      Author: ilbeylia
 */



#ifndef INC_STEP_LIB_H_
#define INC_STEP_LIB_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(STM32F4)
#include "stm32f4xx.h"
#endif


typedef struct
{
	#if defined (STM32F4)
		TIM_HandleTypeDef*	Timer_handle;  // adım için gerekli timer
		uint32_t			Timer_channel;

		GPIO_TypeDef*		dir_GPIOx;
		uint16_t			dir_GPIO_Pin;
	#elif defined (ARDUINO)
		// arduino için gerkli yapı
	#endif

		struct
		{
			uint32_t	sayac;
			uint32_t	adim;
			uint32_t	durum;
		}step_set;



}step_config;

typedef enum {
	Step_DURDU,
	Step_BASLADI,
}step_durum;

typedef enum {
	Step_ILERI,
	Step_GERI,
}Step_yon;




void step_init(	step_config* step_config,
				#if defined (STM32F4)
				TIM_HandleTypeDef*	Timer_handle,
				uint32_t			Timer_channel,
				GPIO_TypeDef*		dir_GPIOx,
				uint16_t			dir_GPIO_Pin
				#elif defined (ARDUINO)
				// arduino için gerkli parametreler
				#endif
				);

void step(step_config* step_config, uint32_t adim, uint32_t yon);

void step_durdur(step_config* step_config);

#endif /* INC_STEP_LIB_H_ */
