/************************************************************************************
 * app.h
 *
 *  Created on: 02/01/2018
 *      Author: Evandro Teixeira
 *************************************************************************************/

#ifndef SOURCES_APP_APP_H_
#define SOURCES_APP_APP_H_


#include "extern.h"

#define	 BUTTOM_SW3 		GPIO_PTH3,GPIO_PinInput
#define	 BUTTOM_SW2 		GPIO_PTH4,GPIO_PinInput

#define  READ_BUTTOM_SW2  	GPIO_PTH4_MASK
#define  READ_BUTTOM_SW3  	GPIO_PTH3_MASK

#define  BUTTOM				GPIOB

void app_init(void);

void app_task_led_red(void);

void app_task_led_blue(void);

void app_task_led_green(void);

void app_task_sw2(void);

void app_task_sw3(void);


#endif /* SOURCES_APP_APP_H_ */
