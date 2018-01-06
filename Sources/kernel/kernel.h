/****************************************************************************************************
 * kernel.h																							*
 *																									*
 *  Created on: 02/01/2018																			*
 *      Author: Evandro Teixeira																	*
 ****************************************************************************************************/

#ifndef SOURCES_KERNEL_KERNEL_H_
#define SOURCES_KERNEL_KERNEL_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "extern.h"

#define NUMBER_TASK		8
#define ID_IDLE			0

#ifndef kernel_tick
typedef uint32_t kernel_tick;
#endif

#ifndef id_Tack
typedef uint8_t id_Task;
#endif

#ifndef id_Flag
typedef uint16_t id_Flag;
#endif

#ifndef ptrTask
typedef void(*ptrTask)(void);
#endif

typedef enum
{
	Flag_False = false,
	Flag_True = true,
}Kernel_Flag;

typedef enum
{
	kernel_fail = false,
	kernel_ok = true,
}kernel_status;

typedef enum
{
	kernel_task_running = true,
	kernel_task_waiting = false,
}kernel_task;

typedef enum
{
	//Task_Running = 0,//
	Task_Ready = 0,
	Task_Blocked,
	Task_Paused,
	Task_Deleted,
}task_state;

typedef enum
{
	Priority_Idle = 0,
	Priority_Low,
	Priority_Medium,
	Priority_High,
}task_priority;

typedef struct
{
	ptrTask task;
	uint8_t state;
	uint8_t priority;
	kernel_tick pausedtime;
	kernel_task kernel_task_state;
}strTask;

typedef struct
{
	strTask task_vetor[NUMBER_TASK];
	uint8_t position_index;
	uint8_t counter_task;
	uint8_t index_high;
	uint8_t index_medium;
	uint8_t index_low;
}strKernel;

typedef struct
{
	id_Flag flag;
	id_Flag flag_index;
	id_Task number_task;
	bool flag_status;
}strFlag;

kernel_status kernel_init(void);

kernel_status kernel_add_task(ptrTask task,task_priority priority,task_state state, id_Task *id);

void kernel_task_delay(kernel_tick time);

void kernel_task_set_priority(task_priority priority);

void kernel_run(void);

kernel_status kernel_add_task_idle(void (*task)(void));

void kernel_task_delete(void);

kernel_status kernel_task_blocks(id_Task id);

kernel_status kernel_task_unlock(id_Task id);

void kernel_error(void);

//Kernel_Flag kernel_create_flag(id_Flag *ret);
//Kernel_Flag kernel_check_flag(id_Flag flag);
//void kernel_set_flag(id_Flag flag,Kernel_Flag st);

#endif /* SOURCES_KERNEL_KERNEL_H_ */
