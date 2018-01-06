/****************************************************************************************************
 * kernel.c																							*
 *																									*
 *  Created on: 02/01/2018																			*
 *      Author: Evandro Teixeira																	*
 ****************************************************************************************************/
#include "kernel.h"
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
//static strFlag vFlag;
static strKernel vKernel;
static kernel_tick ticks = 0;
extern uint32_t SystemCoreClock;
static kernel_tick kernel_ticks_get(void) ;
static void kernel_setup_systick(void);
static void kernel_task_idle(void);
static void (*IdleTask)(void);
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void SysTick_Handler(void)
{
	ticks++;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
static kernel_tick kernel_ticks_get(void)
{
	return ticks;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
static void kernel_setup_systick(void)
{
	uint32_t ticks = SystemCoreClock/1000;
	SysTick->LOAD  = ticks - 1;
	SysTick->VAL   = 0;
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	NVIC_EnableIRQ(SysTick_IRQn);
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
kernel_status kernel_init(void)
{
	strTask task_ilde = {kernel_task_idle,Task_Ready,Priority_Idle,0};
	vKernel.position_index = 0;
	vKernel.counter_task = 0;
	vKernel.task_vetor[ID_IDLE] = task_ilde;
	//vFlag.flag = 0;
	kernel_setup_systick();
	return kernel_ok;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
kernel_status kernel_add_task(ptrTask task,task_priority priority,task_state state, id_Task *id)
{
	if((vKernel.counter_task < NUMBER_TASK) && (task != NULL))
	{
		// Nota: task idle na posicao zero vetor
		vKernel.counter_task++;
		vKernel.task_vetor[vKernel.counter_task].task = task;
		vKernel.task_vetor[vKernel.counter_task].priority = priority;
		vKernel.task_vetor[vKernel.counter_task].state = state;
		vKernel.task_vetor[vKernel.counter_task].pausedtime = 0;

		*id = vKernel.counter_task;

		return kernel_ok;
	}
	else return kernel_fail;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void kernel_run(void)
{
	uint8_t i = 0;

	while(1)
	{
		//
		// Checa se tem alguma em pausa
		//
		for(i=1;i<(vKernel.counter_task + 1);i++)
		{
			if(vKernel.task_vetor[i].state == Task_Paused)
			{
				if(kernel_ticks_get() > vKernel.task_vetor[i].pausedtime)
				{
					vKernel.task_vetor[i].state = Task_Ready;
					vKernel.task_vetor[i].pausedtime = 0;
				}
			}
		}

		//
		// busca tarefa de alta prioridade pronta para ser executada
		//
		for(i=1;i<=(vKernel.counter_task + 1);i++)
		{
			if( (vKernel.task_vetor[i].priority == Priority_High) &&
				(vKernel.task_vetor[i].state == Task_Ready) )
				break;
		}
		if(i > (vKernel.counter_task + 1))
		{
			// busca tarefa de media prioridade para ser executada
			for(i=1;i<=(vKernel.counter_task + 1);i++)
			{
				if( (vKernel.task_vetor[i].priority == Priority_Medium) &&
					(vKernel.task_vetor[i].state == Task_Ready) )
					break;
			}
			if(i > (vKernel.counter_task + 1))
			{
				// busca tarefa de baixa prioridade para ser executada
				for(i=1;i<=(vKernel.counter_task + 1);i++)
				{
					if( (vKernel.task_vetor[i].priority == Priority_Low) &&
						(vKernel.task_vetor[i].state == Task_Ready) )
						break;
				}
				if(i > (vKernel.counter_task + 1))
					vKernel.position_index = ID_IDLE;	// Executa tarefa ilde
				else
					vKernel.position_index = i;
			}
			else
				vKernel.position_index = i;
		}
		else
			vKernel.position_index = i;

		// executa tarefa
		(*vKernel.task_vetor[vKernel.position_index].task)();
	}
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
static void kernel_task_idle(void)
{
	IdleTask();
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void kernel_task_delay(kernel_tick time)
{
	time += kernel_ticks_get();
	vKernel.task_vetor[vKernel.position_index].pausedtime = time;
	vKernel.task_vetor[vKernel.position_index].state = Task_Paused;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void kernel_task_set_priority(task_priority priority)
{
	if((priority > ID_IDLE) && (priority <= Priority_High))
		vKernel.task_vetor[vKernel.position_index].priority = priority;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
kernel_status kernel_add_task_idle(void (*task)(void))
{
	if(task != NULL)
	{
		IdleTask = task;
		return kernel_ok;
	}
	else return kernel_fail;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void kernel_task_delete(void)
{
	vKernel.task_vetor[vKernel.position_index].state = Task_Deleted;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
kernel_status kernel_task_blocks(id_Task id)
{
	if(id < vKernel.counter_task)
	{
		vKernel.task_vetor[id].state = Task_Blocked;

		return kernel_ok;
	}
	else return kernel_fail;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
kernel_status kernel_task_unlock(id_Task id)
{
	if(id < vKernel.counter_task)
	{
		vKernel.task_vetor[id].state = Task_Ready;

		return kernel_ok;
	}
	else return kernel_fail;
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
void kernel_error(void)
{
	while(1)
	{
		__asm ("NOP");
	}
}
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
/*void kernel_set_flag(id_Flag flag,Kernel_Flag st)
{
	if( vFlag.flag_status == true )
	{
		if(st == Flag_True)
			vFlag.flag |= flag;
		else
			vFlag.flag &= ~flag;
	}
}*/
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
/*Kernel_Flag kernel_check_flag(id_Flag flag)
{
	if( vFlag.flag_status == true)
	{
		if( vFlag.flag & flag )
		{
			return Flag_True;
		}
		else return Flag_False;
	}
	else return Flag_False;
}*/
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
/*Kernel_Flag kernel_create_flag(id_Flag *ret)
{
	id_Flag i = 1;

	if(vFlag.flag_index < 16)
	{
		*ret = i << vFlag.flag_index;
		vFlag.flag_index++;
		vFlag.flag_status = true;
		return Flag_True;
	}
	else return Flag_False;
}*/
/****************************************************************************************************
 * 																									*
 ****************************************************************************************************/
