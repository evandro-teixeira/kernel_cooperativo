/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/****************************************************************************************************
 * main.c																							*
 *																									*
 *  Created on: 02/01/2018																			*
 *      Author: Evandro Teixeira																	*
 ****************************************************************************************************/
#include "MKE06Z4.h"
#include "extern.h"

extern id_Task id_task_led_red;
extern id_Task id_task_led_blue;
extern id_Task id_task_sw2;
extern id_Task id_task_sw3;

int main(void)
{
	app_init();

	printf("\n\rAplicacao de Demonstracao do Kernel");

	if(kernel_init() == kernel_fail )
	{
		printf("\n\rFalha em iniciar o kernel");
		kernel_error();
	}

	if(kernel_add_task(app_task_led_red,Priority_High,Task_Ready,&id_task_led_red) == kernel_fail)
	{
		printf("\n\rFalha em add tarefa ao kernel");
		kernel_error();
	}

	if(kernel_add_task(app_task_led_blue,Priority_High,Task_Ready,&id_task_led_blue) == kernel_fail)
	{
		printf("\n\rFalha em add tarefa ao kernel");
		kernel_error();
	}

	if(kernel_add_task(app_task_sw2,Priority_High,Task_Ready,&id_task_sw2) == kernel_fail)
	{
		printf("\n\rFalha em add tarefa ao kernel");
		kernel_error();
	}

	if(kernel_add_task(app_task_sw3,Priority_High,Task_Ready,&id_task_sw3) == kernel_fail)
	{
		printf("\n\rFalha em add tarefa ao kernel");
		kernel_error();
	}

	if(kernel_add_task_idle(app_task_led_green) == kernel_fail)
	{
		printf("\n\rFalha em add tarefa ao kernel");
		kernel_error();
	}

	kernel_run();

    for (;;)
    {
    	printf("\n\rFalha em iniciar o kernel");
    	kernel_error();
    }
    /* Never leave main */
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
