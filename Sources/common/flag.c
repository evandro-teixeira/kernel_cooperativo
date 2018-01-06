/*
 *
 */

#include "flag.h"

static uint16_t Flag = 0;

void flag_Set(Flag_Value flag,Flag_Status status)
{
	if(status == flag_SET)
	{
		Flag |= flag;
	}
	else
	{
		Flag &= ~flag;
	}
}

Flag_Status flag_Get(Flag_Value flag)
{
	uint16_t var = 0;
	var = Flag & flag;
	if(var == flag)
		return flag_SET;
	else
		return flag_RESET;
}

void flag_Reset(void)
{
	Flag = 0;
}
