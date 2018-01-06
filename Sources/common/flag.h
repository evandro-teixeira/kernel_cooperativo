#ifndef SOURCES_FLAG_H_
#define SOURCES_FLAG_H_

//#include "extern_common.h"
#include "extern.h"

typedef enum
{
	flag_0  = 0x0001,
	flag_1  = 0x0002,
	flag_2  = 0x0004,
	flag_3  = 0x0008,
	flag_4  = 0x0010,
	flag_5  = 0x0020,
	flag_6  = 0x0040,
	flag_7  = 0x0080,
	flag_8  = 0x0100,
	flag_9  = 0x0200,
	flag_10 = 0x0400,
	flag_11 = 0x0800,
	flag_12 = 0x1000,
	flag_13 = 0x2000,
	flag_14 = 0x4000,
	flag_15 = 0x8000,
}Flag_Value;

typedef enum
{
	flag_SET = true,
	flag_RESET = false
}Flag_Status;

// Flags da App
#define AD_CPU_FULL_BUFFER 		flag_0
#define AD_SENSOR_FULL_BUFFER 	flag_1
#define SW_BOTTUM_1				flag_2
#define SW_BOTTUM_2				flag_3
#define SW_BOTTUM_3				flag_4
#define SW_BOTTUM_4				flag_5
#define REFRESH_DISPLAY		    flag_6

void flag_Set(Flag_Value flag,Flag_Status status);
Flag_Status flag_Get(Flag_Value flag);
void flag_Reset(void);

#endif /* SOURCES_FLAG_H_ */
