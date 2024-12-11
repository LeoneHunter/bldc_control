#pragma once

#include "stdint.h"

/******************************************************************************
* Public types
******************************************************************************/

typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;

typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;

typedef float32_t	f32;
typedef float64_t   d64;

typedef int8_t      sf8;
typedef int16_t     q15;
typedef int32_t     sf32; 

typedef q15			q15;

typedef float		f32;

enum			{OFF, ON};
enum sign		{minus, plus};
enum flag		{reset,  set};
enum adc_mode	{READY=1, FINISHED, START};

enum			{TMR_AUROSET_OFF,  TMR_AUROSET_ON};
enum			{TMR_SET,          TMR_DONT_SET};

/* Typedefs ----------------------------------------------------------------------*/

typedef struct
{
	u16		dwell;
	u8		autoset;
	s32	    last_time;
	
}Timer_typedef;

typedef struct 
{
	arm_pid_instance_f32 arm_pid_params;
	f32 posPIDlimit;
	f32 negPIDlimit;

}PID_params_typedef;

typedef struct  
{
	u16 A, B, C;

}PWM_buffer_typedef;






