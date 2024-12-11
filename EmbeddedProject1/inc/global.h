#pragma once

#include "main.h"

/******************************************************************************
* Global functions
******************************************************************************/

#define FRAC16(x) ((q15)((x) < 0.999969482421875 ? ((x) >= -1 ? (x)*0x8000 : 0x8000) : 0x7FFF))  // macro to get SF16 from fraction
#define FRAC32(x) ((Frac32)((x) < 1 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFFFF))
#define LIMITED_FRAC32(x) ((Frac32)((x) < 0.99999994 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFF80))



/*   ASM Functions   */


__attribute__( ( always_inline ) ) __STATIC_INLINE f32 asm_sqrt (f32 op1)
{
  f32 result;

  __ASM volatile ("VSQRT.F32 %0, %1" : "=t"(result) : "t"(op1));
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE f32 q15_to_float (s16 op1)
{
	f32 result;
  __ASM volatile ("VCVT.F32.S16 %0, %0, #15" : "=t"(result): "0"(op1));	
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE q15 float_to_q15 (f32 op1)
{
	q15 result;
  __ASM volatile ("VCVT.S16.F32 %0, %0, #15" : "=t"(result): "0"(op1));	
  return(result);
}


inline q15 mulf16(q15 a, q15 b)						/* Multiply two fractional SF16 numbers				*/
{
	return ((int32_t)a * b) >> 15;		
}


static inline void delay_us(u32 time_delay)
{		
	volatile u32 i;
	for (i = 0; i < time_delay; i++) ;
}


static inline void debug_toggle()
{
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
	delay_us(10);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
}

void	debug_init()										// debug pin PB15 
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
				
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}



void set_timer(Timer_typedef* timer_n, u32 delay)
{
	timer_n->dwell		= delay;
	timer_n->last_time	= HAL_GetTick();		
}

u8 check_tmr(Timer_typedef* timer_n)
{
	if (timer_n->autoset == ON)					// если с автоустановкой то устанавливаем следующий таймер
	{
		if ( HAL_GetTick() - timer_n->last_time >= timer_n->dwell )
		{			
			timer_n->last_time = HAL_GetTick();
			return 1;			
		}
	}
	else										// если без автоустановки то проверяем установила ли программа следующий таймер
	{
		if		(timer_n->last_time == -1) return 0;
		else if ( HAL_GetTick() - timer_n->last_time >= timer_n->dwell )
		{			
			timer_n->last_time = -1;
			return 1;		
		}
	}	
	return 0;
}





volatile u32 exec_time;
static inline void	Dbg_Tim_Start() /*start running*/
{
	#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
	#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
	#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT_CYCCNT = 0;
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk; 		// включаем счётчик
}
static inline void	Dbg_Tim_Stop_ns()
{
	exec_time = (DWT_CYCCNT) * 6;
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;	 
}



void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

void itoa(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }




void reverse_(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 
  
 // Converts a given integer x to string str[].  d is the number 
 // of digits required in output. If d is more than the number 
 // of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse_(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating point number to string. 
void ftoa_n(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * std::pow((float)10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 



void ftoa(float f, char *str, uint8_t precision) {
  uint8_t i, j, divisor = 1;
  int8_t log_f;
  int32_t int_digits = (int)f;             //store the integer digits
  float decimals;
  char s1[12];

  memset(str, 0, sizeof(s1));  
  memset(s1, 0, 10);

  if (f < 0) {                             //if a negative number 
    str[0] = '-';                          //start the char array with '-'
    f = abs(f);                            //store its positive absolute value
  }
  log_f = ceil(log10(f));                  //get number of digits before the decimal
  if (log_f > 0) {                         //log value > 0 indicates a number > 1
    if (log_f == precision) {              //if number of digits = significant figures
      f += 0.5;                            //add 0.5 to round up decimals >= 0.5
      itoa(f, s1);                         //itoa converts the number to a char array
      strcat(str, s1);                     //add to the number string
    }
    else if ((log_f - precision) > 0) {    //if more integer digits than significant digits
      i = log_f - precision;               //count digits to discard
      divisor = 10;
      for (j = 0; j < i; j++) divisor *= 10;    //divisor isolates our desired integer digits 
      f /= divisor;                             //divide
      f += 0.5;                            //round when converting to int
      int_digits = (int)f;
      int_digits *= divisor;               //and multiply back to the adjusted value
      itoa(int_digits, s1);
      strcat(str, s1);
    }
    else {                                 //if more precision specified than integer digits,
      itoa(int_digits, s1);                //convert
      strcat(str, s1);                     //and append
    }
  }

  else {                                   //decimal fractions between 0 and 1: leading 0
    s1[0] = '0';
    strcat(str, s1);
  }

  if (log_f < precision) {                 //if precision exceeds number of integer digits,
    decimals = f - (int)f;                 //get decimal value as float
    strcat(str, ".");                      //append decimal point to char array

    i = precision - log_f;                 //number of decimals to read
    for (j = 0; j < i; j++) {              //for each,
      decimals *= 10;                      //multiply decimals by 10
      if (j == (i-1)) decimals += 0.5;     //and if it's the last, add 0.5 to round it
      itoa((int)decimals, s1);			   //convert as integer to character array
      strcat(str, s1);                     //append to string
      decimals -= (int)decimals;           //and remove, moving to the next
    }
  }
}