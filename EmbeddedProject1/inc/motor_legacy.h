/* USER CODE BEGIN Header */


#include "main.h"
#include "global.h"



//#include "arm_math.h"

typedef struct
{
	u16 motor_accel_time, 
		motor_spdref, 
		motor_freq, 
		motor_umin,
		motor_deadtime,
		motor_vfcoeff;

}Motor_Params_typedef;





/******************************************************************************
* Defines
******************************************************************************/

#define PWM_MODE_6


#define ACCELTIME		100									// ms
#define SPDREF			3000								// Max spd open_loop	// 10k max no load 2V
#define SPDMIN			100									// Min spd. Abs min 22

//#define FREQ			32

#define PWM_FREQUENCY	32									// PWM carrier frequency kHz			!!!  2KHz MIN  !!!
#define PWM_RELOAD_K	1


#define PWM_DEADTIME	0									// Deadtime in clocks


//#define PWM_TIMER_FREQ		(42000/FREQ)

#define IRUNNING		0.4
//#define NUMBEROFPOLES	3.0

#define UMIN			2.0									// Min Voltage on start		(1 - 4)/8	(1A MAX)
//#define UDCBUS			5.0
#define UMAX			12.0								// Max Voltage,		V
#define VFCOEFF			10.0								// V/F Coeff;		Voltage per step	Step = 50 rev/min/8kHz	Default = 0.03

#define TPWM			(1000/FREQ)							// 40 kHz = 25us
#define TRELOAD			62


#define UMIN_FRAC		FRAC16((float)UMIN/UMAX)			// Umin fractional
#define UMAX_FRAC		FRAC16(1)							// Umax fractional
#define VFCOEFF_FRAC	FRAC16((float)VFCOEFF/UMAX)			// Coeff of voltage to speed calcs
#define ACCELCONST		((ACCELTIME*610000)/(SPDREF*TRELOAD^2))

#define SPDTOSTEP(a)	((a*TRELOAD)/610)					// Get step from speed
#define STEPTOSPD(a)	((a*610)/TRELOAD)					// Get speed from step
	
/******************************************************************************
* Global variables
******************************************************************************/
		
static volatile	u32					time_ms; 							//time variable												
				
				q15					U_ref_ol	= UMIN_FRAC;			// Open loop ref voltage    	~1500 MAX				
				u16					Spd_ref_ol	= SPDREF;				// Open loop ref speed
				u8					pwm_freq	= PWM_FREQUENCY;					// PWM carrier frequency
				u16					vfcoeff		= VFCOEFF;				// Voltage/speed coefficient open loop

				u8					closeloop	= OFF;

				q15					Ua;
				q15					Ub;

				q15					Kalpha, Kbeta;		

static			u16					angle; 								// angle: 0-360;			range: 0-360: 0-32767 s	

				u8					pwm_reload_flag;					//  					
				
				q15					step =  SPDTOSTEP(SPDMIN);			// SF16 Angle increment per open loop exec; 0 - 32766
				
				Motor_Params_typedef motor_params_init;						

				//PWM_buffer_typedef	PWM_buffer = {0};

/******************************************************************************
* Init and reset functions
******************************************************************************/


void	foo()
{
	u16 data = 5;
	
}


void	Motor_Params_Init()
{
		/*motor_params.motor_accel_time	=	(ACCELTIME*83)/TPWM;
		motor_params.motor_freq			=	FREQ;
		motor_params.motor_spdref		=	SPDREF;
		motor_params.motor_umin			=	UMIN_FRAC;
		motor_params.motor_vfcoeff		=	VFCOEFF;*/
}

//void	PWM_timer_init()
//{
//		
//		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// TIM1 clock enable	
//			
//		TIM1->PSC	 = 1;   					// prescaler 8	
//		TIM1->ARR	 = PWM_TIMER_FREQ; 				// PWM frequency Period
//		
//		TIM1->CCR1   = 0;   					// channel 1 pwm value 1
//		TIM1->CCR2   = 0;    					// channel 2 pwm value 2
//		TIM1->CCR3   = 0;     					// channel 3 pwm value 2
//		TIM1->CCR4   = PWM_TIMER_FREQ-10;				// channel 4 trigger value		
//
//
//		LL_TIM_EnableARRPreload(TIM1);		
//		TIM1->CR1	|= TIM_CR1_CMS_0;  			// center align mode 1		
//		TIM1->DIER  |= TIM_DIER_UIE;  			// interrupt enable
//
//		
//		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM2);	// channel 1 PWM1 mode
//		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM2);	// channel 2 PWM1 mode
//		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);	// channel 3 PWM1 mode
//		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);	
//
//		LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
//
//		   		
//		TIM1->CCMR1	|= TIM_CCMR1_OC1PE;  							// channel 1 oc1 preload enable		    	
//		TIM1->CCMR1	|= TIM_CCMR1_OC2PE;   							// channel 2 oc2 preload enable		    	
//		TIM1->CCMR2	|= TIM_CCMR2_OC3PE;   							// channel 3 oc3 preload enable
//		
//		
//		TIM1->CCER	|= TIM_CCER_CC1E;  			// channel 1 output pin enable			PE9		PA8		
//		TIM1->CCER	|= TIM_CCER_CC2E;   		// channel 2 output pin enable			PE11	PA9
//		TIM1->CCER	|= TIM_CCER_CC3E;    		// channel 3 output pin enable			PE13	PA10
//				
//	#ifdef PWM_MODE_6
//	
//		TIM1->CCER	|= TIM_CCER_CC1NE;  		// channel 1 output pin enable			PE8		PB13
//		TIM1->CCER	|= TIM_CCER_CC2NE;  		// channel 2 output pin enable			PE10	PB14
//		TIM1->CCER	|= TIM_CCER_CC3NE;  		// channel 3 output pin enable			PE12	PB15
//		
//	#endif // PWM_MODE_6
//
//		LL_TIM_OC_SetDeadTime(TIM1, PWM_DEADTIME);
//		
//		
//		LL_TIM_SetRepetitionCounter(TIM1, 0);
//
//		TIM1->EGR	|= TIM_EGR_UG;  			// init registers
//		TIM1->BDTR	|= TIM_BDTR_MOE;  			// main outut enable	
//		//TIM1->CR1	|= TIM_CR1_CEN;   			// timer enable
//	
//		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);		// Interrupt enable
//		NVIC_SetPriority(TIM1_UP_TIM10_IRQn, MOTOR_PWM_RELOAD_ISR_PRIORITY);
//		
//		
//
//
//
///* GPIO init ----------------------------------------------------------------------*/
//
//		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);	// GPIO clock
//		
//		LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };  			// GPIO init
//	
//		GPIO_InitStruct.Pin			= LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
//		GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
//		GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
//		GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
//		GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
//		GPIO_InitStruct.Alternate	= LL_GPIO_AF_1;
//		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//		
//	#ifdef PWM_MODE_6
//	
//		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
//
//		GPIO_InitStruct.Pin			= LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
//		GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
//		GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
//		GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
//		GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
//		GPIO_InitStruct.Alternate	= LL_GPIO_AF_1;
//		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//		
//	#endif // PWM_MODE_6
//	
//}

void	PWM_Init()
{
		LL_TIM_InitTypeDef		Tim_init_s	= {0};
		LL_TIM_OC_InitTypeDef	Tim_oc_s	= {0};




}

void	Motor_init()
{	
	//debug_init(); 
	//PWM_timer_init();

}



/* Functions -----------------------------------------------------------------*/
//
//static inline q15 mulf16(q15 a, q15 b)						/* Multiply two fractional SF16 numbers				*/
//{
//		return ((int32_t)a * b) >> 15;		
//}
//
//																					// Normalizes Frac 0 - 32767 to PWM timer ARR
//static inline void SVPWM_DC16_set(q15 T_u, q15 T_v, q15 T_w)		 
//{
//	PWM_buffer.PWM_A = mulf16(T_u, (q15)PWM_TIMER_FREQ);
//	PWM_buffer.PWM_B = mulf16(T_v, (q15)PWM_TIMER_FREQ);
//	PWM_buffer.PWM_C = mulf16(T_w, (q15)PWM_TIMER_FREQ);		
//}
//
//																					// NXP MCLIB					Exec time ~1.7us
//void SVPWM_Mclib_Fract(q15 Ualpha, q15 Ubeta)									
//{
//	
//	#define SQF3	    FRAC16(0.866f)		// 0.866	frac
//	#define Tpf			FRAC16(1)			// 1		frac
//	
//	q15			A, B;
//	q15			Uref1, Uref2, Uref3;
//	q15			t_1, t_2;
//	q15			tar[4];
//	
//	uint8_t			seq;
//	
//	
//	
//	B =	Ubeta >> 1; 						//  Ubeta  * 0.5
//	A = mulf16(Ualpha, SQF3);	 			//  Ualpha * SQRT(3) * 0.5			exec 1.04us
//	
//	
//	Uref1 =   Ubeta;
//	Uref2 =	  A - B;
//	Uref3 = -(A + B);
//	
//	
//	if (Uref3 <= 0)
//	{
//		if (Uref2 > 0)
//		{
//			if (Uref1 <= 0)	{t_1 = -Uref1; t_2 = -Uref3; seq = 0b00110110; }	//              S6
//			else			{t_1 =  Uref1; t_2 =  Uref2; seq = 0b00111001; }	//              S1
//		}
//		else				{t_1 = -Uref3; t_2 =  B - A; seq = 0b00101101; }	//              S2
//	}
//	else
//	{
//		if (Uref2 <= 0)
//		{
//			if (Uref1 <= 0)	{t_1 =  B - A; t_2 = -Uref1; seq = 0b00011011; }	//              S4
//			else			{t_1 =  Uref3; t_2 =  Uref1; seq = 0b00011110; }	//              S3
//		}
//		else				{t_1 =  Uref2; t_2 = Uref3; seq  = 0b00100111; }	//              S5
//	}
//	
//	
//	tar[1] = (Tpf - t_1 - t_2) >> 1;
//	tar[2] = tar[1] + t_1;
//	tar[3] = tar[2] + t_2;
//	
//	SVPWM_DC16_set(tar[seq >> 4], tar[seq >> 2 & 0x3], tar[seq & 0x3]);
//}


//
//void Open_Loop_Ramp() 
//{ 
//	static q15  Uvf;
//	static u16  accel_const = 0;
//
//	if (step < SPDTOSTEP(Spd_ref_ol))						// V/f control
//	{
//		accel_const++;
//		if (accel_const == ACCELCONST)
//		{
//			step++;
//			Uvf = step * vfcoeff;
//
//			if		(Uvf < UMIN_FRAC);
//			else if (Uvf > UMAX_FRAC);
//			else	   U_ref_ol = Uvf;
//
//			accel_const = 0;
//		}
//	}
//}
//
//void Open_Loop_Resolver()						// Manages open loop start;	Exec time ~4us						
//{
//	Kalpha	= arm_cos_q15((q15)angle);			// sin:						range:	0-1: 0-32767 s
//	Kbeta	= arm_sin_q15((q15)angle);	
//
//	Ua		= mulf16(Kalpha, U_ref_ol);			// Ua = Ka * Uref/Umax		range:	0-1: 0-32767 s
//	Ub		= mulf16(Kbeta,  U_ref_ol);
//		
//	if (closeloop)
//	{
//		extern f32 uSalpha;
//		extern f32 uSbeta;
//
//		extern f32 theta;
//
//		Kalpha	= arm_cos_q15((q15)(theta+7)*91);		// sin:						range:	0-1: 0-32767 s
//		Kbeta	= arm_sin_q15((q15)(theta+7)*91);	
//
//		Ua		= mulf16(Kalpha, U_ref_ol);			// Ua = Ka * Uref/Umax		range:	0-1: 0-32767 s
//		Ub		= mulf16(Kbeta,  U_ref_ol);
//
//
//		SVPWM_Mclib_Fract(Ua, Ub);
//		//SVPWM_Mclib_Fract((s16)(uSalpha*0.25*32768), (s16)(uSbeta*0.25*32768));					// takes voltages Ua, Ub:	range:	Umin-Umax: 0-32767 s
//	}
//	else
//	{
//		SVPWM_Mclib_Fract(Ua, Ub); 					// takes voltages Ua, Ub:	range:	Umin-Umax: 0-32767 s
//	}		
//	
//	Open_Loop_Ramp();
//
//		angle += step;
//	if (angle >= FRAC16(1)) angle = angle - FRAC16(1);	
//}
//
//
//
//






	

