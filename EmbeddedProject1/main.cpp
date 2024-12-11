
/******************************************************************************
* Global Defines
******************************************************************************/

#define MOTOR_PWM_RELOAD_ISR_PRIORITY			1
#define MOTOR_CURRENT_CONTROL_ISR_PRIORUTY		2
#define MOTOR_SPEED_CONTROL_ISR_PRIORITY		4



/******************************************************************************
* Includes
******************************************************************************/
#include "main.h"


#include "global.h"
#include "lcd.h"
#include "menu.h"
#include "adc.h"

#include "pmsm_sensorless_cntrl.h"
#include "motor_params.h"

#include "lcd_menu_items.h"

#include "img.h"
//#include "qPIDs.h"

#ifdef __cplusplus
extern "C"
#endif

/******************************************************************************
* Local types
******************************************************************************/

typedef void (*FcnPointer)(void);               /* pointer to function */

enum {DATA_A, DATA_B, DATA_C};
enum {CLEAR};

typedef enum 
{
	START_INIT,
	START_RAMP_OL,	
	RUN_ESTIM,
	STOP,
	STOPPED,
}Motor_State_enum;

typedef enum
{
	START_RAMP_NO_ESTIM,
	START_RAMP_ESTIM,	
}Motor_Start_State_enum;



/******************************************************************************
* Local function prototypes
******************************************************************************/
	
void System_Init(void);
static	void MX_GPIO_Init(void);
u32 LCD_Prop_Speed_Handler(u32);
u32 LCD_Prop_Empty(u32);

void Motor_Start_Ramp_OL();
void Motor_Start_Init();
void Motor_Run_Estim();
void Motor_Stop();
void Estim_Init();
f32  PID_Controller(f32, f32, PID_params_typedef*);


FcnPointer			Motor_Control_fcn[] = { Motor_Start_Init,
											Motor_Start_Ramp_OL,											
											Motor_Run_Estim,
											Motor_Stop };


/******************************************************************************
* Defines
******************************************************************************/

#define DATABUFFERSIZE	1000

#define DATALOGSIZE     400						// Array size for data analisis

//#define LCD_Class_mode 




/* Private macro ------------------------------------------------------------*/

//#define SVM_MODULATION_MI_HALF  (0.866025f)

//#define RPMTORADS				(6.28318530f/60.0f)



/******************************************************************************
* Local Variables
******************************************************************************/

					Motor_State_enum	motorControlState		= STOP;
					Motor_Start_State_enum
										motorStartState			= START_RAMP_NO_ESTIM;
					u8					appState				= ON;


					PWM_buffer_typedef  PWM;			

					u8					closeLoopCntrl			= ON;
					u8					datalog					= DISABLE;
		
					u8					dataTransmit			= OFF;
					u8					estimInit;

					f32					motorPower;

					u8					dataToBeSent			= DATA_A;
					u8					sendDataFlag			= CLEAR;

static				u32					dataCounter				= 0;
static				u16					counter					= 0;

					PMSM_SMObserverBemf_typedef				
										SMOStruct;

					PMSM_SpeedAdaptScheme_params_typedef	
										AdaptSchStruct;

					PMSM_Decoupl_typedef					
										DecouplStruct;

					PID_params_typedef	IDPIDParams;
					PID_params_typedef	IQPIDParams;
					PID_params_typedef	SpeedPIDParams;

					MC_3PhSyst			I_abc_meas;
					MC_2PhSyst			I_alphabeta_meas;

					MC_2PhSyst			U_alphabeta;
					MC_2PhSyst			U_alphabeta_prev;
					
					MC_DQSyst			I_DQ;
					MC_DQSyst			I_DQ_desired;					

					MC_DQSyst			U_DQ;
					MC_DQSyst			U_DQ_PID_out;
					MC_DQSyst			U_DQ_decoup;

					MC_Angle			motor_pos;
					MC_Angle			motor_pos_est;
					MC_Angle			motor_pos_gen;
					MC_Angle			motor_pos_predict;

					f32					amplBEMF;										
					
					f32					g1k1Tsmult				= MOTOR_EST_G1_K1_TS_MULTIPLE;
					f32					g2k1Tsmult				= MOTOR_EST_G2_K1_TS_MULTIPLE;
					
					f32					i_d_pid_p				= MOTOR_PID_I_D_PROP;
					f32					i_d_pid_i				= MOTOR_PID_I_D_INTEG;
					f32					i_d_pid_d				= MOTOR_PID_I_D_DIFF;

					f32					i_q_pid_p				= MOTOR_PID_I_Q_PROP;
					f32					i_q_pid_i				= MOTOR_PID_I_Q_INTEG;
					f32					i_q_pid_d				= MOTOR_PID_I_Q_DIFF;

					f32					spd_pid_p				= MOTOR_PID_SPEED_PROP;			
					f32					spd_pid_i				= MOTOR_PID_SPEED_INTEG;		
					f32					spd_pid_d				= MOTOR_PID_SPEED_DIFF;

					f32					motor_speed_ramp_end	= 5000*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS;
					f32					motor_speed_accel_ol	= MOTOR_ACCEL_SPEED_OPEN_L_EL;
					f32					motor_speed_accel_cl	= MOTOR_ACCEL_SPEED_CLOSE_L_EL;

					f32					motor_speed_ramp		= 100*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS;
					f32					motor_speed_decoup_input;
					f32					motor_speed_smo_on_threshold
																= MOTOR_TRESHOLD_SPEED_SMO_ON_EL;
					f32					motor_speed_smo_fb_enable
																= MOTOR_TRESHOLD_SPEED_SPEST_ON_EL;
					f32					motor_speed_estim;
					f32					motor_speed_ref			= 6000*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS;

extern				menuItem			Null_Menu;					
extern				menuItem*			SelectedMenuItem; 
extern				menuItem*			PreviousMenuItem; 
extern				u8					KEYMODE;


					f32						Id_buffer[DATABUFFERSIZE];
					f32						Iq_buffer[DATABUFFERSIZE];
					f32						Id_ref_buffer[DATABUFFERSIZE];				
					f32						Iq_ref_buffer[DATABUFFERSIZE];
					f32							ia_buffer[DATABUFFERSIZE];
					f32							ib_buffer[DATABUFFERSIZE];
					f32							ic_buffer[DATABUFFERSIZE];
					f32					 speed_est_buffer[DATABUFFERSIZE];
					

/* Timers -------------------------------------------------------------------*/

Timer_typedef		data_log_tmr	= {8000,	TMR_AUROSET_OFF,	TMR_SET};
Timer_typedef		lcd_tmr			= {100,		TMR_AUROSET_ON,		TMR_SET};




/******************************************************************************
* PERIPH INIT functions
******************************************************************************/

void EXTI_Motor_Control_Init()
{
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
	
	NVIC_SetPriority(EXTI0_IRQn, MOTOR_CURRENT_CONTROL_ISR_PRIORUTY);
	NVIC_EnableIRQ(EXTI0_IRQn);		
	
}

void Speed_Control_Timer_ISR_Init()
{
	LL_APB1_GRP1_EnableClock(RCC_APB1ENR_TIM6EN);

	LL_TIM_InitTypeDef Speed_Control_Timer;
		
	Speed_Control_Timer.ClockDivision		= 1;
	Speed_Control_Timer.Prescaler			= 84;
	Speed_Control_Timer.Autoreload			= MOTOR_SPEED_SAMPL_PER*1000000;
	Speed_Control_Timer.CounterMode			= LL_TIM_COUNTERMODE_DOWN;

	LL_TIM_Init(TIM6, &Speed_Control_Timer);

	LL_TIM_EnableIT_UPDATE(TIM6);   

	NVIC_EnableIRQ(TIM6_DAC_IRQn);		// Interrupt enable
	NVIC_SetPriority(TIM6_DAC_IRQn, MOTOR_SPEED_CONTROL_ISR_PRIORITY);

	LL_TIM_EnableCounter(TIM6);
}

void PWM_timer_init()
{
		
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// TIM1 clock enable	
			
		TIM1->PSC	 = 1;   					// prescaler 8	
		TIM1->ARR	 = PWM_TIMER_FREQ_ARR; 				// PWM frequency Period
		
		TIM1->CCR1   = 0;   					// channel 1 pwm value 1
		TIM1->CCR2   = 0;    					// channel 2 pwm value 2
		TIM1->CCR3   = 0;     					// channel 3 pwm value 2
		TIM1->CCR4   = PWM_TIMER_FREQ_ARR-10;				// channel 4 trigger value		


		LL_TIM_EnableARRPreload(TIM1);		
		TIM1->CR1	|= TIM_CR1_CMS_0;  			// center align mode 1		
		TIM1->DIER  |= TIM_DIER_UIE;  			// interrupt enable

		
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM2);	// channel 1 PWM1 mode
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM2);	// channel 2 PWM1 mode
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);	// channel 3 PWM1 mode
		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);	

		LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);

		   		
		TIM1->CCMR1	|= TIM_CCMR1_OC1PE;  							// channel 1 oc1 preload enable		    	
		TIM1->CCMR1	|= TIM_CCMR1_OC2PE;   							// channel 2 oc2 preload enable		    	
		TIM1->CCMR2	|= TIM_CCMR2_OC3PE;   							// channel 3 oc3 preload enable
		
		
		TIM1->CCER	|= TIM_CCER_CC1E;  			// channel 1 output pin enable			PE9		PA8		
		TIM1->CCER	|= TIM_CCER_CC2E;   		// channel 2 output pin enable			PE11	PA9
		TIM1->CCER	|= TIM_CCER_CC3E;    		// channel 3 output pin enable			PE13	PA10
				

	
		TIM1->CCER	|= TIM_CCER_CC1NE;  		// channel 1 output pin enable			PE8		PB13
		TIM1->CCER	|= TIM_CCER_CC2NE;  		// channel 2 output pin enable			PE10	PB14
		TIM1->CCER	|= TIM_CCER_CC3NE;  		// channel 3 output pin enable			PE12	PB15


		LL_TIM_OC_SetDeadTime(TIM1, PWM_DEADTIME);
		
		
		LL_TIM_SetRepetitionCounter(TIM1, 0);

		TIM1->EGR	|= TIM_EGR_UG;  			// init registers
		TIM1->BDTR	|= TIM_BDTR_MOE;  			// main outut enable	
		
	
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);		// Interrupt enable
		NVIC_SetPriority(TIM1_UP_TIM10_IRQn, MOTOR_PWM_RELOAD_ISR_PRIORITY);


/* GPIO init ----------------------------------------------------------------------*/

		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);	// GPIO clock
		
		LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };  			// GPIO init
	
		GPIO_InitStruct.Pin			= LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
		GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate	= LL_GPIO_AF_1;
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);		

	
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

		GPIO_InitStruct.Pin			= LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
		GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate	= LL_GPIO_AF_1;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
}

void System_Init()
{
	LL_FLASH_EnableDataCache();
	LL_FLASH_EnableInstCache();
	LL_FLASH_EnablePrefetch();

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

	if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)	Error_Handler();

	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while(LL_RCC_HSE_IsReady() != 1){}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while(LL_RCC_PLL_IsReady() != 1){}

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){}
	
	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
	LL_SetSystemCoreClock(168000000);



/* Sys_tick timer init -----------------------------------------------*/
	
	#define F_CPU 		168000000UL		// Тактовая у нас 72МГЦ
	#define TimerTick  	F_CPU/1000		// Нам нужен килогерц
	
	SysTick_Config(TimerTick);

	MX_GPIO_Init();

}

void MX_GPIO_Init()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate	= LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  
}

void Error_Handler()
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	_ERROR;
  /* USER CODE END Error_Handler_Debug */
}






/******************************************************************************
* Local functions
******************************************************************************/

void Data_Log_tmr()
{
	if (check_tmr(&data_log_tmr) == 0) return;	
	/*-----------------------------------------------*/	

	datalog = ENABLE;
}

inline void Data_Log()
{
	if (datalog == ENABLE)
	{		
		if (counter < DATABUFFERSIZE)
		{
			Id_buffer[counter]			= I_DQ.d;
			Iq_buffer[counter]			= I_DQ.q;
			Id_ref_buffer[counter]		= I_DQ_desired.d;
			Iq_ref_buffer[counter]		= I_DQ_desired.q;
			ia_buffer[counter]			= I_abc_meas.a;
			ib_buffer[counter]			= I_abc_meas.b;
			ic_buffer[counter]			= I_abc_meas.c;
			speed_est_buffer[counter]	= motor_speed_estim;


			counter++;
		}
		else
		{
			//Motor_Stop();
			volatile u8 STOP_HERE = 1;
		}
	}
}




/* Main APP -----------------------------------------------------------------*/

void APP_Control()
{
	static u8 last_state = OFF;


	if (appState != last_state)	// Если состояние изменилось выполняем
	{
		if (appState == ON)
		{		
				motorControlState = START_INIT;
				LL_TIM_EnableCounter(TIM1);				
		}
		else
		{		
				Motor_Stop();
				motorControlState = STOP;	
		}
		last_state = appState;
	}	
}

f32 PID_Controller(f32 desiredValue, f32 measuredValue, PID_params_typedef* PID_params)
{
	f32 error, out;

	error	= desiredValue-measuredValue;
	out		= arm_pid_f32(&PID_params->arm_pid_params, error);

	if		(out > PID_params->posPIDlimit) out = PID_params->posPIDlimit;
	else if (out < PID_params->negPIDlimit) out = PID_params->negPIDlimit;

	return	out;
}




/* Motor Control ------------------------------------------------------------*/

inline void PWM_Update()
{
	LL_TIM_OC_SetCompareCH1(TIM1, PWM.A);   	
	LL_TIM_OC_SetCompareCH2(TIM1, PWM.B);    
	LL_TIM_OC_SetCompareCH3(TIM1, PWM.C);
}
	
// NXP MCLIB					Exec time ~1.7us																				
void SVPWM_Mclib_Fract(q15 Ualpha, q15 Ubeta)									
{	
	#define SQF3	    FRAC16(0.866025403f) // 0.866
	#define Tp			FRAC16(1)			 // 1	
	
	q15			A, B;
	q15			Uref1, Uref2, Uref3;
	q15			t_1, t_2;
	q15			tar[4];
	
	u8			seq;	
	
	B =	Ubeta >> 1; 						//  Ubeta  * 0.5
	A = mulf16(Ualpha, SQF3);	 			//  Ualpha * SQRT(3) * 0.5	
	
	Uref1 =   Ubeta;
	Uref2 =	  A - B;
	Uref3 = -(A + B);	
	
	if (Uref3 <= 0)
	{
		if (Uref2 > 0)
		{
			if (Uref1 <= 0)	{t_1 = -Uref1; t_2 = -Uref3; seq = 0b00110110; }	//              S6
			else			{t_1 =  Uref1; t_2 =  Uref2; seq = 0b00111001; }	//              S1
		}
		else				{t_1 = -Uref3; t_2 =  B - A; seq = 0b00101101; }	//              S2
	}
	else
	{
		if (Uref2 <= 0)
		{
			if (Uref1 <= 0)	{t_1 =  B - A; t_2 = -Uref1; seq = 0b00011011; }	//              S4
			else			{t_1 =  Uref3; t_2 =  Uref1; seq = 0b00011110; }	//              S3
		}
		else				{t_1 =  Uref2; t_2 =  Uref3; seq = 0b00100111; }	//              S5
	}	
	
	tar[1] = (Tp - t_1 - t_2) >> 1;
	tar[2] = tar[1] + t_1;
	tar[3] = tar[2] + t_2;	

	PWM.A = mulf16(tar[seq >> 4],			(q15)PWM_TIMER_FREQ_ARR);
	PWM.B = mulf16(tar[seq >> 2 & 0x3],		(q15)PWM_TIMER_FREQ_ARR);
	PWM.C = mulf16(tar[seq & 0x3],			(q15)PWM_TIMER_FREQ_ARR);
}

f32  Calc_Theta(f32 sinest, f32 cosest)
{
	if	   (cosest>0)
	{
		if (sinest>0)
		{
			return ((atan(sinest/cosest))*180.0/3.14192);
		}
		else
		{
			return ((atan(sinest/cosest))*180.0/3.14192)+360;
		}		
	}
	else 
	{
			return ((atan(sinest/cosest))*180.0/3.14192)+180;
	}
}

void Estim_Init()
{
	f32 uVectorInit;

	SMOStruct.I_est.alpha	=  I_alphabeta_meas.alpha;
	SMOStruct.I_est.beta	=  I_alphabeta_meas.beta;

	AdaptSchStruct.integPartK_1 
							=  motor_speed_ramp;

	uVectorInit				=  MOTOR_BEMF_CONST_mV_RPM * 0.001 * motor_speed_ramp * 9.5;

	motor_speed_estim		=  motor_speed_ramp;

	amplBEMF				=  uVectorInit;

	SMOStruct.bemf.alpha	= -uVectorInit * motor_pos_gen.sin;
	SMOStruct.bemf.beta	=  uVectorInit * motor_pos_gen.cos;

	U_alphabeta_prev.alpha	=  U_alphabeta.alpha;
	U_alphabeta_prev.beta	=  U_alphabeta.beta;

}

void Estim_Processing()		/* exec time: -O3 9,5us, -O0 11us	~20 multiplications and 19 additions and 3 division			*/
{	

	/* SMO Observer calculations-------------------------------------------------*/

	SMOStruct.g1k1Ts	   = amplBEMF * g1k1Tsmult;
	SMOStruct.g2k1Ts	   = amplBEMF * g2k1Tsmult;

	PMSM_SMObserverBemf(&SMOStruct, &I_alphabeta_meas, &U_alphabeta_prev, motor_speed_estim);
	
	/* Adaptive scheme -------------------------------------------------------*/

	motor_speed_estim = PMSM_AdaptSch(&SMOStruct.mZTs, &SMOStruct.bemf, &AdaptSchStruct, amplBEMF);
	
	/* BEMF and position------------------------------------------------------*/

	amplBEMF   = asm_sqrt(SMOStruct.bemf.alpha * SMOStruct.bemf.alpha + SMOStruct.bemf.beta * SMOStruct.bemf.beta);	

	motor_pos_est.cos = -SMOStruct.bemf.beta  / amplBEMF;
	motor_pos_est.sin =  SMOStruct.bemf.alpha / amplBEMF;

	//thetaest   = Calc_Theta(motor_pos_est.sin, motor_pos_est.cos);	
}

void Motor_Current_Control()
{
	/* PMSM_SL_VecInRotatedOut -----------------------------------------------*/

	f32 omegatemp;
	f32 temp;

	omegatemp					= -motor_speed_decoup_input  * SMOStruct.Ts;

	motor_pos_predict.sin		=  motor_pos.sin - omegatemp * motor_pos.cos;
	motor_pos_predict.cos		=  motor_pos.cos + omegatemp * motor_pos.sin;

	/* DQ Decoupling ---------------------------------------------------------*/

	PMSM_DQDecoupl(&U_DQ_decoup, &I_DQ, &DecouplStruct, motor_speed_decoup_input);

	/* PI Controller ---------------------------------------------------------*/
		   
	temp						=  (HW_DC_BUS_VOLTAGE*SVM_MODULATION_MI_HALF);

	/*   D   */

	IDPIDParams.posPIDlimit	=  temp - U_DQ_decoup.d;
	IDPIDParams.negPIDlimit	= -temp - U_DQ_decoup.q;

	U_DQ_PID_out.d				=  PID_Controller(I_DQ_desired.d, I_DQ.d, &IDPIDParams);

	U_DQ.d						=  U_DQ_PID_out.d + U_DQ_decoup.d;

	temp						=  asm_sqrt(temp * temp - U_DQ.d * U_DQ.d);

	/*   Q   */

	IQPIDParams.posPIDlimit	=  temp - U_DQ_decoup.q;
	IQPIDParams.negPIDlimit	= -temp - U_DQ_decoup.q;

	U_DQ_PID_out.q				=  PID_Controller(I_DQ_desired.q, I_DQ.q, &IQPIDParams);

	U_DQ.q						=  U_DQ_PID_out.q + U_DQ_decoup.q;

	/* Inverse Park transform -------------------------------------------------*/

	arm_inv_park_f32(U_DQ.d, U_DQ.q, &U_alphabeta.alpha, &U_alphabeta.beta,
					 motor_pos_predict.sin, motor_pos_predict.cos);

	/* SVPWM Modulation -------------------------------------------------------*/

	SVPWM_Mclib_Fract(float_to_q15(U_alphabeta.alpha * (1.0/HW_DC_BUS_VOLTAGE)),\
					  float_to_q15(U_alphabeta.beta  * (1.0/HW_DC_BUS_VOLTAGE)));
}

inline void Vector_Rotate()
{
	f32 omegatemp;
	
	omegatemp					= -motor_speed_ramp * SMOStruct.Ts;

	motor_pos_gen.sin			= motor_pos_gen.sin - omegatemp * motor_pos_gen.cos;
	motor_pos_gen.cos			= motor_pos_gen.cos + omegatemp * motor_pos_gen.sin;

}

void Speed_Ramp(f32* speed_act, f32 speed_desired, f32 accel)
{
	f32 temp_speed;

	if (speed_desired > *speed_act)
	{
		temp_speed = *speed_act + accel;

		if (temp_speed > speed_desired)
		{
			*speed_act = speed_desired;
		}
		else
		{
			*speed_act = temp_speed;
		}
	}
	else
	{
		temp_speed = *speed_act - accel;

		if (temp_speed < motor_speed_ramp_end)
		{
			*speed_act = speed_desired;
		}
		else
		{
			*speed_act = temp_speed;
		}
	}	
}




/* Motor State Functions -----------------------*/

void Motor_Start_Ramp_OL()
{
	U_alphabeta_prev.alpha = U_alphabeta.alpha;
	U_alphabeta_prev.beta  = U_alphabeta.beta;

	Motor_Current_Control();	

	/* Vector rotation  -------------------------------------------------------*/	
		
	Vector_Rotate();

	if (motorStartState == START_RAMP_NO_ESTIM)
	{
		// Enable estimation if get to smo_on_speed_thres. Still don't use feedback

		if (motor_speed_ramp > motor_speed_smo_on_threshold && closeLoopCntrl == ON)	
		{
			Estim_Init();
			motorStartState = START_RAMP_ESTIM;
		}
	}
	else
	{
		Estim_Processing();
			
		// Enable speed estimation feedback if get to smo_speed_fb_thres
		if (motor_speed_ramp >= motor_speed_smo_fb_enable)
		{
			I_DQ_desired.d			=  0.0;
			I_DQ_desired.q			= -MOTOR_CURRENT_RUNING0;

			motorControlState		=  RUN_ESTIM;
		}
		else
		{
			AdaptSchStruct.integPartK_1 = motor_speed_ramp;
		}
	}

	motor_speed_decoup_input = motor_speed_ramp;
}

void Motor_Run_Estim()
{
	U_alphabeta_prev.alpha = U_alphabeta.alpha;
	U_alphabeta_prev.beta  = U_alphabeta.beta;

	Motor_Current_Control();

	Estim_Processing();

	motor_speed_decoup_input = motor_speed_estim;

	//datalog = ENABLE;
}

void Motor_Start_Init()
{
	/* Enable PWM Timer and outputpins ----------------------------------------*/

	LL_TIM_OC_SetCompareCH1(TIM1, 0);   	// channel 1 DC 0 - ARR
	LL_TIM_OC_SetCompareCH2(TIM1, 0);    	// channel 2 DC
	LL_TIM_OC_SetCompareCH3(TIM1, 0);     	// channel 3 DC	
	
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8,  LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);	

	/* Initiate Open Loop parameters-------------------------------------------*/

	SMOStruct.Ts					= MOTOR_SMO_TS;
	SMOStruct.k1Ts					= MOTOR_SMO_K1*MOTOR_SMO_TS;
	SMOStruct.a11Ts				= MOTOR_SMO_A11TS;
	SMOStruct.a12Ts				= MOTOR_SMO_A12TS;

	AdaptSchStruct.integGain		= MOTOR_AD_SCH_INTEG_GAIN;
	AdaptSchStruct.integPartK_1	= 0;

	DecouplStruct.DECOa12			= MOTOR_DECOUP_A12;
	DecouplStruct.DECOa21			= MOTOR_DECOUP_A21;

	estimInit						= 1;
	motor_speed_ramp				= 0;	

	motor_pos_gen.sin				= 0.0;
	motor_pos_gen.cos				= 1.0;

	motor_pos.sin					= 0.0;
	motor_pos.cos					= 1.0;	
	
	I_DQ_desired.d					= MOTOR_CURRENT_STARTING;
	I_DQ_desired.q					= 0.0;

	IDPIDParams.arm_pid_params.Kp = i_d_pid_p;
	IDPIDParams.arm_pid_params.Ki = i_d_pid_i;
	IDPIDParams.arm_pid_params.Kd = i_d_pid_d;

	IQPIDParams.arm_pid_params.Kp = i_q_pid_p;
	IQPIDParams.arm_pid_params.Ki = i_q_pid_i;
	IQPIDParams.arm_pid_params.Kd = i_q_pid_d;

	arm_pid_init_f32(&IDPIDParams.arm_pid_params,1);
	arm_pid_init_f32(&IQPIDParams.arm_pid_params,1);	
	
	SpeedPIDParams.arm_pid_params.Kp =  spd_pid_p;
	SpeedPIDParams.arm_pid_params.Ki =  spd_pid_i;
	SpeedPIDParams.arm_pid_params.Kd =  spd_pid_d;

	SpeedPIDParams.posPIDlimit	   =  MOTOR_CURRENT_MAX;
	SpeedPIDParams.negPIDlimit	   = -MOTOR_CURRENT_MAX;

	arm_pid_init_f32(&SpeedPIDParams.arm_pid_params,1);

	motor_speed_decoup_input = 0;

	Motor_Current_Control();

	/* Alignment stage ---------------------------------*/

	static u32 alignment_counter;
	alignment_counter++;
	if (alignment_counter == MOTOR_ALIGNMENT_DURATION)
	{
		motorControlState = START_RAMP_OL;
		alignment_counter = 0;
	}
}

void Motor_Stop()
{
	if (motorControlState == STOPPED) return;	

	LL_TIM_OC_SetCompareCH1(TIM1, 0);   	// channel 1 DC 0 - ARR
	LL_TIM_OC_SetCompareCH2(TIM1, 0);    	// channel 2 DC
	LL_TIM_OC_SetCompareCH3(TIM1, 0);     	// channel 3 DC

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8,  LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8  | LL_GPIO_PIN_9  | LL_GPIO_PIN_10);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
	
	LL_TIM_DisableCounter(TIM1);

	motor_speed_ramp				= 0;
	AdaptSchStruct.integPartK_1	= 0;
	motor_speed_estim				= 0;

	I_DQ.d							= 0;
	I_DQ.q							= 0;

	motorControlState = STOPPED;
}




/* LCD Menu -----------------------------------------------------------------*/

void LCD_Clear()
{
	extern u16 maxX, maxY;

	LCD_Fill_Screen(0, maxX, 0, maxY, BACKGROUND);	
}

void Start_Menu()
{
	SelectedMenuItem = (menuItem*)&menu_motor_cntrl;	
}


void LCD_Draw_Interface()
{	
	LCD_Clear();
	LCD_DrawHLine(0,17,399,BLACK);
	LCD_DrawHLine(0,18,399,BLACK);
	LCD_Draw_String(150, 2, BLACK, BACKGROUND,	(char*)"Main",		FONTSIZE);
}

void LCD_Draw_Menu_Interface()
{	
	#define SELECTAREALENGTH	8
	#define TEXTAREALENGTH		11

	#define RECTWIDTH			(FONT_Y*FONTSIZE+3)

	#define RECTXOFFSET			(-1)
	#define RECTYOFFSET			(-2)

	if (PreviousMenuItem != &Null_Menu)
	{
		LCD_Draw_Rect(PreviousMenuItem->Prop->x_pos+RECTXOFFSET,				PreviousMenuItem->Prop->y_pos+RECTYOFFSET,	FONT_X*FONTSIZE*TEXTAREALENGTH, \
			RECTWIDTH, BACKGROUND);
	}

	if (KEYMODE == KEYMODE_NORMAL)
	{
		LCD_Draw_Rect(SelectedMenuItem->Prop->x_pos+RECTXOFFSET+PROPDATAOFFSET,	SelectedMenuItem->Prop->y_pos+RECTYOFFSET,	FONT_X*FONTSIZE*SELECTAREALENGTH,\
			RECTWIDTH,	BACKGROUND);	// clear rect data area
		
		LCD_Draw_Rect(SelectedMenuItem->Prop->x_pos+RECTXOFFSET,				SelectedMenuItem->Prop->y_pos+RECTYOFFSET,	FONT_X*FONTSIZE*TEXTAREALENGTH,	 \
			RECTWIDTH,	BLACK);			// draw  tect text area
	}
	else
	{
		LCD_Draw_Rect(SelectedMenuItem->Prop->x_pos+RECTXOFFSET,				SelectedMenuItem->Prop->y_pos+RECTYOFFSET,	FONT_X*FONTSIZE*TEXTAREALENGTH,	 \
			RECTWIDTH,	BACKGROUND);	// clear rect text area
		
		LCD_Draw_Rect(SelectedMenuItem->Prop->x_pos+RECTXOFFSET+PROPDATAOFFSET,	SelectedMenuItem->Prop->y_pos+RECTYOFFSET,	FONT_X*FONTSIZE*SELECTAREALENGTH,\
			RECTWIDTH,	BLACK);			// draw data area
	}
}

void LCD_Draw_Menu_Item(menuItem* Menu)
{	
											// Отрисовываем текст элемента меню
	LCD_Draw_String(Menu->Prop->x_pos, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color, Menu->Prop->text1, FONTSIZE);
											// Очищаем место под текст или переменную
	LCD_Draw_String(Menu->Prop->x_pos+PROPDATAOFFSET, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color,(char*)"      ", FONTSIZE);
	
	if		(Menu->Prop->mode == DATA_MODE)	// Если показываем число, производим преобразование в строку и записываем в поле param1
	{	
		f32 tmp;
		switch(Menu->Prop->data_type)
		{
			case TYPE_U8:		tmp = (f32)(*( u8*)Menu->Prop->data_ptr); break;
			case TYPE_U16:		tmp = (f32)(*(u16*)Menu->Prop->data_ptr); break;
			case TYPE_U32:		tmp = (f32)(*(u32*)Menu->Prop->data_ptr); break;
			case TYPE_S8:		tmp = (f32)(*( s8*)Menu->Prop->data_ptr); break;
			case TYPE_S16:		tmp = (f32)(*(s16*)Menu->Prop->data_ptr); break;
			case TYPE_S32:		tmp = (f32)(*(s32*)Menu->Prop->data_ptr); break;
			case TYPE_FLOAT:	tmp = (f32)(*(f32*)Menu->Prop->data_ptr); break;
			case TYPE_DOUBLE:	tmp = (f32)(*(d64*)Menu->Prop->data_ptr); break;
		}
		
		ftoa((tmp*Menu->Prop->data_multiplier), Menu->Prop->param1, 5);		
		
		LCD_Draw_String(Menu->Prop->x_pos+PROPDATAOFFSET, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color, Menu->Prop->param1, FONTSIZE);
	}
	else if (Menu->Prop->mode == TEXT_MODE) // Если показываем состояние (ON, OFF, ENABLE etc) показываем поля param(n) в зависимости от data
	{
		switch(*(u8*)Menu->Prop->data_ptr)
		{
			case 0:	LCD_Draw_String(Menu->Prop->x_pos+PROPDATAOFFSET, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color, Menu->Prop->param1, FONTSIZE); break;
			case 1: LCD_Draw_String(Menu->Prop->x_pos+PROPDATAOFFSET, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color, Menu->Prop->param2, FONTSIZE); break;
			case 2: LCD_Draw_String(Menu->Prop->x_pos+PROPDATAOFFSET, Menu->Prop->y_pos, Menu->Prop->color, Menu->Prop->back_color, Menu->Prop->param3, FONTSIZE); break;
		}	
	}	
}

void LCD_Draw_Menu(menuItem* Menu_Items[])
{	
	static u8 current_page = P_MAIN;

	if (SelectedMenuItem->Prop->page != current_page)	// Если перешли на другую страницу - очищаем дисплей
	{
		current_page = SelectedMenuItem->Prop->page;
		LCD_Clear();
		switch(current_page)
		{
			case P_MAIN:	LCD_Draw_String(150, 0, BLACK, BACKGROUND,	(char*)"Main",		FONTSIZE); break;
			case P_OPTIONS: LCD_Draw_String(150, 0, BLACK, BACKGROUND,	(char*)"Options",	FONTSIZE); break;		
		}
	}

	u8 i = 0;
	while ((void*)Menu_Items[i] != END)
	{
		if (Menu_Items[i]->Prop->page == current_page)
		{
			LCD_Draw_Menu_Item(Menu_Items[i]);
		}		
		i++;
	}
}



void LCD_Refresh()
{		
	if (check_tmr(&lcd_tmr) == 0) return;	
	/*-----------------------------------------------*/		
		
		Key_Menu();									// Опрос кнопок и хождение по меню									
		
		LCD_Draw_Menu_Interface();					// Отрисовка элементов интерфейса
		
		LCD_Draw_Menu(menu_items);					// Отрисовка текста элементов меню

		


		

		//ADC_Start_Triple_Conversion();

		//char	buf[11];
		//		
		//f32 temp = -(f32)adc_inj_c * 0.00517f;
		//
		//

		//ftoa(temp, buf, 5);

		//LCD_Draw_String(0, 0, BLACK, BACKGROUND, (char*)"          ", 2);
		//LCD_Draw_String(0, 0, BLACK, BACKGROUND, buf, 2);
}




/* Trace functions ----------------------------------------------------------*/

void SWO_Send_String(char string[])
{
	unsigned char i = 0;

	while (string[i] != 0)
	{
		ITM_SendChar(string[i]);
		i++;		
	}
}

void SWO_Send_Array(s16 string[], u16 length)
{
	u16 i = 0;
	char buf1[11];	

	for (i = 0; i < length; i++)
	{	
		
		itoa(string[i], buf1);
	 	SWO_Send_String(buf1);
		SWO_Send_String((char*)" ");		
	}			
}

void Send_Log()
{
	if (ADC_state == FINISHED && sendDataFlag == SET)
	{
// 		switch(data_to_be_sent)
// 		{
// 			case DATA_A: SWO_Send_Array(Buff.current_a,		ADCBUFFERSIZE); break;
// 			case DATA_B: SWO_Send_Array(Buff.current_b,		ADCBUFFERSIZE); break;
// 			case DATA_C: SWO_Send_Array(Buff.current_c,		ADCBUFFERSIZE); break;
// 			case 3:		 SWO_Send_Array(Buff.voltage_alpha, ADCBUFFERSIZE); break;
// 			case 4:		 SWO_Send_Array(Buff.voltage_beta,	ADCBUFFERSIZE); break;
// 		}
// 		send_data_flag = CLEAR;		
	}	
}




/* ADC functions ------------------------------------------------------------*/

void ADC_Calibration()
{
	ADC_Start_Triple_Conversion();
	ADC_Start_Triple_Conversion();

	LL_ADC_INJ_SetOffset(ADC1, LL_ADC_INJ_RANK_1, (adc_inj_a));
	LL_ADC_INJ_SetOffset(ADC2, LL_ADC_INJ_RANK_1, (adc_inj_b));
	LL_ADC_INJ_SetOffset(ADC3, LL_ADC_INJ_RANK_1, (adc_inj_c));

}

void ADC_Sensing()
 {	 
	ADC_Start_Triple_Conversion();		

	I_abc_meas.a = -(f32)adc_inj_a * 0.00517f;  //HW_CURRENT_SENS_SCALE * 1.3;
	I_abc_meas.b = -(f32)adc_inj_b * 0.00517f;  //HW_CURRENT_SENS_SCALE * 1.3;
	I_abc_meas.c = -(f32)adc_inj_c * 0.00517f;  //HW_CURRENT_SENS_SCALE * 1.3;	 

	if (I_abc_meas.a >= MOTOR_CURRENT_MAX || I_abc_meas.b >= MOTOR_CURRENT_MAX || I_abc_meas.c >= MOTOR_CURRENT_MAX)
	{
		motorControlState = STOP;
	}
 }
  



/* Interrupts ---------------------------------------------------------------*/
#pragma GCC optimize ("-O0")


/*------ PWM Reload ISR --------------------*/
void TIM1_UP_TIM10_IRQHandler()		// Main PWM ISR
{
	static volatile u8 cycle		= 0;
	static volatile u8 pwm_counts	= 0;	

	if (cycle == 2)	
	{
		cycle = 0;	
		PWM_Update();		
	}	
	else
	{
		pwm_counts++;
	}
	

	if (pwm_counts == PWM_RELOAD_K && appState == ON)					// Exec time ~ 5 us
	{
		Dbg_Tim_Start();

		ADC_Sensing();
		
		LL_EXTI_GenerateSWI_0_31(LL_EXTI_LINE_0);			// Call Main motor routine (SVM, PID, etc)
		
		pwm_counts = 0;
	}	

	cycle++;
	LL_TIM_ClearFlag_UPDATE(TIM1);
}

/*------ Current and Position Control ISR --*/
void EXTI0_IRQHandler()
{	
	arm_clarke_f32(I_abc_meas.a, I_abc_meas.b, &I_alphabeta_meas.alpha, &I_alphabeta_meas.beta);
	
	if (motorControlState == RUN_ESTIM)
	{
		motor_pos.cos = motor_pos_est.cos;
		motor_pos.sin = motor_pos_est.sin;
	}
	else
	{
		motor_pos.cos = motor_pos_gen.cos;
		motor_pos.sin = motor_pos_gen.sin;
	}

	arm_park_f32(I_alphabeta_meas.alpha, I_alphabeta_meas.beta, &I_DQ.d, &I_DQ.q, 
				 motor_pos.sin, motor_pos.cos);

	Motor_Control_fcn[motorControlState]();

	motorPower = (I_alphabeta_meas.alpha * U_alphabeta.alpha + I_alphabeta_meas.beta * U_alphabeta.beta) * 3.0/2.0;

	Data_Log();
	
	Dbg_Tim_Stop_ns();
	
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

/*------ Speed Control ISR -----------------*/
void TIM6_DAC_IRQHandler()
{
	if		(motorControlState == START_RAMP_OL)
	{
		Speed_Ramp(&motor_speed_ramp, motor_speed_ramp_end, motor_speed_accel_ol);
	}
	else if (motorControlState == RUN_ESTIM)
	{
		Speed_Ramp(&motor_speed_ramp, motor_speed_ref, motor_speed_accel_ol);

		I_DQ_desired.q = -PID_Controller(motor_speed_ramp, motor_speed_estim, &SpeedPIDParams);		
		
	}
	//Data_Log();
	LL_TIM_ClearFlag_UPDATE(TIM6);
}


void ADC_IRQHandler()
{
	LL_ADC_ClearFlag_JEOS(ADC1);

	
}

void test__()
{
	
	//volatile f32 ans = __VCVT(-16000);


	//HAL_FLASHEx_Erase();	
	//char ans[11];
	//char ans2[11];
	//volatile f32  input = 1.0786f;
	//
	//Dbg_Tim_Start();
	//ftoa_n(input, ans, 5);		// 13us
	//Dbg_Tim_Stop_ns();

	//volatile u32 nh =10;

	//Dbg_Tim_Start();
	//ftoa(input, ans2, 5);		// 59us
	//Dbg_Tim_Stop_ns();

	//s16 data1  = -1234;	//  2
	//u32 data3 = 0x121;
	//u32 data4 = 0x13213;


	//u32* data_ptr = (u32*)&data1;

	//volatile u32 data2 = *data_ptr & 0xffff;

}




/******************************************************************************
* MAIN
******************************************************************************/

int main(void)
{	
	System_Init();

	//test__();

	Buttons_Init();	

	Speed_Control_Timer_ISR_Init();
	
	ADC_Triple_Init();	
	ADC_Triple_Enable();
	ADC_Calibration();
	
	Start_Menu();
	
	LCD_Init();
	LCD_Draw_Interface();	
	
	EXTI_Motor_Control_Init();
	PWM_timer_init();		
	
	


			while (1)
			{
								
				APP_Control();				
				
				

				/*----------------GLOBAl TIMERS----------------*/

				LCD_Refresh();		// Refresh rate 500ms				
				//Send_Log();
				Data_Log_tmr();
			  				
			}
}