#pragma once

#include "main.h"

#define RPMTORADS	(6.28318530/60)		


/******************************************************************************
* Constants
******************************************************************************/

/**                POWER STAGE AND MOTOR CONFIGURATION                      **/

#define MOTOR_SPEED_MIN_RPM				100								// [RPM]	Min spd. Abs min 22

#define PWM_FREQUENCY					128								// [KHz]	PWM carrier frequency kHz			!!!  2KHz MIN  !!!
#define PWM_RELOAD_K					4
#define PWM_PERIOD						(1.0/(((f32)PWM_FREQUENCY)*1000.0))

#define PWM_DEADTIME					0								// Deadtime in clocks

#define PWM_TIMER_FREQ_ARR				(42000/PWM_FREQUENCY)


/**                 3ph AC / BLDC High Voltage board parameters constants   **/

#define HW_APP_VOLT_MAX_V			   12.00							// [V]                  
#define HW_APP_CURR_MAX_A			   10.00							// [A]

#define HW_ADC_VOLTAGE					3.24							// [V]		ADC Reference voltage
#define HW_OPAMP_GAIN					5.10							//			Operational Amplifier Gain
#define HW_SHUNT_RES					0.03							// [Om]		Shunt resistance

#define HW_CURRENT_SENS_SCALE			(((HW_ADC_VOLTAGE/4095.0)/HW_OPAMP_GAIN)/HW_SHUNT_RES)

#define HW_DC_BUS_VOLTAGE			   12.0								// [V]		DC bus voltage

#define SVM_MODULATION_MI_HALF		   (0.866025f)



/**                    MOTOR PARAMETERS                                     **/

#define MOTOR_APP_SPEED_MAX_RPM     30000.0								// Application Speed range Max          
                                                     
#define MOTOR_MAX_SPEED_RPM         30000.0								// maximal motor desired speed [rpm]    
#define MOTOR_MAX_SPEED_EL			(MOTOR_MAX_SPEED_RPM*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS)
                                                        
#define MOTOR_NUMBER_OF_POLE_PAIRS      7.0								// Number of motor pole pairs           
#define MOTOR_BEMF_CONST_mV_RPM      1125.0								// Motor BEMF constant voltage/mech speed [mV/rpm] 
#define MOTOR_CURRENT_MAX              10.0								// Maximal Current amplitude limit                                                           

#define MOTOR_CURRENT_MIN             (-MOTOR_CURRENT_MAX)

#define MOTOR_RS						0.116000f						// [Om] Phase resistance
#define MOTOR_LS						0.000008f						// [H]  Phase inductance
#define MOTOR_BEMF_SPEED_CONST          (MOTOR_BEMF_CONST_mV_RPM/HW_APP_VOLT_MAX_V/1000.0*MOTOR_APP_SPEED_MAX_RPM)




/**                PID Controller parameters                                  **/

/* i_SQ (D stator current component) controller parameters */

#define MOTOR_PID_I_D_PROP			    0.005				// 0.1             
#define MOTOR_PID_I_D_INTEG				0.002				// 0.05
#define MOTOR_PID_I_D_DIFF				0.0          

#define MOTOR_PID_I_D_POS_LIMIT        (1.0)       
#define MOTOR_PID_I_D_NEG_LIMIT       (-1.0)          

/* i_SQ (D stator current component) controller parameters */

#define MOTOR_PID_I_Q_PROP			    0.005             
#define MOTOR_PID_I_Q_INTEG				0.002            
#define MOTOR_PID_I_Q_DIFF				0.0           

#define MOTOR_PID_I_Q_POS_LIMIT        (1.0)         
#define MOTOR_PID_I_Q_NEG_LIMIT       (-1.0)          

/* SPEED controller parameters */

#define MOTOR_SPEED_SAMPL_PER           0.001           

#define MOTOR_PID_SPEED_PROP			0.001            
#define MOTOR_PID_SPEED_INTEG			0.0000005        
#define MOTOR_PID_SPEED_DIFF			0.001			// 0.4  

#define MOTOR_PID_SPEED_POS_LIMIT       (1.0)           
#define MOTOR_PID_SPEED_NEG_LIMIT      (-1.0)          
	

/* Decoupling parameters */

#define MOTOR_DECOUP_A12			   -MOTOR_LS
#define MOTOR_DECOUP_A21				MOTOR_LS


/* Speed Ramp parameters */

#define MOTOR_ACCEL_SPEED_OPEN_L_RPM_SEC	(800.0)    /* acceleration at starting open loop [rpm/s] */
#define MOTOR_ACCEL_SPEED_CLOSE_L_RPM_SEC   (500.0)   /* acceleration at close loop [rpm/s]   */

#define MOTOR_ACCEL_SPEED_OPEN_L_EL		(MOTOR_ACCEL_SPEED_OPEN_L_RPM_SEC *MOTOR_SPEED_SAMPL_PER)
#define MOTOR_ACCEL_SPEED_CLOSE_L_EL	(MOTOR_ACCEL_SPEED_CLOSE_L_RPM_SEC*MOTOR_SPEED_SAMPL_PER)



/**                           OBSERVER PARAMETERS							**/

#define MOTOR_SMO_TS					(PWM_PERIOD*PWM_RELOAD_K)
#define MOTOR_SMO_A11TS				   -(MOTOR_RS*MOTOR_SMO_TS/MOTOR_LS)           /* a11*Ts = -(Rs/Ls*Ts) SMO model coeficient */
#define MOTOR_SMO_A12TS				   -(MOTOR_SMO_TS/MOTOR_LS)            /* b11*Ts = 1/Ls SMO model coeficient */

#define MOTOR_SMO_K1				-4500.0
#define MOTOR_SMO_G1				   -0.5
#define MOTOR_SMO_G2				    0.0

#define MOTOR_EST_G1_K1_TS_MULTIPLE     (MOTOR_SMO_G1*MOTOR_SMO_K1*MOTOR_SMO_TS)           /* gain g1 multiplicant coef g1*k1*Ts/amplBEMF */
#define MOTOR_EST_G2_K1_TS_MULTIPLE     0.0												/* gain g2 multiplicant coef g1*k1*Ts/amplBEMF */


/* Adaptive speed scheme parameters */

#define MOTOR_AD_SCH_INTEG_GAIN         0.8				/* speed adaptive schematic gain [frac16] */




/**                           REFERENCE VALUES							**/


#define MOTOR_CURRENT_STARTING          4.0             /* Starting (open loop) Current [A ] */
#define MOTOR_CURRENT_RUNING0           1.0             /* Running MODE begin Current [A ]            */

#define MOTOR_STABILISATION_PERIOD_S    0.25            /* alignment stabilisation time [s] */

#define MOTOR_ALIGNMENT_DURATION		200             /* alignment stabilisation time [s] */

#define MOTOR_TRESHOLD_SPEED_SMO_ON_EL		(500*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS)         // SMO begins ti estimate angle

#define MOTOR_TRESHOLD_SPEED_SPEST_ON_EL	(2000*RPMTORADS*MOTOR_NUMBER_OF_POLE_PAIRS)         // SMO feedback enable speed
