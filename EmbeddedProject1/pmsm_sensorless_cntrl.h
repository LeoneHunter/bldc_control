#pragma once

/******************************************************************************
* Includes
******************************************************************************/

#include "main.h"


/******************************************************************************
* Types
******************************************************************************/

typedef struct
{
    f32 sin;
    f32 cos;
} MC_Angle;

typedef struct
{
    f32 alpha;
    f32 beta;
} MC_2PhSyst;

typedef struct
{
    f32 a;
    f32 b;
    f32 c;
} MC_3PhSyst;

typedef struct
{
    f32 d;
    f32 q;
} MC_DQSyst;


typedef struct
{
    MC_2PhSyst bemf;				 /* estimated bemf vector */
    MC_2PhSyst I_est;				 /* estimated current vector */   
    MC_2PhSyst sgnI;				 /* sgnI = sgn (current estim. - current meas.) */
    MC_2PhSyst mZTs;				 /* -z = -k1 sgn (curr - pCurrMeas)*/
    f32 a11Ts;                       /* coef a11*Ts = -(R/L*Ts) */
    f32 a12Ts;                       /* coef a12*Ts = -b1*Ts */
    f32 Ts;							 /* Ts*/
    f32 k1Ts;						 /* coef k1*Ts */
    f32 signGain;                    /* coef extended sgn signGain*2^signGainSc slope */
    f32 g1k1Ts;                      /* coef g1*k1*Ts */
    f32 g2k1Ts;                      /* coef g2*k1*Ts */
} PMSM_SMObserverBemf_typedef;			/* SMO Bemf observer with speed component universal version with coeficients scaling structure */

typedef struct
{
    f32 integPartK_1;                /* integral part */
    f32 integGain;                   /* integral gain */
} PMSM_SpeedAdaptScheme_params_typedef;   /* adaptive scheme gain structure */

typedef struct
{
    f32 DECOa12;                     /* coef Decoupling a21 = -Lq Scaled = -Lq*Su/Si/SomegaEl */
    f32 DECOa21;                     /* coef Ld Scaled = = Ld*Su/Si/SomegaEl */

} PMSM_Decoupl_typedef;					/* currents decoupling structure */



/******************************************************************************
* Global functions
******************************************************************************/

void		PMSM_SMObserverBemf(PMSM_SMObserverBemf_typedef*, 
								MC_2PhSyst*,	 
								MC_2PhSyst*,
								f32);



/******************************************************************************
* Inline functions
******************************************************************************/

inline  f32 PMSM_AdaptSch(MC_2PhSyst* mzts, 
						 MC_2PhSyst* bemf, 
						 PMSM_SpeedAdaptScheme_params_typedef* adaptStruct,
						 f32 amplbemf)
{
	adaptStruct->integPartK_1 = (((mzts->alpha * bemf->beta - mzts->beta * bemf->alpha) 
					 / amplbemf) * adaptStruct->integGain) + adaptStruct->integPartK_1;

	return adaptStruct->integPartK_1;
}


inline void PMSM_DQDecoupl(MC_DQSyst* Udecoup, 
						   MC_DQSyst* Idecoup, 
						   PMSM_Decoupl_typedef* pDecoup,
						   f32 omegaEl)
{
	Udecoup->d				=  omegaEl  * pDecoup->DECOa12 * Idecoup->q;
	Udecoup->q				=  omegaEl  * pDecoup->DECOa21 * Idecoup->d;
}