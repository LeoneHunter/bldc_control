#include "pmsm_sensorless_cntrl.h"

/******************************************************************************
* Global functions
******************************************************************************/

void PMSM_SMObserverBemf(PMSM_SMObserverBemf_typedef* smoStruct, 
						 MC_2PhSyst* I_meas, 
						 MC_2PhSyst* U_meas,
						 f32		 OmegaEl)
{
	f32	omegaElTs;

	omegaElTs  = OmegaEl  * smoStruct->Ts;	

	smoStruct->sgnI.alpha  = ((smoStruct->I_est.alpha - I_meas->alpha >= 0) ? 1.0 : -1.0);
    smoStruct->sgnI.beta   = ((smoStruct->I_est.beta  - I_meas->beta  >= 0) ? 1.0 : -1.0);

	smoStruct->mZTs.alpha  = smoStruct->sgnI.alpha * smoStruct->k1Ts;
	smoStruct->mZTs.beta   = smoStruct->sgnI.beta  * smoStruct->k1Ts;

	// Iest(n)             = Iest(n-1) + Iest(n-1)*a11Ts + a12Ts*Bemf(n-1) - a12Ts*Uprev(n-1) + k1Ts*sgn( Iest(n-1) - I_meas(n-1) )

	smoStruct->I_est.alpha  = smoStruct->I_est.alpha + smoStruct->I_est.alpha * smoStruct->a11Ts  + smoStruct->a12Ts 
							* smoStruct->bemf.alpha  - smoStruct->a12Ts * U_meas->alpha + smoStruct->mZTs.alpha;

	smoStruct->I_est.beta   = smoStruct->I_est.beta  + smoStruct->I_est.beta  * smoStruct->a11Ts  + smoStruct->a12Ts 
							* smoStruct->bemf.beta   - smoStruct->a12Ts * U_meas->beta  + smoStruct->mZTs.beta;
	 
	smoStruct->bemf.alpha   = (smoStruct->bemf.alpha - omegaElTs * smoStruct->bemf.beta ) + (smoStruct->g1k1Ts 
							* smoStruct->sgnI.alpha - smoStruct->g2k1Ts * smoStruct->sgnI.beta);

	smoStruct->bemf.beta    = (smoStruct->bemf.beta  + omegaElTs * smoStruct->bemf.alpha) + (smoStruct->g2k1Ts 
							* smoStruct->sgnI.alpha + smoStruct->g1k1Ts * smoStruct->sgnI.beta);
}