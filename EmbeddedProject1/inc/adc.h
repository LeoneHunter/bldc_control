#pragma once

#include "main.h"

#define ADCBUFFERSIZE	400
//#define SCOPE_MODE

s16 adc_buffer_a[ADCBUFFERSIZE] = {0};
s16 adc_buffer_b[ADCBUFFERSIZE] = {0};
s16 adc_buffer_c[ADCBUFFERSIZE] = {0};

s16 adc_inj_a = 0;
s16 adc_inj_b = 0;
s16 adc_inj_c = 0;


void ADC_Single_Init(void)
{
	/* GPIO and CLOCKs Init -----------------------------------------------------------------*/

	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**ADC1 GPIO Configuration
	PA4   ------> ADC1_IN4
	PA5   ------> ADC1_IN5
	PA3   ------> ADC1_IN3
	*/
	GPIO_InitStruct.Pin		= LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode	= LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull	= LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* DMA ----------------------------------------------------------------------------------*/
	/* ADC1 DMA Init */	

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	//NVIC_SetPriority(DMA2_Stream0_IRQn, 3);
	//NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	///////////////////////////////////////////////

	
	
#ifdef SCOPE_MODE
	LL_DMA_InitTypeDef DMA_ADC_Init = {0};

	DMA_ADC_Init.Channel				= LL_DMA_CHANNEL_0;
	DMA_ADC_Init.Direction				= LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_ADC_Init.FIFOMode				= LL_DMA_FIFOMODE_DISABLE;
	DMA_ADC_Init.MemoryOrM2MDstAddress	= (u32)adc_buffer_a;
	DMA_ADC_Init.MemoryOrM2MDstDataSize	= LL_DMA_MDATAALIGN_HALFWORD;
	DMA_ADC_Init.MemoryOrM2MDstIncMode	= LL_DMA_MEMORY_INCREMENT;
	DMA_ADC_Init.Mode					= LL_DMA_MODE_NORMAL;
	DMA_ADC_Init.NbData					= ADCBUFFERSIZE;
	DMA_ADC_Init.PeriphBurst			= LL_DMA_PBURST_SINGLE;
	DMA_ADC_Init.PeriphOrM2MSrcAddress	= (u32)&ADC1->DR;
	DMA_ADC_Init.PeriphOrM2MSrcDataSize	= LL_DMA_PDATAALIGN_HALFWORD;
	DMA_ADC_Init.PeriphOrM2MSrcIncMode	= LL_DMA_PERIPH_NOINCREMENT;
	DMA_ADC_Init.Priority				= LL_DMA_PRIORITY_MEDIUM;

	LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMA_ADC_Init);

	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
#endif



	/* Common config --------------------------------------------------------------------*/
	
	ADC_InitStruct.Resolution			= LL_ADC_RESOLUTION_10B;				// 10bit resolution 0 - 1024
	ADC_InitStruct.DataAlignment		= LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.SequencersScanMode	= LL_ADC_SEQ_SCAN_DISABLE;				// Scan of more than one channel
	LL_ADC_Init(ADC1, &ADC_InitStruct);
		
#ifndef SCOPE_MODE
	ADC_REG_InitStruct.TriggerSource	= LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength	= LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;	// Number of channels to be converted
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;		// Conversion interrupted every 1 rank
	ADC_REG_InitStruct.ContinuousMode	= LL_ADC_REG_CONV_SINGLE;				// Continuous convertion mode
	ADC_REG_InitStruct.DMATransfer		= LL_ADC_REG_DMA_TRANSFER_NONE;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
#else
	ADC_REG_InitStruct.TriggerSource	= LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength	= LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;	// Number of channels to be converted
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;		// Conversion interrupted every 1 rank
	ADC_REG_InitStruct.ContinuousMode	= LL_ADC_REG_CONV_CONTINUOUS;			// Continuous convertion mode
	ADC_REG_InitStruct.DMATransfer		= LL_ADC_REG_DMA_TRANSFER_LIMITED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
#endif

	LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
	//LL_ADC_EnableIT_EOCS(ADC1);												// Interrupt eoc

	ADC_CommonInitStruct.CommonClock	= LL_ADC_CLOCK_SYNC_PCLK_DIV2;
	ADC_CommonInitStruct.Multimode		= LL_ADC_MULTI_INDEPENDENT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);	// Scanning sequence setting
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_3);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_3CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_3CYCLES);

	LL_ADC_Enable(ADC1);
}

u16 ADC_Start_Single_REG_Conversion(void)
{

	LL_ADC_REG_StartConversionSWStart(ADC1);
	while (!LL_ADC_IsActiveFlag_EOCS(ADC1))
	{
	}
	 LL_ADC_ClearFlag_EOCS(ADC1);

	return	LL_ADC_REG_ReadConversionData12(ADC1);
}



void ADC_Triple_Init(void)
{
	/* GPIO and CLOCKs Init -----------------------------------------------------------------*/

	LL_ADC_InitTypeDef			ADC_InitStruct			= { 0 };					// INIT Structs
	LL_ADC_INJ_InitTypeDef		ADC_INJ_InitStruct		= { 0 };
	LL_ADC_CommonInitTypeDef	ADC_CommonInitStruct	= { 0 };
	LL_GPIO_InitTypeDef			GPIO_InitStruct			= { 0 };

		
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);								// Clock
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);								// Clock
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);								// Clock

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**ADC1 GPIO Configuration
	PA4   ------> ADC1_IN4
	PA2   ------> ADC1_IN2
	PA0   ------> ADC1_IN0
	*/
	GPIO_InitStruct.Pin		= LL_GPIO_PIN_4|LL_GPIO_PIN_2|LL_GPIO_PIN_0;			// GPIO
	GPIO_InitStruct.Mode	= LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull	= LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		
	
	/* Main ADC config --------------------------------------------------------------------*/
	
	ADC_InitStruct.Resolution			= LL_ADC_RESOLUTION_12B;				
	ADC_InitStruct.DataAlignment		= LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.SequencersScanMode	= LL_ADC_SEQ_SCAN_ENABLE;				
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	LL_ADC_Init(ADC2, &ADC_InitStruct);
	LL_ADC_Init(ADC3, &ADC_InitStruct);
		


	/* Master ---------------------------------------------------------------*/

	ADC_INJ_InitStruct.TriggerSource	= LL_ADC_INJ_TRIG_SOFTWARE;
	ADC_INJ_InitStruct.SequencerLength	= LL_ADC_INJ_SEQ_SCAN_DISABLE;			
	ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;		
	ADC_INJ_InitStruct.TrigAuto			= LL_ADC_INJ_TRIG_INDEPENDENT;	
	LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);

	/* Slaves ---------------------------------------------------------------*/
		
	ADC_INJ_InitStruct.SequencerLength	= LL_ADC_INJ_SEQ_SCAN_DISABLE;			
	ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;		
	ADC_INJ_InitStruct.TrigAuto			= LL_ADC_INJ_TRIG_INDEPENDENT;	
	LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
	LL_ADC_INJ_Init(ADC3, &ADC_INJ_InitStruct);
	   	  

	ADC_CommonInitStruct.CommonClock	= LL_ADC_CLOCK_SYNC_PCLK_DIV4;
	ADC_CommonInitStruct.Multimode		= LL_ADC_MULTI_TRIPLE_INJ_SIMULT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC2), &ADC_CommonInitStruct);
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC3), &ADC_CommonInitStruct);


	LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_4);	
	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_2);
	LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_0);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_15CYCLES);

	LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
	
	/*LL_ADC_EnableIT_JEOS(ADC1);
	NVIC_SetPriority(ADC_IRQn, 4);
	NVIC_EnableIRQ(ADC_IRQn);*/	
		
}

void ADC_Triple_Enable()
{
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
	LL_ADC_Enable(ADC3);
}

void ADC_Triple_Disable()
{
	LL_ADC_Disable(ADC1);
	LL_ADC_Disable(ADC2);
	LL_ADC_Disable(ADC3);
}


void ADC_Start_Triple_Conversion(void)
{
	LL_ADC_INJ_StartConversionSWStart(ADC1);
	while (!LL_ADC_IsActiveFlag_JEOS(ADC1))
	{
	}
	LL_ADC_ClearFlag_JEOS(ADC1);

	adc_inj_a = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	adc_inj_b = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	adc_inj_c = LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_1);
}



