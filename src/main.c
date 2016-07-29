#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_adc.h"

//Float Method
#define KD 5    //Derivative Gain
#define KI 0.00085   //Integral Gain
#define KP 0.4   //Proportional Gain
#define ALPHA 0.4
#define OC_TRIGGER 600

#define CURRENT 0
#define DELAY_1 1

void my_delay(int wait)
{
	for(int i=wait;i!=0;i--)
	{
		__asm__("nop");
	}
}

void TM_TIMER_Init(void)
{
	/*
		timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
		PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
		TIM_Period = timer_tick_frequency / PWM_frequency - 1

		In our case, for 50Khz PWM_frequency, set Period to
		TIM_Period = 72000000 / 50000 - 1 = 1439
	*/

		TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	    /**************************
	     *       INIT TIM1        *
	     **************************/
		TIM_BaseStruct.TIM_Prescaler = 0;
	    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	    TIM_BaseStruct.TIM_Period = 1439;
	    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	    TIM_BaseStruct.TIM_RepetitionCounter = 0;

	    /* Select APB2 as clock for TIM1 */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	    /* Init TIM1 with BaseStruct, then enable TIM1 and output */
	    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
	    TIM_Cmd(TIM1, ENABLE);
	    TIM_CtrlPWMOutputs(TIM1, ENABLE);

	    /**************************
	     *       INIT TIM2        *
	     **************************/
		TIM_BaseStruct.TIM_Prescaler = 0;
	    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	    TIM_BaseStruct.TIM_Period = 0xFFFF;
	    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	    TIM_BaseStruct.TIM_RepetitionCounter = 0;

	    /* Select APB2 as clock for TIM1 */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	    /* Init TIM1 with BaseStruct, then enable TIM1 and output */
	    TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
	    TIM_Cmd(TIM2, ENABLE);
}

void TM_PWM_Init(void)
{
	/*
		pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
		where DutyCycle is in percent, between 0 and 100%
	*/
		TIM_OCInitTypeDef TIM_OCStruct;

		TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

		TIM_OCStruct.TIM_Pulse = 0;
		TIM_OC1Init(TIM1, &TIM_OCStruct);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
}

void TM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitTypeDef GPIO_ADCInitStructure;
    GPIO_InitTypeDef GPIO_LEDInitStructure;

    /**************************
     *    INIT PWM ON PA8     *
     **************************/
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;

    /* Clock for GPIOA for PWM */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /*	Init GPIO to AF_6 for TIM1 */
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);

    /**************************
     *     INIT ADC1 ON PC1	  *
     *     INIT ADC2 ON PC4   *
     **************************/
    GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4 ;

	/* GPIOC Periph clock enable for ADC*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE );

    /* Init GPIO for ADC */
    GPIO_Init( GPIOC, &GPIO_ADCInitStructure );

    /**************************
     *     INIT DAC1 ON PA4   *
     **************************/
    GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_4 ;

    /* Init GPIO for ADC */
    GPIO_Init( GPIOA, &GPIO_ADCInitStructure );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    /**************************
     *   INIT LED ON  PE10    *
     **************************/
    GPIO_LEDInitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_LEDInitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_LEDInitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_LEDInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_LEDInitStructure.GPIO_Pin = GPIO_Pin_13;

    /* Enable the GPIO_LED Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

    /*	Configure GPIO to output for LED */
    GPIO_Init(GPIOE, &GPIO_LEDInitStructure);

}

void ADC_Activate()
{
	ADC_InitTypeDef       ADC1_InitStructure;
    ADC_CommonInitTypeDef ADC1_CommonInitStructure;
    ADC_InitTypeDef       ADC2_InitStructure;
    ADC_CommonInitTypeDef ADC2_CommonInitStructure;

    /* Configure & Enable the ADC 1 & 2 clock */
    RCC_ADCCLKConfig( RCC_ADC12PLLCLK_Div2 );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_ADC12, ENABLE );

    /**************************
     *       INIT ADC1        *
     **************************/
    ADC_VoltageRegulatorCmd( ADC1, ENABLE );

    /* delay for 10us for ADC to stabilize */
    my_delay(1000);

    ADC_SelectCalibrationMode( ADC1, ADC_CalibrationMode_Single );
    ADC_StartCalibration( ADC1 );

    while ( ADC_GetCalibrationStatus( ADC1 ) != RESET )
    {
    	__asm__("nop");
    };

    ADC1_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC1_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
    ADC1_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC1_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
    ADC1_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit( ADC1, &ADC1_CommonInitStructure );

    ADC1_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    ADC1_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC1_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC1_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC1_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
    ADC1_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
    ADC1_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init( ADC1, &ADC1_InitStructure );

    /* ADC1 regular channel7 configuration */
    ADC_RegularChannelConfig( ADC1, ADC_Channel_7, 1, ADC_SampleTime_19Cycles5 );

    /* Enable ADC1 */
    ADC_Cmd( ADC1, ENABLE );

    /* wait for ADRDY */
    while( !ADC_GetFlagStatus( ADC1, ADC_FLAG_RDY ) )
    {
    	__asm__("nop");
    }

    /* Start ADC1 Software Conversion */
    ADC_StartConversion( ADC1 );

    /**************************
     *       INIT ADC2        *
     **************************/
    ADC_VoltageRegulatorCmd( ADC2, ENABLE );

    /* delay for 10us for ADC to stabilize */
    my_delay(1000);

    ADC_SelectCalibrationMode( ADC2, ADC_CalibrationMode_Single );
    ADC_StartCalibration( ADC2 );

    while ( ADC_GetCalibrationStatus( ADC2 ) != RESET )
    {
    	__asm__("nop");
    };

    ADC2_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC2_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
    ADC2_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC2_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
    ADC2_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit( ADC2, &ADC2_CommonInitStructure );

    ADC2_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    ADC2_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC2_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC2_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC2_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC2_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
    ADC2_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
    ADC2_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init( ADC2, &ADC2_InitStructure );

    /* ADC1 regular channel7 configuration */
    ADC_RegularChannelConfig( ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5 );

    /* Enable ADC1 */
    ADC_Cmd( ADC2, ENABLE );

    /* wait for ADRDY */
    while( !ADC_GetFlagStatus( ADC2, ADC_FLAG_RDY ) )
    {
    	__asm__("nop");
    }

    /* Start ADC1 Software Conversion */
    ADC_StartConversion( ADC2 );
}


void DACtivate()
{
	DAC_InitTypeDef DAC_InitStruct;

	/* DAC1 channel1 Configuration */
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);

	/* Enable DAC1 Channel1 */
	DAC_Cmd(DAC_Channel_1, ENABLE);
}
// ----- main() ---------------------------------------------------------------
int main()
{
    trace_puts("START!");

	__IO uint16_t  ADC1ConvertedValue = 0;
	int error_signal;
	int ADC2CurrentSense;
//	int clock_count;

	float yd[2] = {0,0};
	float yi[2] = {0,0};
	float yp[2] = {0,0};
	float x_in[2] = {0,0};
	float filter_input;
	float filter_output;

	/* Initialize system */
	SystemInit();

	/* Init leds */
	TM_GPIO_Init();

	/* Init timer */
	TM_TIMER_Init();

	/* Init PWM */
	TM_PWM_Init();

	/* Activate the ADC */
	ADC_Activate();

	/* Activate the DAC */
	DACtivate();

	while (1)
	{
		/* For Latency testing */
		TIM_SetCounter(TIM2,0);

//	    /* Test EOC flag */
	    while ( ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) == RESET );

//	    /* Get ADC1 converted data */
	    ADC1ConvertedValue = ADC_GetConversionValue( ADC1 );

	    filter_input = (float)ADC1ConvertedValue;
	    filter_input = 3.0 * filter_input/0x3FF;
	    filter_input = (2.5 - filter_input) * 1439;

	    x_in[CURRENT] = filter_input;

	    //FILTER (FLOATS)
	    yp[CURRENT] = x_in[CURRENT]*KP;
	    yi[CURRENT] = x_in[DELAY_1]*KI+yi[DELAY_1];
	    yd[CURRENT] = ALPHA * yd[DELAY_1]+KD*(x_in[CURRENT]-x_in[DELAY_1]);

//	    //OUTPUT
	    filter_output=yp[CURRENT]+yi[CURRENT]+yd[CURRENT];
//
//	    //DELAYS
	    yd[DELAY_1] = yd[CURRENT];
	    yi[DELAY_1] = yi[CURRENT];
	    x_in[DELAY_1] = x_in[CURRENT];

	    error_signal = (int)filter_output;


//		OC Protection
	    ADC2CurrentSense = ADC_GetConversionValue( ADC2 );

	    if(ADC2CurrentSense<OC_TRIGGER)
	    {
	    	// Turn off LED
	    	GPIOE->BRR = GPIO_Pin_13;
	    }
	    else
	    {
	    	// Turn on LED
	    	GPIOE->BSRR = GPIO_Pin_13;
	    	error_signal = 1;
	    }

//		Set PWM Duty Cycle
	    TIM1->CCR1 = error_signal;


	    /* For Latency testing */
//	    clock_count = TIM_GetCounter(TIM2);
//	    trace_printf("%d\n",clock_count);


	}
}
