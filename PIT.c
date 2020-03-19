/*
 * PIT.c
 	This is the API for manage of the PIT: Periodic Interrupt Timer for Kinetis K64.
 	It contains all the implementations of configuration functions and runtime functions.
 *  Created on: Feb 22, 2020
 *      Author: Sofía Salazar, Omar Soto
 */
#include "PIT.h"
#include "bits.h"
#include "stdint.h"

#define CLEAR_FLAG (0U)

static pit_interrupt_flags_t g_pit_inter_status = {0};
static void (*sequence_handler[4])(void);

/********************************************************************************************/
/*!
 	 \brief	 Handler for int of PIT_0 should be the change of colors
 	 \param[in]  void.
 	 \return void
 */
void PIT_callback_init(PIT_timer_t pit_timer,void(*handler)(void))
{
	sequence_handler[pit_timer]=handler;
}


/*IRQ handlers from hardware vectors to control what happens*/
void PIT0_IRQHandler()
{
	PIT->CHANNEL[PIT_0].TFLG |= PIT_TFLG_TIF_MASK;
	/** Read control register for clear PIT flag, this is silicon bug */
	(void)PIT->CHANNEL[PIT_0].TCTRL;
	/*Change the programmer flag to be used on other modules*/
	g_pit_inter_status.PIT_0_flag_status = TRUE;
	/*Make the callback once the interrupt vector was reached*/
	if(sequence_handler[PIT_0])
	{
		sequence_handler[PIT_0]();
	}
}
void PIT1_IRQHandler()
{
	PIT->CHANNEL[PIT_1].TFLG |= PIT_TFLG_TIF_MASK;
	/** Read control register for clear PIT flag, this is silicon bug */
	(void)PIT->CHANNEL[PIT_1].TCTRL;
	/*Change the programmer flag to be used on other modules*/
	g_pit_inter_status.PIT_1_flag_status = TRUE;
	/*Make the callback once the interrupt vector was reached*/
	if(sequence_handler[PIT_1])
	{
		sequence_handler[PIT_1]();
	}


}
void PIT2_IRQHandler()
{
	PIT->CHANNEL[PIT_2].TFLG |= PIT_TFLG_TIF_MASK;
	/** Read control register for clear PIT flag, this is silicon bug */
	(void)PIT->CHANNEL[PIT_2].TCTRL;
	/*Change the programmer flag to be used on other modules*/
	g_pit_inter_status.PIT_2_flag_status = TRUE;
	/*Make the callback once the interrupt vector was reached*/
	if(sequence_handler[PIT_2])
	{
		sequence_handler[PIT_2]();
	}


}

void PIT3_IRQHandler()
{
	PIT->CHANNEL[PIT_3].TFLG |= PIT_TFLG_TIF_MASK;
	/** Read control register for clear PIT flag, this is silicon bug */
	(void)PIT->CHANNEL[PIT_3].TCTRL;
	/*Change the programmer flag to be used on other modules*/
	g_pit_inter_status.PIT_3_flag_status = TRUE;
	/*Make the callback once the interrupt vector was reached*/
	if(sequence_handler[PIT_3])
	{
		sequence_handler[PIT_3]();
	}


}

/********************************************************************************************/
/*!
 	 \brief	 This function loads the value from the timer considering the frequence of
 	 		PIT to 1/2 of quarz
 	 \param[in]  void.
 	 \return void
 */
void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay)
{

	/*Init a Load value for timer counter*/
	uint32_t LDVAL = 0;
	/*Clock period to know how many steps will take*/
	My_float_pit_t clock_period = 0.0F;
	/*Freq from PIT is equal to half the freq of clock*/
	system_clock = system_clock /2;
	/*Numer of periods the timer will count to delay_time*/
	clock_period = (1/system_clock);
	/*Calculating number of clocks to make*/
	LDVAL = (uint32_t)(delay / clock_period);
	/*Since clock starts counting from 0, negative addition of 1*/
	LDVAL = LDVAL - 1;

	/*First load the value on the channel*/
	/* Indicate the current value position steps of time */
	PIT->CHANNEL[pit_timer].LDVAL = LDVAL;
	/* Enable the timer */
	PIT->CHANNEL[pit_timer].TCTRL |= PIT_TCTRL_TEN_MASK;

}


/********************************************************************************************/
/*!
 	 \brief	 This function enable the clock signal of the pit
 	 SIM_SCGC6   BIT 23 corresponds to PIT clock enable
	 0 Clock Disabled
	 1 Clock Enabled
 	 \param[in]  void.
 	 \return void
 */
void PIT_clock_gating(void)
{
	SIM->SCGC6 |= PIT_CLOCK_GATING;
}

/********************************************************************************************/
/*!
 	 \brief	It return the status of the interrupt flag. This flag is a variable created by the programmer.
 	 It is not the flag related with bit TIF in PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK, this flag must be clear in the ISR of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit)
{
	uint8_t PITn_flag_status;
	switch(pit)
	{
	case(PIT_0):
		/*Clear programmer flag status*/
		PITn_flag_status = g_pit_inter_status.PIT_0_flag_status;
	break;
	case(PIT_1):
		/*Clear programmer flag status*/
		PITn_flag_status = g_pit_inter_status.PIT_1_flag_status;
	break;
	case(PIT_2):
		/*Clear programmer flag status*/
		PITn_flag_status = g_pit_inter_status.PIT_2_flag_status;
	break;
	case(PIT_3):
		PITn_flag_status = g_pit_inter_status.PIT_3_flag_status;
	break;
	default:
		PITn_flag_status = FALSE;
	break;
	}
	return 	PITn_flag_status;
}

/********************************************************************************************/
/*!
 	 \brief	Clears the interrupt flag created by the programmer.
 	 It is not the flag related with bit TIF in PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK, this flag must be clear in the ISR of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_clear_interrupt_flag(PIT_timer_t pit)
{
	switch(pit)
	{
	case(PIT_0):
		/*Clear programmer flag status*/
		g_pit_inter_status.PIT_0_flag_status &= CLEAR_FLAG;
	break;
	case(PIT_1):
		/*Clear programmer flag status*/
		g_pit_inter_status.PIT_1_flag_status &= CLEAR_FLAG;
	break;
	case(PIT_2):
		/*Clear programmer flag status*/
		g_pit_inter_status.PIT_2_flag_status &= CLEAR_FLAG;
	break;
	case(PIT_3):
		g_pit_inter_status.PIT_3_flag_status &= CLEAR_FLAG;
	break;

	}

}

/********************************************************************************************/
/*!
 	 \brief	It enables the PIT bit2
 	 Active on 0
 	 \param[in]  void.
 */
void PIT_enable(PIT_timer_t pit)
{
	/*Allows debug mode*/
	//PIT->MCR |= PIT_MCR_;
	/*Enable MODULE DISABLE */
	PIT->MCR &= ~(PIT_MCR_MDIS_MASK);
	PIT->CHANNEL[pit].TCTRL = PIT_TCTRL_TEN(1);


}

void PIT_stop(PIT_timer_t pit)
{

	PIT->CHANNEL[pit].TCTRL = PIT_TCTRL_TEN_SHIFT;
}

/********************************************************************************************/
/*!
 	 \brief	It enable de interrupt capabilities of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_enable_interrupt(PIT_timer_t pit)
{
	/*Timer interrupt Enable using Mask from kinetis64*/
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TIE_MASK;

}


void PIT_init(PIT_timer_t pit)
{
	PIT_clock_gating();
	PIT_enable(pit);
	PIT_enable_interrupt(pit);
}
