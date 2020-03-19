/**
	\file PIT.h
	\brief
 		This is the source file of PIT control driver for Kinetis K64.
 		It contains all the implementations and definitions for PIT control.
 		It includes interruption vectors and flags.
	\author Sofía Salazar
	\date	Feb 15, 2020
	\todo
 */
#ifndef PIT_H_
#define PIT_H_

#include "MK64F12.h"

#define FLAG_PIT_0 			(0x01U)
#define FLAG_PIT_1 			(0X02U)
#define FLAG_PIT_2			(0x04U)
#define FLAG_PIT_3			(0x08U);

/*! This is the constant value for maping the SCGC6 bit 23 for PITs**/
#define PIT_CLOCK_GATING 	(0x800000U)

typedef float My_float_pit_t;

/*! This enumerated constant are used to select the PIT to be used*/
typedef enum {PIT_0,PIT_1,PIT_2,PIT_3} PIT_timer_t;

/*! This struct contains flags to be used by the programmer*/
typedef struct
{
	uint8_t PIT_0_flag_status;
	uint8_t PIT_1_flag_status;
	uint8_t PIT_2_flag_status;
	uint8_t PIT_3_flag_status;
} pit_interrupt_flags_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function configure the callback on pits though a handler.
 	 \param[in]  pit_timer channel to be used.
	 \param[in]  hanlder function to make the callback
 	 \return void
 */
void PIT_callback_init(PIT_timer_t pit_timer,void(*handler)(void));

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function configure the PIT to generate a delay base on the system clock.
 	 It is important to note that this strictly is not device driver since everything is
 	 contained in a single function,  in general you have to avoid this practices, this only
 	 for the propose of the homework
 	 \param[in]  pit_timer channel to be used.
	 \param[in]  system_clock system clock use in the K64 (defult = 21e6).
	 \param[in]  delay the amount of time the delay the microcontroller
 	 \return void
 */
void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enable the clock signal of the pit
 	 \param[in]  void.
 	 \return void
 */
void PIT_clock_gating(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	It return the status of the interrupt flag. This flag is a variable created by the programmer.
 	 It is not the flag related with bit TIF in PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK, this flag must be clear in the ISR of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	Clears the interrupt flag created by the programmer.
 	 It is not the flag related with bit TIF in PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK, this flag must be clear in the ISR of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_clear_interrupt_flag(PIT_timer_t pit);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	It enables the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_enable(PIT_timer_t pit);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	It enable de interrupt capabilities of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_stop(PIT_timer_t pit);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	It enable de interrupt capabilities of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_enable_interrupt(PIT_timer_t pit);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	It enable de interrupt capabilities of the PIT
 	 \param[in]  void.
 	 \return uint8_t flag status
 */
void PIT_init(PIT_timer_t pit);
#endif /* PIT_H_ */
