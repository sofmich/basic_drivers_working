/**
	\file
	\brief
		This is the main file to emulate a system with a motor and a wave generator.
		All modules and main functions are called from here.
	\author
	\date	February 28, 2020
 */


#include "MK64F12.h"
#include "GPIO.h"
#include "bits.h"
#include "RGB.h"
#include "PIT.h"
#include "NVIC.h"
#include "stdio.h"

#define SYSTEM_CLOCK 	(21000000U)
#define DELAY			(1)

typedef void (*RGB_handler)(void);
RGB_handler led_change[] = {
		&LED_blue_ON,
		&LED_red_ON,
		&LED_white_ON
};

void led_control();
uint8_t g_counter = 0;

int main(void) {
	RGB_init();
	init_bit_as_falling_int_input(SW2_PORT,SW2_BIT);
	init_bit_as_falling_int_input(SW3_PORT, SW3_BIT);
	PIT_init(PIT_0);
	PIT_delay(PIT_0, 21000000, 2);

	/*GPIO callback init*/
	GPIO_callback_init(GPIO_A, 	&led_control);
	GPIO_callback_init(GPIO_C, 	&led_control);
	PIT_callback_init(PIT_0, &led_control);
	RGB_turn_led_on(RED_COLOR);

	/*Enable threshold to take priority level 10*/
	NVIC_set_basepri_threshold(PRIORITY_10);

	NVIC_enable_interrupt_and_priority(PORTC_IRQ, PRIORITY_4);
	NVIC_enable_interrupt_and_priority(PORTA_IRQ, PRIORITY_5);
	NVIC_enable_interrupt_and_priority(PIT_CH0_IRQ, PRIORITY_3);
	NVIC_global_enable_interrupts;

	while(1)
	{


	}
    return 0 ;
}

void led_control()
{
	printf("SI JALA \n");
	led_change[g_counter];
	g_counter++;
}
