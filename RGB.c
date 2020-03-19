/**
	\file RGB.c
 	\brief
 		This is the source file for the RGB led control for Kinetis K64.
 		It contains all the implementations for configuring PORTB and PORTE to RGB control.
		This is the API for the RGB control.
   	\author	Sofía Salazar, Omar Soto
   	\date Feb 14, 2020
   	\todo
 */

#include "MK64F12.h"
#include "RGB.h"
#include "bits.h"
#include "GPIO.h"
#include "stdint.h"

void RGB_init()
{
	/*Define the PCR for each port using the MUX1*/
	gpio_pin_control_register_t pinControlRegisterGPIOBpin21 = GPIO_MUX1;
	gpio_pin_control_register_t pinControlRegisterGPIOBpin22 = GPIO_MUX1;
	gpio_pin_control_register_t pinControlRegisterGPIOEpin26 = GPIO_MUX1;

	/*Activating the GPIO clock gating from ports A,B,C,E*/
	GPIO_clock_gating(GPIO_B);
	GPIO_clock_gating(GPIO_E);

	/*Pin control configuration for blue led from port b*/
	GPIO_pin_control_register(GPIO_B,bit_21, &pinControlRegisterGPIOBpin21);
	/*Pin control configuration for red led from port b*/
	GPIO_pin_control_register(GPIO_B,bit_22, &pinControlRegisterGPIOBpin22);
	/*Pin control configuration for green led from port b*/
	GPIO_pin_control_register(GPIO_E,bit_26, &pinControlRegisterGPIOEpin26);

	/*Assigns a safe value to each port*/
	GPIO_set_pin(GPIO_B, bit_21); //blue
	GPIO_set_pin(GPIO_B, bit_22); //red
	GPIO_set_pin(GPIO_E, bit_26); //green

	/*Configures bit(s) from each port as an input or output port*/
	GPIO_data_direction_pin(GPIO_B,	GPIO_OUTPUT,bit_21);
	GPIO_data_direction_pin(GPIO_B,	GPIO_OUTPUT,bit_22);
	GPIO_data_direction_pin(GPIO_E,	GPIO_OUTPUT,bit_26);

}
void RGB_turn_led_on(rgb_led_color led_color)
{
	switch(led_color) /*Alternate according to each color*/
	{
	case WHITE_COLOR: /**When white color selected, select correct RGB led*/
	{
		GPIO_clear_pin(GPIO_E, bit_26);
		GPIO_clear_pin(GPIO_B,  bit_21);
		GPIO_clear_pin(GPIO_B,  bit_22);
		break;
	}
	case YELLOW_COLOR: /**When yellow color selected, turn ON green and red led*/
	{
		GPIO_set_pin(GPIO_B,  bit_21);
		GPIO_clear_pin(GPIO_B,  bit_22);
		GPIO_clear_pin(GPIO_E,  bit_26);
		break;
	}
	case RED_COLOR: /**When red color selected, turn ON only red*/
	{
		GPIO_set_pin(GPIO_E, bit_26);
		GPIO_set_pin(GPIO_B, bit_21);
		GPIO_clear_pin(GPIO_B, bit_22);
		break;
	}
	case PURPLE_COLOR:  /**When purple color selected, turn OFF only green led*/
	{
		GPIO_set_pin(GPIO_E, bit_26);
		GPIO_clear_pin(GPIO_B, bit_21);
		GPIO_clear_pin(GPIO_B, bit_22);
		break;
	}

	case BLUE_COLOR:  /**When blue color selected, turn ON only blue led*/
	{
		GPIO_set_pin(GPIO_B, bit_22);
		GPIO_set_pin(GPIO_E, bit_26);
		GPIO_clear_pin(GPIO_B, bit_21);
		break;
	}

	case GREEN_COLOR: /**When green color selected, turn ON only green led*/
	{
		GPIO_set_pin(GPIO_B, bit_22);
		GPIO_set_pin(GPIO_B, bit_21);
		GPIO_clear_pin(GPIO_E, bit_26);
		break;
	}

	default:
		RGB_turn_all_leds_off();
	}
}

void RGB_turn_led_off(rgb_led_color led_color)
{
	switch(led_color) /*Alternate acording to led color selected*/
	{
	case RED_COLOR: /*When RED color selected, write its constant to PORTB to turn off*/
		GPIO_write_port(GPIO_B,(RED_LED_CONST));
		break;
	case BLUE_COLOR: /*When BLUE color selected, write its constant to PORTB to turn off*/
		GPIO_write_port(GPIO_B,(BLUE_LED_CONST));
		break;
	case GREEN_COLOR: /*When GREEN color selected, write its constant to PORTB to turn off*/
		GPIO_write_port(GPIO_E,(GREEN_LED_CONST));
		break;
	default:
		RGB_turn_all_leds_off();
		break;
	}
}

void RGB_turn_all_leds_off()
{
	/*Turn all leds off writin on their port each constant ON(0) OFF(1)*/
	GPIO_set_pin(GPIO_E, bit_26);
	GPIO_set_pin(GPIO_B, bit_22);
	GPIO_set_pin(GPIO_B, bit_21);

}

void LED_white_ON()
{
	RGB_turn_led_on(WHITE_COLOR);
}
void LED_yellow_ON()
{
	RGB_turn_led_on(YELLOW_COLOR);
}
void LED_red_ON()
{
	RGB_turn_led_on(RED_COLOR);
}
void LED_purple_ON()
{
	RGB_turn_led_on(PURPLE_COLOR);
}
void LED_blue_ON()
{
	RGB_turn_led_on(BLUE_COLOR);
}
void LED_green_ON()
{
	RGB_turn_led_on(GREEN_COLOR);
}
