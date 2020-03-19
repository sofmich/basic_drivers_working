/**
	\file RGB.h
	\brief
 		This is the source file for the RGB led control driver for Kinetis K64.
 		It contains all the implementations and definitions for RGB control.
	\author Sofía Salazar
	\date	Feb 15, 2020
	\todo
 */
#ifndef RGB_H_


#define RGB_H_


/*Constans for each color from the RGB*/
typedef enum{
	WHITE_COLOR, 	/*Definition for white color*/
	YELLOW_COLOR, 	/*Definition to create yellow color*/
	RED_COLOR, 		/*Definition for red color*/
	PURPLE_COLOR,   /*Definition for purple color*/
	BLUE_COLOR, 	/*Definition for blue color*/
	GREEN_COLOR 	/*Definition for green color*/
} rgb_led_color;

/**Bit value for RED LED*/
#define RED_LED_CONST (0x00400000)
/**Bit value for BLUE LED*/
#define BLUE_LED_CONST (0x00200000)
/**Bit value for GREEN LED*/
#define GREEN_LED_CONST (0x04000000)

/**Off constants to initalize port with leds off*/
#define GPIOB_OFF_CONST (0xFFFFFFFFU)
#define GPIOB_ON_CONST 	(0U)
#define GPIOE_OFF_CONST (0xFFFFFFFFU)
#define GPIOE_ON_CONST 	(0U)

/*Init configuration for RGB use*/

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function resumes the initialization of pines for RGB use.
 	 \param[in]  void
 	 \return void
 */
void RGB_init(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  led color that will be alternated
 	 \return void
 */
void RGB_turn_led_on(rgb_led_color led_color);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns off selected led.
 	 \param[in]  led color that will be alternated
 	 \return void
 */
void RGB_turn_led_off(rgb_led_color led_color);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  led color that will be alternated
 	 \return void
 */
void RGB_turn_all_leds_off();


/*Leds should be set ON within a separated function for State machine purpouses*/

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_white_ON(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_yellow_ON(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_red_ON(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_purple_ON(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_blue_ON(void);


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function turns on selected led.
 	 \param[in]  void
 	 \return void
 */
void LED_green_ON(void);


#endif /* RGB_H_ */
