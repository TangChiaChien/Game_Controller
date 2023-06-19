/*
 * File: JOYSTICK_cfg.c
 * Driver Name: [[ JoyStick ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */
#include "JOYSTICK.h"

const JoyStick_CfgType JoyStick_CfgParam[JOYSTICK_UNITS] =
{
	// JoyStick unit 1 Configurations
    {
	    GPIOC,
		GPIOC,
		GPIO_PIN_0,
		GPIO_PIN_2,
		ADC1,
		ADC_CHANNEL_10,
		ADC_CHANNEL_12
	},
	// JoyStick unit 2 Configurations
    {
	    GPIOB,
		GPIOB,
		GPIO_PIN_0,
		GPIO_PIN_1,
		ADC2,
		ADC_CHANNEL_8,
		ADC_CHANNEL_9
	}
};

