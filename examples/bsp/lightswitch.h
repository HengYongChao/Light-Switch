/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef LIGHT_SWITCH_H
#define LIGHT_SWITCH_H

#include <stdint.h>

// LEDs definitions for LightSwitch
#define LEDS_NUMBER    4

#define LED_START      21
#define LED_1          21
#define LED_2          22
#define LED_3          23
#define LED_4          24
#define LED_STOP       24

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define BSP_LED_3_MASK (1<<BSP_LED_3)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 4

#define BUTTON_START   17
#define BUTTON_1       17
#define BUTTON_2       18
#define BUTTON_3       19
#define BUTTON_4       20
#define BUTTON_STOP    20
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

#define BUTTONS_MASK   0x001E0000

#define RX_PIN_NUMBER  9
#define TX_PIN_NUMBER  10
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 8
#define HWFC           true

#define RELAY_ON		14
#define RELAY_OFF		13
#define RELAY2_ON		26
#define RELAY2_OFF		27

#define IR_SEND			11		
#define IR_REC			12


#define SOUND_EVENT_ON				50
#define SOUND_EVENT_OFF				30
#define MOTION_EVENT_ON				50
#define MOTION_EVENT_OFF			30
#define COMMUNICATE_EVENT_ON		3
#define COMMUNICATE_EVENT_OFF		30				
#define HEARTBEAT_EVENT_ON			1
#define HEARTBEAT_EVENT_OFF			30


typedef enum 
{
	SOUND_EVENT = 0,
	MOTION_EVENT,
	COMMUNICATE_EVENT,
	HEARTBEAT_EVENT,
	FIRMWAREUPDATE_EVENT
}led_event_e;


typedef enum
{
	SWITCH_ON,			// KEY STATUS ON
	SWITCH_OFF,	
	BL_ON,				//BACK LIGHT ON
	BL_OFF,
	BL_BREATH
}switch_event_e;


typedef struct 
{
	led_event_e		event_type;
	uint16_t		on_time;
	uint16_t		off_time;
	_Bool 			status;
}led_event_t;


//typedef struct
//{
//	
//#ifdef  ONE_SWITCH

//#elif	TWO_SWITCH

//#elif	THREE_SWITCH

//#else	//OTHERS_SWITCH

//#endif	
//			
//}switch_event_t;






#endif // LIGHT_SWITCH_H
