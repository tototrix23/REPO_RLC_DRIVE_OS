/*
 * leds.c
 *
 *  Created on: 20 févr. 2024
 *      Author: Ch.Leclercq
 */
#include "leds.h"

void leds_init(void)
{
    led_error_off();
}

void led_error_on(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ERROR,LED_ON );
}

void led_error_off(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, LED_ERROR,LED_OFF );
}


