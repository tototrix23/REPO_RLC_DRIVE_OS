/*
 * leds.h
 *
 *  Created on: 20 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef LEDS_LEDS_H_
#define LEDS_LEDS_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


#define LED_ON    1
#define LED_OFF   0

void leds_init(void);
void led_error_motor_on(void);
void led_error_motor_off(void);



#endif /* LEDS_LEDS_H_ */
