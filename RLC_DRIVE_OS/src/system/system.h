/*
 * system.h
 *
 *  Created on: 29 janv. 2024
 *      Author: Ch.Leclercq
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#include <hal_data.h>
#include <_core/c_common.h>
#include <motor/motor.h>

typedef struct st_system_t
{
   struct
   {
       bool_t overcurrent_hw;
       bool_t vcc_hallh;
       bool_t vcc_halll;
       bool_t moth_driver_access;
       bool_t motl_driver_access;
       int16_t moth_fsp;
       int16_t motl_fsp;
       bool_t relay;
   }error_hw;

   int16_t error_sw;
}st_system_t;

extern st_system_t system_instance;

void system_init(void);

#endif /* SYSTEM_SYSTEM_H_ */
