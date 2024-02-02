/*
 * system.h
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>

typedef struct st_system_motor_t
{
   union
   {
       uint16_t value;
       struct{
           bool_t overcurrent_vm;
           bool_t vcc_hall_h;
           bool_t vcc_hall_l;
           bool_t config_driver_h;
           bool_t config_driver_l;
           bool_t fsp_h;
           bool_t fsp_l;
       }bits;
   }error_lvl1;

   union
   {
       uint16_t value;
       struct
       {
           bool_t error_pattern_h;
           bool_t error_pattern_l;
           bool_t timeout_pulses_h;
           bool_t timeout_pulses_l;
           bool_t unknown;
       }bits;
   }error_lvl2;


   union
   {
      uint16_t value;
      struct
      {
          bool_t damaged_panels;
          bool_t unknown;
      }bits;
   }error_lvl3;

}st_system_motor_t;


typedef struct st_system_t
{

   /*struct
   {
       union
       {
           uint16_t value;
           struct{
               bool_t overcurrent_vm;
               bool_t vcc_hall_h;
               bool_t vcc_hall_l;
               bool_t config_driver_h;
               bool_t config_driver_l;
               bool_t fsp_h;
               bool_t fsp_l;
           }bits;
       }error_hw;
       uint16_t error_fsp_h;
       uint16_t error_fsp_l;
       uint16_t error_sw;
   }motor;*/

    st_system_motor_t motor;

}st_system_t;

extern bool_t flag_overcurrent_vm;
extern st_system_t system_inst;

void system_init(void);
void system_set_motor(st_system_motor_t value);
st_system_t system_get(void);


#endif /* SYSTEM_SYSTEM_H_ */

