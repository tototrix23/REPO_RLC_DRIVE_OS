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
       uint16_t value :16;
       struct{
           bool_t overcurrent_vm :1;
           bool_t vcc_hall_h :1;
           bool_t vcc_hall_l :1;
           bool_t config_driver_h :1;
           bool_t config_driver_l :1;
           bool_t fsp_h :1;
           bool_t fsp_l :1;
       }bits;
   }error_lvl1;

   union
   {
       uint16_t value:16;
       struct
       {
           bool_t error_pattern_h :1;
           bool_t error_pattern_l :1;
           bool_t timeout_pulses_h :1;
           bool_t timeout_pulses_l :1;
           bool_t unknown :1;
       }bits;
   }error_lvl2;


   union
   {
      uint16_t value:16;
      struct
      {
          bool_t damaged_panels :1;
          bool_t motor_driving_h :1;
          bool_t motor_driving_l :1;
          bool_t unknown :1;
      }bits;
   }error_lvl3;

}st_system_motor_t;


typedef struct st_system_t
{
    st_system_motor_t motor;
}st_system_t;

extern bool_t flag_overcurrent_vm;
extern st_system_t system_inst;

void system_init(void);
void system_set_motor(st_system_motor_t *ptr_value);
st_system_t system_get(void);
void system_clear_all(void);
void system_clear_motor(void);
bool_t system_check_error(void);

#endif /* SYSTEM_SYSTEM_H_ */

