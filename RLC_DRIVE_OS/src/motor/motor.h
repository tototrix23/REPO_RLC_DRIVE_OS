/*
 * moteur.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_MOTOR_MOTOR_H_
#define APPLICATION_MOTOR_MOTOR_H_

#include <stdint.h>
#include <rm_motor_api.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <motor/drive_mode.h>
#include <motor/motor_type.h>

typedef struct st_motor_t
{
    motor_ext_technology_t motor_technology;
    motor_instance_t *motor_ctrl_instance;
    motor_120_driver_instance_t *motor_driver_instance;
    motor_120_control_instance_t *motor_hall_instance;
    uint16_t status;
    uint16_t error;
}st_motor_t;


typedef struct st_drive_t
{
    drive_mode_t mode;
    st_motor_t *motorH;
    st_motor_t *motorL;
    st_motor_t *motors[2];
    motor_profil_t profil;
}st_drive_t;

extern st_drive_t motors_instance;


void motor_structures_init(void);

void motor_init_fsp(void);

return_t motor_is_speed_achieved(st_motor_t *mot,bool_t *res);
return_t motor_wait_stop(st_motor_t *mot);


#endif /* APPLICATION_MOTOR_MOTOR_H_ */
