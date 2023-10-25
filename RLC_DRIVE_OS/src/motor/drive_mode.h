/*
 * motor_mode.h
 *
 *  Created on: 17 oct. 2023
 *      Author: Christophe
 */

#ifndef APPLICATION_MOTOR_DRIVING_MODE_H_
#define APPLICATION_MOTOR_DRIVING_MODE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


typedef enum  e_drive_mode
{
    MOTOR_UNKNOWN_MODE = 0,
    MOTOR_MANUAL_MODE  = 1,
    MOTOR_INIT_MODE    = 2,
    MOTOR_AUTO_MODE    = 3,
} drive_mode_t;


return_t set_drive_mode(drive_mode_t mode);


#endif /* APPLICATION_MOTOR_DRIVING_MODE_H_ */
