/*
 * motor_mode.c
 *
 *  Created on: 17 oct. 2023
 *      Author: Christophe
 */
#include <remotectrl/remotectrl.h>
#include "motor.h"
#include "drive_mode.h"
#include <motor/modes/init_mode.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "drive"

return_t set_drive_mode(drive_mode_t mode)
{
    drive_mode_t current_mode = motors_instance.mode;
    if (mode != current_mode)
    {
        switch (mode)
        {
            case MOTOR_UNKNOWN_MODE:
                LOG_D(LOG_STD,"Set unknown mode");
            break;

            case MOTOR_MANUAL_MODE:
                LOG_D(LOG_STD,"Set manual mode");
                manual_mode_start();
            break;

            case MOTOR_INIT_MODE:
                LOG_D(LOG_STD,"Set init mode");
            break;

            case MOTOR_AUTO_MODE:
                LOG_D(LOG_STD,"Set auto mode");
            break;
        }
        motors_instance.mode = mode;
    }
    return X_RET_OK;
}
