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
#include <motor/modes/manual_mode.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "drive"

return_t set_drive_mode(drive_mode_t mode)
{
    drive_mode_t current_mode = motors_instance.mode;
    if (mode != current_mode)
    {

        // Le mode manuel est une boucle infinie.
        // Il faut envoyer un message au mode manuel pour lui demander de
        // s'arreter
        if(current_mode == MOTOR_MANUAL_MODE)
        {
            // Envoie de la demande au mode manuel
            manual_mode_stop();
            // Attente de sortie du mode manuel
            while(manual_mode_is_running() == TRUE)
                tx_thread_sleep(1);
        }


        switch (mode)
        {
            case MOTOR_UNKNOWN_MODE:
                LOG_D(LOG_STD,"Set unknown mode");
            break;

            case MOTOR_MANUAL_MODE:
                LOG_D(LOG_STD,"Set manual mode");

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
