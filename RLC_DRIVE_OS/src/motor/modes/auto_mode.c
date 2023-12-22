/*
 * auto_mode.c
 *
 *  Created on: 22 déc. 2023
 *      Author: Ch.Leclercq
 */


#include "auto_mode.h"
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include <motor/motor.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <motor/drive_process/drive_process.h>
#include <motor/drive_process/drive_sequence.h>
#include <motor/motors_errors.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "AUTO"


static bool_t mode_running = FALSE;
static bool_t mode_stop_order = FALSE;

static bool_t check_stop_request(void);

static bool_t check_stop_request_nested(void)
{
    return mode_stop_order;
}


static bool_t check_stop_request(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    if(mode_stop_order == TRUE)
    {
        // Arrêt des moteurs
        motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
        // RAZ des flags du mode manuel
        mode_stop_order = FALSE;
        mode_running = FALSE;
        return TRUE;
    }
    else
        return FALSE;
}

void auto_mode_stop(void)
{
    mode_stop_order = TRUE;
}

bool_t auto_mode_is_running(void)
{
   return mode_running;
}


return_t auto_mode_process(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;

    return ret;
}








