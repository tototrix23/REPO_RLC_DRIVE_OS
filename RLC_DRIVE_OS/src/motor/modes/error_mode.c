/*
 * error_mode.c
 *
 *  Created on: 29 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "error_mode.h"
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include <motor/motor.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <motor/drive_process/drive_process.h>
#include <motor/drive_process/drive_sequence.h>
#include <motor/motors_errors.h>
#include <adc/adc.h>
#include <return_codes.h>

return_t error_mode_process(void)
{
    drive_control.running = TRUE;
    LOG_D(LOG_STD,"START");
    return_t ret = X_RET_OK;
    bool_t end = FALSE;

    while(!end)
    {
        tx_thread_sleep(1);
    }



    drive_control.running = FALSE;
    return ret;
}
