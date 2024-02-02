/*
 * error_mode.c
 *
 *  Created on: 31 janv. 2024
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
#include <motor/check/motor_check.h>
#include <adc/adc.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "ERROR"

static return_motor_cplx_t error_test_1H(void);
static return_motor_cplx_t error_test_1L(void);
static void scroll_stop(void);

static void scroll_stop(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
}

static return_motor_cplx_t error_test_1H(void)
{
    return_motor_cplx_t ret;
    return_motor_cplx_update(&ret,X_RET_OK);


    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    uint32_t error_fsp_counter = 0;
    error_test_1_start:
    h_time_update(&ts);


    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);

    motor_drive_sequence(&ptr->sequences.error_check.test1H,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    bool_t end = FALSE;
    bool_t ts_elasped;
    while(!end)
    {
        CHECK_STOP_REQUEST_NESTED_CPLX();
        h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
        if(ts_elasped == TRUE)
            end = TRUE;

        if(motors_instance.motorH->error != 0x00 )
        {
           error_fsp_counter++;
           if(error_fsp_counter >= 4)
           {
               end = TRUE;
               return_motor_cplx_update(&ret,F_RET_MOTOR_ERROR_API_FSP);
               LOG_E(LOG_STD,"Error FSP 0x%02x",motors_instance.motorH->error);
               scroll_stop();
               return ret;
           }
           else
           {
               LOG_W(LOG_STD,"Error FSP 0x%02x",motors_instance.motorH->error);
               scroll_stop();
               motors_instance.motorH->error = 0;
               delay_ms(100);
               goto error_test_1_start;
           }
        }
    }
    scroll_stop();

    delay_ms(200);
    int32_t pulses;
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulses);
    pulses = abs(pulses);
    LOG_I(LOG_STD,"Counter %d",pulses);

    return ret;
}

static return_motor_cplx_t error_test_1L(void)
{
    return_motor_cplx_t ret;
    return_motor_cplx_update(&ret,X_RET_OK);


    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    uint32_t error_fsp_counter = 0;
    error_test_1_start:
    h_time_update(&ts);


    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);

    motor_drive_sequence(&ptr->sequences.error_check.test1L,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    bool_t end = FALSE;
    bool_t ts_elasped;
    while(!end)
    {
        CHECK_STOP_REQUEST_NESTED_CPLX();
        h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
        if(ts_elasped == TRUE)
            end = TRUE;

        if(motors_instance.motorL->error != 0x00 )
        {
            error_fsp_counter++;
            if(error_fsp_counter >= 4)
            {
               end = TRUE;
               return_motor_cplx_update(&ret,F_RET_MOTOR_ERROR_API_FSP);
               LOG_E(LOG_STD,"Error FSP 0x%02x",motors_instance.motorL->error);
               scroll_stop();
               return ret;
            }
            else
            {
               LOG_W(LOG_STD,"Error FSP 0x%02x",motors_instance.motorL->error);
               scroll_stop();
               motors_instance.motorL->error = 0;
               delay_ms(100);
               goto error_test_1_start;
            }
        }
    }
    scroll_stop();

    delay_ms(200);
    int32_t pulses;
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,&pulses);
    pulses = abs(pulses);
    LOG_I(LOG_STD,"Counter %d",pulses);
    return ret;
}

return_t error_mode_process(void)
{
    drive_control.running = TRUE;
    return_t ret = X_RET_OK;
    bool_t end = FALSE;
    st_system_motor_t sys_mot;
    memset(&sys_mot,0x00,sizeof(sys_mot));

    LOG_D(LOG_STD,"Start");
    ret = motor_check();
    if(ret != X_RET_OK)
    {
        do
        {
            CHECK_STOP_REQUEST();
            tx_thread_sleep(1);
        }while(!end);
    }


    return_motor_cplx_t retcplx = error_test_1H();
    if(flag_overcurrent_vm == TRUE)
    {
       sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
       system_set_motor(sys_mot);
       do
       {
           CHECK_STOP_REQUEST();
           tx_thread_sleep(1);
       }while(!end);
    }
    if(retcplx.code != X_RET_OK)
    {
         if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_HALL_PATTERN ||
                 retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_PATTERN)
         {
             sys_mot.error_lvl2.bits.error_pattern_h = TRUE;
         }
         else if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_TIMEOUT)
         {
             sys_mot.error_lvl2.bits.timeout_pulses_h = TRUE;
         }
         else
             sys_mot.error_lvl2.bits.unknown = TRUE;
    }
    system_set_motor(sys_mot);

    retcplx = error_test_1L();
    if(flag_overcurrent_vm == TRUE)
    {
       sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
       system_set_motor(sys_mot);
       do
       {
           CHECK_STOP_REQUEST();
           tx_thread_sleep(1);
       }while(!end);
    }

    if(retcplx.code != X_RET_OK)
    {
         if(retcplx.fsp_motorL_error_code == MOTOR_ERROR_HALL_PATTERN ||
                 retcplx.fsp_motorL_error_code == MOTOR_ERROR_BEMF_PATTERN)
         {
             sys_mot.error_lvl2.bits.error_pattern_l = TRUE;
         }
         else if(retcplx.fsp_motorL_error_code == MOTOR_ERROR_BEMF_TIMEOUT)
         {
             sys_mot.error_lvl2.bits.timeout_pulses_l = TRUE;
         }
         else
             sys_mot.error_lvl2.bits.unknown = TRUE;
    }
    system_set_motor(sys_mot);



    if(sys_mot.error_lvl1.value == 0x00 &&
       sys_mot.error_lvl2.value == 0x00 &&
       sys_mot.error_lvl3.value == 0x00)
    {
        LOG_W(LOG_STD,"No error detected -> starting init mode")
        drive_control.running = FALSE;
        set_drive_mode(MOTOR_INIT_MODE);
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error detected")
        do
        {
            CHECK_STOP_REQUEST();
            tx_thread_sleep(1);
        }while(!end);
    }


    drive_control.running = FALSE;
    return ret;
}


