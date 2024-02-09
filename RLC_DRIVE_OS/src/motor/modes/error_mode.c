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
static return_motor_cplx_t error_test_2(void);
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
    error_test_1L_start:
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
               goto error_test_1L_start;
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

static return_motor_cplx_t error_test_2(void)
{
    return_motor_cplx_t ret;
    return_motor_cplx_update(&ret,X_RET_OK);

    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    c_timespan_t ts_pulses[2];
    volatile uint8_t index = 0;
    const uint8_t stat_process = 1;
    const uint8_t stat_finished = 2;
    volatile uint32_t error_fsp_counter[2] = {0,0};
    volatile uint16_t error_fsp[2] = {0,0};
    volatile uint8_t status[2];
    volatile int32_t pulses1[2];
    volatile int32_t pulses2[2];
    status[0] = stat_process;
    status[1] = stat_process;

    h_time_update(&ts);
    h_time_update(&ts_pulses[0]);
    h_time_update(&ts_pulses[1]);


    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);
    pulses1[0]=0;
    pulses1[1]=0;
    pulses2[0]=0;
    pulses2[1]=0;

    LOG_D(LOG_STD,"br");
    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    delay_ms(500);
    //motors_instance.motorH->error=0x00;
    //motors_instance.motorL->error=0x00;
    LOG_D(LOG_STD,"test2");
    motor_drive_sequence(&ptr->sequences.error_check.test2,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

    while(1)
    {
        CHECK_STOP_REQUEST_NESTED_CPLX();

        for(index=0;index<2;index++)
        {
            st_motor_t *mot = motors_instance.motors[index];

            // Si le traitement est en cours
            if(status[index] == stat_process)
            {
                // Gestion des erreurs de FSP
                if(mot->error != 0x00 )
                {
                    error_fsp_counter[index]++;
                    if(error_fsp_counter[index] >= 4)
                    {
                       status[index] = stat_finished;
                       error_fsp[index] = mot->error;
                       LOG_E(LOG_STD,"Error FSP 0x%02x",mot->error);
                    }
                    else
                    {
                       LOG_W(LOG_STD,"Error FSP 0x%02x",mot->error);
                       mot->error = 0;
                       LOG_D(LOG_STD,"1");
                       motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                       LOG_D(LOG_STD,"11-%d",sequence_result.status);
                       LOG_D(LOG_STD,"12-%d",sequence_result.errorH);
                       LOG_D(LOG_STD,"13-%d",sequence_result.errorL);
                       delay_ms(100);
                       h_time_update(&ts_pulses[0]);
                       h_time_update(&ts_pulses[1]);
                       LOG_D(LOG_STD,"2");
                       motor_drive_sequence(&ptr->sequences.error_check.test2,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                       //goto error_test_2_start;
                    }
                }

                // Gestion de la comparaison sur les points
                bool_t ts_pulses_elasped = FALSE;
                h_time_is_elapsed_ms(&ts_pulses[index], 1000, &ts_pulses_elasped);
                if(ts_pulses_elasped == TRUE)
                {
                    h_time_update(&ts_pulses[index]);
                    mot->motor_ctrl_instance->p_api->pulsesGet(mot->motor_ctrl_instance->p_ctrl,&pulses2[index]);
                    if(abs(pulses2[index] - pulses1[index]) < 5)
                    {
                        LOG_D(LOG_STD,"Motor %d no more pulses",index);
                        status[index] = stat_finished;
                    }
                    pulses1[index] = pulses2[index];
                }

                //

            }
        }

        if(status[0] != stat_finished && motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW)
        {
            LOG_D(LOG_STD,"MotorH reverse detected");
            status[0] = stat_finished;
        }

        if(status[1] != stat_finished && motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
        {
            LOG_D(LOG_STD,"MotorL reverse detected");
            status[1] = stat_finished;
        }

        bool_t ts_elasped;
        h_time_is_elapsed_ms(&ts, 30000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            scroll_stop();
            LOG_E(LOG_STD,"Panel damaged");
            return_motor_cplx_update(&ret,F_RET_PANELS_DAMAGED);
        }

        if(status[0] == stat_finished && status[1] == stat_finished)
        {
            scroll_stop();
            if(error_fsp[0] != 0x00 || error_fsp[1]!= 0x00)
            {
                LOG_E(LOG_STD,"Error FSP");
                ret.code = F_RET_MOTOR_ERROR_API_FSP;
                ret.fsp_motorH_error_code = error_fsp[0];
                ret.fsp_motorL_error_code = error_fsp[1];
                return ret;
            }
            else
            {
                LOG_D(LOG_STD,"No error");
                return_motor_cplx_update(&ret,F_RET_OK);
                return ret;
            }
        }
    }
    return_motor_cplx_update(&ret,F_RET_OK);
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


    volatile return_motor_cplx_t retcplx = error_test_1H();
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
           sys_mot.error_lvl2.value == 0x00)
    {
        retcplx = error_test_2();
        volatile uint8_t x=0;
    }

    if(sys_mot.error_lvl1.value == 0x00 &&
       sys_mot.error_lvl2.value == 0x00 &&
       sys_mot.error_lvl3.value == 0x00)
    {
        LOG_W(LOG_STD,"No error detected -> starting init mode");
        drive_control.running = FALSE;
        set_drive_mode(MOTOR_INIT_MODE);
        return X_RET_OK;
    }
    else
    {
        LOG_E(LOG_STD,"Error detected");
        do
        {
            CHECK_STOP_REQUEST();
            tx_thread_sleep(1);
        }while(!end);
    }


    drive_control.running = FALSE;
    return ret;
}


