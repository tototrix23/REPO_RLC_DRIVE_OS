/*
 * init_mode.c
 *
 *  Created on: 18 oct. 2023
 *      Author: Ch.Leclercq
 */
#include "init_mode.h"
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
#define LOG_MODULE    "INIT"



static return_t init_strectch(void);
static return_t init_enrl(void);
static return_t init_finish(void);
static void scroll_stop(void);





return_t init_mode_process(void) {
    drive_control.running = TRUE;
    volatile return_t ret = X_RET_OK;
	motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    bool_t end = FALSE;
    bool_t ts_elasped;
    int32_t pulsesH;
    int32_t pulsesL;
    h_time_update(&ts);

    motors_instance.error = MOTORS_ERROR_NONE;


    //----------------------------------------------------------------------------------------------
    // Check de la partie moteur
    //----------------------------------------------------------------------------------------------
    ret = motor_check();
    if(ret != X_RET_OK)
    {
        drive_control.running=FALSE;
        set_drive_mode(MOTOR_ERROR_MODE);
        return X_RET_OK;
    }

    //----------------------------------------------------------------------------------------------
    // Arrêt des moteurs
    //----------------------------------------------------------------------------------------------
    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    end = FALSE;
    while(!end)
    {
        CHECK_STOP_REQUEST();
        h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);
            end = TRUE;
        }
        tx_thread_sleep(1);
    }


    //----------------------------------------------------------------------------------------------
    // Mise en tension des affiches
    //----------------------------------------------------------------------------------------------
    ret = init_strectch();
    CHECK_STOP_REQUEST();
    if(ret != X_RET_OK)
        MOTOR_SET_ERROR_EVENT_AND_RETURN(MOTOR_INIT_MODE,ret);
    //----------------------------------------------------------------------------------------------
    // Recherche bande mère haute
    //----------------------------------------------------------------------------------------------
    ret = init_enrl();
    CHECK_STOP_REQUEST();
    if(ret != X_RET_OK)
        MOTOR_SET_ERROR_EVENT_AND_RETURN(MOTOR_INIT_MODE,ret);
    init_finish();
    delay_ms(1000);
    CHECK_STOP_REQUEST();
    if(ret != X_RET_OK)
        MOTOR_SET_ERROR_EVENT_AND_RETURN(MOTOR_INIT_MODE,ret);
    drive_control.running=FALSE;
    ptr->panels.index = 0;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);
    set_drive_mode(MOTOR_AUTO_MODE);
    tx_thread_sleep(1);
    return X_RET_OK;


}




static return_t init_strectch(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);


    motor_phase_t *phase = 0x00;
    c_linked_list_get_by_index(&motors_instance.profil.sequences.init.stretch1,1,(void**)&phase);
    motor_drive_sequence(&ptr->sequences.init.stretch1,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);


    end = FALSE;
    while(!end)
    {
        CHECK_STOP_REQUEST_NESTED();
        h_time_is_elapsed_ms(&ts, 500, &ts_elasped);
        if(ts_elasped == TRUE)
            end = TRUE;
    }
    end = FALSE;

    c_linked_list_get_by_index(&motors_instance.profil.sequences.init.stretch2,1,(void**)&phase);
    motor_ext_settings_t motorH_stretch_settings;
    motor_ext_settings_t motorL_stretch_settings;
    memcpy(&motorH_stretch_settings,&phase->params_motors[0].non_regulated.settings,sizeof(motor_ext_settings_t));
    memcpy(&motorL_stretch_settings,&phase->params_motors[1].non_regulated.settings,sizeof(motor_ext_settings_t));
    motorH_stretch_settings.timeout_hall_ms=0;
    motorL_stretch_settings.timeout_hall_ms=0;


    motor_drive_sequence(&ptr->sequences.init.stretch2,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    end = FALSE;
    while(!end)
    {
        CHECK_STOP_REQUEST_NESTED();

        // On verifie pour savoir si les moteurs tournent toujours
        // au bout de 20 secondes (anormal).
        h_time_is_elapsed_ms(&ts, 15000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            // Arrêt des moteurs
            motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            // Macro d'enregistrement d'erreur et code de retour
            MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_DAMAGED_PANEL,F_RET_MOTOR_INIT_DAMAGED_PANELS);

        }

        // Verification des erreurs remontées par la librairie
        if((motors_instance.motorH->error != 0x00) ||
           (motors_instance.motorL->error != 0x00))
        {
            volatile int8_t expected_errorH_ok=0;
            volatile int8_t expected_errorL_ok=0;

            if(motors_instance.motorH->error != 0x00)
            {
                if( (expected_errorH_ok == 0) &&
                        ((motors_instance.motorH->error == MOTOR_ERROR_OVER_CURRENT_SW) ||
                         (motors_instance.motorH->error == MOTOR_ERROR_BEMF_TIMEOUT))
                  )
                {
                    motors_instance.motorH->motor_ctrl_instance->p_api->settingsSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,motorH_stretch_settings);
                    expected_errorH_ok = 1;
                }
                else
                    expected_errorH_ok = -1;
            }
            if(motors_instance.motorL->error != 0x00)
            {
                if( (expected_errorL_ok == 0) &&
                        ((motors_instance.motorL->error == MOTOR_ERROR_OVER_CURRENT_SW) ||
                    (motors_instance.motorL->error == MOTOR_ERROR_BEMF_TIMEOUT))
                  )
                {
                    motors_instance.motorL->motor_ctrl_instance->p_api->settingsSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,motorL_stretch_settings);
                    expected_errorL_ok = 1;
                }
                else
                    expected_errorL_ok = -1;
            }

            if((expected_errorH_ok==1 && expected_errorL_ok==1)||
                    motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW ||
                    motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
            {
                /*if(motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW)
                   LOG_D(LOG_STD,"motorH reverse detected");
                if(motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
                   LOG_D(LOG_STD,"motorL reverse detected");

                LOG_D(LOG_STD,"motorH: 0x%X ,motorL: 0x%X",motors_instance.motorH->error,motors_instance.motorL->error);*/

                LOG_D(LOG_STD,"current %d mA",adc_inst.instantaneous.iin);
                end = TRUE;
            }
            else if(expected_errorH_ok==-1 || expected_errorL_ok==-1)
            {
                LOG_E(LOG_STD,"motorH: 0x%X ,motorL: 0x%X",motors_instance.motorH->error,motors_instance.motorL->error);
                motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                // Macro d'enregistrement d'erreur et code de retour
                MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_GENERIC,F_RET_MOTOR_INIT_STRETCH);
            }
        }
        //
        tx_thread_sleep(1);
    }
    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

    return ret;
}


static return_t init_finish(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    delay_ms(200);
    int32_t pulsesH1,pulsesH2;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH1);
    //LOG_D(LOG_STD,"P1 %d",pulsesH1);
    motor_drive_sequence(&ptr->sequences.init.enrl_finish,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    delay_ms(1500);
    scroll_stop();
    delay_ms(200);
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH2);
    LOG_D(LOG_STD,"Delta %d",pulsesH2-pulsesH1);
    return ret;
}



static return_t init_enrl(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts_start;
    c_timespan_t ts;
    c_timespan_t ts2;
    bool_t start_finished = FALSE;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);
    h_time_update(&ts_start);

    int32_t pulsesH1;
    int32_t pulsesH2;

    //flag_overcurrent_vm = TRUE;

    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH1);
    motor_drive_sequence(&ptr->sequences.init.enrl_start,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    delay_ms(500);
    h_time_update(&ts2);
    while(!end)
    {
        CHECK_STOP_REQUEST_NESTED();



        h_time_is_elapsed_ms(&ts_start, 1000, &ts_elasped);
        if(ts_elasped == TRUE && start_finished == FALSE)
        {
            start_finished = TRUE;
            motor_drive_sequence(&ptr->sequences.init.enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
        }

        h_time_is_elapsed_ms(&ts, 60000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            // Arrêt des moteurs
            motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            // Macro d'enregistrement d'erreur et code de retour
            MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_TIMEOUT_SEARCHING_BASE_H,F_RET_MOTOR_INIT_TIMEOUT_BASE_H);
        }



        if((motors_instance.motorH->error != 0x00) )
        {
            if(motors_instance.motorH->error != 0x00)
            {
                if((motors_instance.motorH->error == MOTOR_ERROR_OVER_CURRENT_SW) ||
                   (motors_instance.motorH->error == MOTOR_ERROR_BEMF_TIMEOUT))

                {
                    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                    LOG_D(LOG_STD,"motorH: 0x%X",motors_instance.motorH->error);
                    end = TRUE;
                }
                else
                {
                    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                    LOG_E(LOG_STD,"!!! motorH: 0x%X",motors_instance.motorH->error);
                    MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_GENERIC,F_RET_MOTOR_INIT_UNEXPECTED_ERROR);
                }

            }
        }

        bool_t ts2_elasped;
        if(start_finished == FALSE)
            h_time_is_elapsed_ms(&ts2, 300, &ts2_elasped);
        else
            h_time_is_elapsed_ms(&ts2, 100, &ts2_elasped);

        if(ts2_elasped == TRUE)
        {
            h_time_update(&ts2);
            motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH2);
            //LOG_D(LOG_STD,"p %d",pulsesH2);
            if(abs(pulsesH2 - pulsesH1) <= 5)
            {
                motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                LOG_D(LOG_STD,"enrL stop condition");
                end=TRUE;
            }
            pulsesH1 = pulsesH2;
        }

        uint16_t value_iin = adc_inst.instantaneous.iin;
        if(value_iin > ptr->current_stop)
        {
            motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            end=TRUE;
            LOG_D(LOG_STD,"current %d mA",value_iin);
        }
        tx_thread_sleep(1);
    }
    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    return ret;
}






static void scroll_stop(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    motor_drive_sequence(&ptr->sequences.init.end,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
}

