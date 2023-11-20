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
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "INIT"


static bool_t mode_running = FALSE;
static bool_t mode_stop_order = FALSE;


static bool_t check_stop_request(void);
static return_t init_strectch(void);
static return_t init_enrh(void);
static return_t init_enrl(void);
static return_t init_poster(void);
static uint16_t line = 0;

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
        LOG_D(LOG_STD,"manual mode stop order begin ");
        // Arrêt des moteurs
        motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
        // RAZ des flags du mode manuel
        mode_stop_order = FALSE;
        mode_running = FALSE;
        //
        LOG_D(LOG_STD,"manual mode stop order end");
        // Fin
        return TRUE;
    }
    else
        return FALSE;
}

void init_mode_stop(void)
{
    mode_stop_order = TRUE;
    LOG_D(LOG_STD,"order to stop init mode %d", line);
}

bool_t init_mode_is_running(void)
{
   return mode_running;
}

return_t init_mode_process(void) {
    return_t ret = X_RET_OK;
	motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);
    mode_running = TRUE;
    mode_stop_order = FALSE;
    motors_instance.error = MOTORS_ERROR_NONE;
    LOG_I(LOG_STD,"INIT -> début");
    //----------------------------------------------------------------------------------------------
    // Arrêt des moteurs
    //----------------------------------------------------------------------------------------------
    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    end = FALSE;
    while(!end)
    {
        if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
        h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);
            end = TRUE;
        }
        tx_thread_sleep(1);
    }
    LOG_I(LOG_STD,"Fin arrêt");

    //----------------------------------------------------------------------------------------------
    // Mise en tension des affiches
    //----------------------------------------------------------------------------------------------
    ret = init_strectch();
    if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
    //----------------------------------------------------------------------------------------------
    // Recherche bande mère basse
    //----------------------------------------------------------------------------------------------
    ret = init_enrh();
    if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
    delay_ms(500);
    //----------------------------------------------------------------------------------------------
    // Recherche bande mère haute
    //----------------------------------------------------------------------------------------------
    ret = init_enrl();
    if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
    //----------------------------------------------------------------------------------------------
    // Positionnement sur 1ere affiche
    //----------------------------------------------------------------------------------------------
    LOG_I(LOG_STD,"init_poster");
    ret = init_poster();
    LOG_I(LOG_STD,"fin_init_poster");
    if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
    //----------------------------------------------------------------------------------------------
    // Fin
    //----------------------------------------------------------------------------------------------


    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    LOG_I(LOG_STD,"INIT -> fin");

    while(1)
    {
        if(check_stop_request()) return F_RET_MOTOR_INIT_CANCELLED;
        tx_thread_sleep(1);
    }


    mode_running = FALSE;
    mode_stop_order = FALSE;


 return ret;
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
        if(mode_stop_order == TRUE) return F_RET_MOTOR_INIT_CANCELLED;

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
                if(motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW)
                   LOG_D(LOG_STD,"motorH reverse detected");
                if(motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
                   LOG_D(LOG_STD,"motorL reverse detected");

                LOG_D(LOG_STD,"motorH: 0x%X ,motorL: 0x%X",motors_instance.motorH->error,motors_instance.motorL->error);
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

static return_t init_enrh(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);


    //motor_phase_t *phase = 0x00;
    //c_linked_list_get_by_index(&motors_instance.profil.sequences.init.enrh,1,(void**)&phase);
    motor_drive_sequence(&ptr->sequences.init.enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    while(!end)
    {
        if(mode_stop_order == TRUE) return F_RET_MOTOR_INIT_CANCELLED;

        h_time_is_elapsed_ms(&ts, 60000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            // Arrêt des moteurs
            motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            // Macro d'enregistrement d'erreur et code de retour
            MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_TIMEOUT_SEARCHING_BASE_L,F_RET_MOTOR_INIT_TIMEOUT_BASE_L);
        }


        if((motors_instance.motorH->error != 0x00) )
        {
            if(motors_instance.motorH->error != 0x00)
            {
                if((motors_instance.motorH->error == MOTOR_ERROR_OVER_CURRENT_SW) ||
                   (motors_instance.motorH->error == MOTOR_ERROR_BEMF_TIMEOUT))

                {
                    LOG_D(LOG_STD,"motorH: 0x%X",motors_instance.motorH->error);
                    end = TRUE;
                }
                else
                {
                    LOG_E(LOG_STD,"motorH: 0x%X",motors_instance.motorH->error);
                    MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_GENERIC,F_RET_MOTOR_INIT_UNEXPECTED_ERROR);
                }

            }
        }
        tx_thread_sleep(1);
    }

    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

    return ret;
}

static return_t init_enrl(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);

    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);


    motor_drive_sequence(&ptr->sequences.init.enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    while(!end)
    {
        if(mode_stop_order == TRUE) return F_RET_MOTOR_INIT_CANCELLED;

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
                    LOG_D(LOG_STD,"motorH: 0x%X",motors_instance.motorH->error);
                    end = TRUE;
                }
                else
                {
                    LOG_E(LOG_STD,"motorH: 0x%X",motors_instance.motorH->error);
                    MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_GENERIC,F_RET_MOTOR_INIT_UNEXPECTED_ERROR);
                }

            }
        }
        tx_thread_sleep(1);
    }

    int32_t pulsesH;
    int32_t pulsesL;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,&pulsesL);
    LOG_D(LOG_STD,"pulsesH: %d   pulsesL: %d",pulsesH,pulsesL);

    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

    return ret;
}



static return_t init_poster(void)
{
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    c_timespan_t ts2;
    bool_t end = FALSE;
    bool_t ts_elasped;
    h_time_update(&ts);

    int32_t pulsesH;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);
    motor_drive_sequence(&ptr->sequences.init.posterAccelerate,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    h_time_get_elapsed(&ts, &ts2);
    volatile motor_120_control_instance_t *mot_inst = (motor_120_control_instance_t*)motors_instance.motorH->motor_hall_instance;
        volatile motor_120_control_hall_instance_ctrl_t * p_instance_ctrl = (motor_120_control_hall_instance_ctrl_t *) (mot_inst->p_ctrl);
    LOG_D(LOG_STD,"elaspedH: %d  %d",ts2.ms,(int32_t)p_instance_ctrl->f4_v_ref);

    float speed = p_instance_ctrl->f4_v_ref;
    LOG_D(LOG_STD,"Speed %d mV",(int32_t)(speed*1000));
    //motors_instance.motorH->motor_ctrl_instance->
    while(!end)
    {
        if(mode_stop_order == TRUE) return F_RET_MOTOR_INIT_CANCELLED;
        h_time_is_elapsed_ms(&ts, 5000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);
            motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_TIMEOUT_CHANGING_POSTER,F_RET_MOTOR_INIT_TIMEOUT_POSTER);
        }
        motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
        if(pulsesH >= 1865)
        {
            end = TRUE;
            LOG_D(LOG_STD,"Top1");
        }

        tx_thread_sleep(1);
    }
    h_time_update(&ts);
    end=FALSE;
    /*motor_drive_sequence(&ptr->sequences.init.posterDecelerate,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    h_time_get_elapsed(&ts, &ts2);
    LOG_D(LOG_STD,"elaspedL: %d  %d",ts2.ms,(int32_t)p_instance_ctrl->f4_v_ref);*/




    /*while(!end)
    {
        if(mode_stop_order == TRUE) return F_RET_MOTOR_INIT_CANCELLED;
        h_time_is_elapsed_ms(&ts, 5000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);
            motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_TIMEOUT_CHANGING_POSTER,F_RET_MOTOR_INIT_TIMEOUT_POSTER);
        }
        motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
        if(pulsesH >= 1865)
        {
            LOG_D(LOG_STD,"Top2");
            end = TRUE;
        }

        tx_thread_sleep(1);
    }*/
    h_time_update(&ts);
    motor_drive_sequence(&ptr->sequences.init.posterStop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

    return ret;
}
