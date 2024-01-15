/*
 * drive_sequence.c
 *
 *  Created on: 17 oct. 2023
 *      Author: Ch.Leclercq
 */

#include <_core/c_timespan/c_timespan.h>
#include <_core/c_math/c_math.h>
#include <_hal/h_time/h_time.h>
#include <motor/drive_process/drive_sequence.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "DRIVE"

return_t motor_drive_sequence(c_linked_list_t *list,uint16_t behaviour,sequence_result_t *result)
{
    return_t ret = X_RET_OK;
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(list  != NULL)
    ASSERT(result  != NULL)
#endif

    memset(result,0x00,sizeof(sequence_result_t));

    uint32_t phase_count = 0x0;
    c_linked_list_t *sequence=0x00;
    c_timespan_t ts;

    c_timespan_init(&ts);
    h_time_update(&ts);
    sequence = list;
    c_linked_list_get_count(sequence,&phase_count);
    if(phase_count == 0)
        return ret;

    motor_phase_t *phase = 0x00;
    volatile uint8_t i=0;


    motors_instance.motors[0]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[0]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[0]->status);
    motors_instance.motors[1]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[1]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[1]->status);
    result->errorH = motors_instance.motors[0]->error;
    result->errorL = motors_instance.motors[1]->error;


    if((behaviour & MOTOR_SEQUENCE_CHECK_ERROR_START) != 0x00)
    {
        if((motors_instance.motors[0]->status == MOTOR_120_DEGREE_CTRL_STATUS_ERROR)||
           (motors_instance.motors[1]->status == MOTOR_120_DEGREE_CTRL_STATUS_ERROR))
        {
           result->status = SEQUENCE_FINISH_ERROR;
           ERROR_SET_AND_RETURN(F_RET_MOTOR_SEQUENCE_ERROR_START);
        }
    }



    uint8_t phase_index = 0;
    for(phase_index=0;phase_index<phase_count;phase_index++)
    {
        c_linked_list_get_by_index(sequence,phase_index,(void**)&phase);

        //motor_log_api();
        /*for (i = 0; i < 2; i++)
        {
            LOG_D(LOG_STD,"Motor %d : Current mode %d",i,motors_instance.motors[i]->current_drive_mode);
           //LOG_D(LOG_STD,"Motor %d",i);
           if(phase->params_motors[i].mode == MOTOR_REGULATED_MODE)
           {
              LOG_D(LOG_STD,"Mot%d -> %f rpm",i,phase->params_motors[i].regulated.rpm);
           }
           else if(phase->params_motors[i].mode == MOTOR_NON_REGULATED_MODE)
           {
              LOG_D(LOG_STD,"Mot%d -> %d percent - %d ms",i,phase->params_motors[i].non_regulated.settings.percent,phase->params_motors[i].non_regulated.settings.timeout_hall_ms);
           }
           else if(phase->params_motors[i].mode == MOTOR_BRAKE_MODE)
           {
             LOG_D(LOG_STD,"Mot%d -> mask %d",i,phase->params_motors[i].brake.mask);
           }
        }*/




        for(i=0;i<2;i++)
        {
            motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
            switch(phase->params_motors[i].mode)
            {
                case MOTOR_REGULATED_MODE:
                    switch(motors_instance.motors[i]->status)
                    {
                        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                            if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == FALSE)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
                            if(motors_instance.motors[i]->current_drive_mode != phase->params_motors[i].mode)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                motor_wait_stop(motors_instance.motors[i]);
                                if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == FALSE)
                                {
                                    motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                }
                            }
                            else
                            {
                                if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == TRUE)
                                {
                                    motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                    motor_wait_stop(motors_instance.motors[i]);
                                }
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == FALSE)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->reset(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == FALSE)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;
                    }
                    break;

                case MOTOR_NON_REGULATED_MODE:
                    switch(motors_instance.motors[i]->status)
                    {
                        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                            if(phase->params_motors[i].non_regulated.settings.percent != 0)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
                            if(motors_instance.motors[i]->current_drive_mode != phase->params_motors[i].mode)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                motor_wait_stop(motors_instance.motors[i]);
                                if(phase->params_motors[i].non_regulated.settings.percent != 0)
                                {
                                    motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                }
                            }
                            else
                            {
                                if(phase->params_motors[i].non_regulated.settings.percent == 0)
                                {
                                    motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                                    motor_wait_stop(motors_instance.motors[i]);
                                }
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            if(phase->params_motors[i].non_regulated.settings.percent != 0)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->reset(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            if(phase->params_motors[i].non_regulated.settings.percent != 0)
                            {
                                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            }
                            break;
                    }
                break;

                case MOTOR_BRAKE_MODE:
                    switch(motors_instance.motors[i]->status)
                    {
                        case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:

                            break;

                        case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            motor_wait_stop(motors_instance.motors[i]);
                            motors_instance.motors[i]->motor_ctrl_instance->p_api->reset(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                            break;
                    }
                    break;
            }

        }


        for (i = 0; i < 2; i++)
        {
            switch(phase->params_motors[i].mode)
            {
                case MOTOR_REGULATED_MODE:
                    if (c_math_float_equality(phase->params_motors[i].regulated.rpm,0.0f) == FALSE)
                        motors_instance.motors[i]->motor_ctrl_instance->p_api->speedSet(
                                motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
                                phase->params_motors[i].regulated.rpm);
                    break;

                case MOTOR_NON_REGULATED_MODE:
                    if(phase->params_motors[i].non_regulated.settings.percent != 0)
                    motors_instance.motors[i]->motor_ctrl_instance->p_api->settingsSet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
                            phase->params_motors[i].non_regulated.settings);
                    break;

                case MOTOR_BRAKE_MODE:
                    motors_instance.motors[i]->motor_ctrl_instance->p_api->brake(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,phase->params_motors[i].brake.mask);
                    break;
            }

            motors_instance.motors[i]->current_drive_mode = phase->params_motors[i].mode;
        }



        // Petite temporisation pour ne pas avoir de faux positifs lors des verifications sur la vitesse
        if(phase->next_condition != MOTOR_NEXT_CONDITION_NONE)
        {
           delay_ms(150);
        }
        h_time_update(&ts);


        bool_t end = FALSE;
        do
        {
            result->errorH = motors_instance.motors[0]->error;
            result->errorL = motors_instance.motors[1]->error;

            if((behaviour & MOTOR_SEQUENCE_CHECK_ERROR_RUN) != 0x00)
            {
                if(result->errorH != 0x00 || result->errorL != 0x00)
                {
                    result->status = SEQUENCE_FINISH_ERROR;
                    ERROR_SET_AND_RETURN(F_RET_MOTOR_SEQUENCE_ERROR_RUN);
                }
            }

            // Boucle de sécurité pour ne pas attendre indéfiniemment que la consigne soit atteinte.
            if(phase->condition_timeout_ms != 0x00)
            {
                bool_t end_timeout = FALSE;
                h_time_is_elapsed_ms(&ts, phase->condition_timeout_ms, &end_timeout);
                if(end_timeout == TRUE)
                {
                    h_time_update(&ts);
                    end = TRUE;

                    if((behaviour & MOTOR_SEQUENCE_CHECK_TIMEOUT) != 0x00)
                    {
                        result->status = SEQUENCE_FINISH_ERROR;
                        ERROR_SET_AND_RETURN(F_RET_MOTOR_SEQUENCE_ERROR_TIMEOUT);
                    }
                }
            }

            // Vérification des vitesses
            bool_t speeds_achieved[2]={0};
            for(i=0;i<2;i++)
            {
                // Récupération du status du moteur
                motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
                switch(motors_instance.motors[i]->status)
                {
                case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                    speeds_achieved[i] = TRUE;
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
                    motor_is_speed_achieved(motors_instance.motors[i],&speeds_achieved[i]);
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
                    speeds_achieved[i] = TRUE;
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:

                    break;
                }
            }

            volatile motor_phase_t *pcond = phase;
            switch (pcond->next_condition)
            {
                case MOTOR_NEXT_CONDITION_NONE:
                    end = TRUE;
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDH:
                    if (speeds_achieved[0] == TRUE) {

                        end = TRUE;
                    }
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDL:
                    if (speeds_achieved[1] == TRUE) {
                        end = TRUE;
                    }
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDHL:
                    if (speeds_achieved[0] == TRUE && speeds_achieved[1] == TRUE) {
                        end = TRUE;
                    }
                    break;
            }
            tx_thread_sleep(1);
        }while(!end);

        if(phase->post_tempo_ms != 0x00)
            delay_ms(phase->post_tempo_ms);
    }
    return ret;
}


