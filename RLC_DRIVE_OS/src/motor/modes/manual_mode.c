/*
 * manual_mode.c
 *
 *  Created on: 18 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_MOTOR_MODES_MANUAL_MODE_C_
#define APPLICATION_MOTOR_MODES_MANUAL_MODE_C_
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include "manual_mode.h"
#include <motor/motor.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <motor/drive_process/drive_process.h>
#include <motor/drive_process/drive_sequence.h>
#include <remotectrl/remotectrl.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "MANUAL"


static bool_t mode_running = FALSE;
static bool_t mode_stop_order = FALSE;


static bool_t check_stop_request(void);

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
        //
        LOG_D(LOG_STD,"manual mode stop order applied")
        // Fin
        return TRUE;
    }
    else
        return FALSE;
}


void manual_mode_stop(void)
{
    mode_stop_order = TRUE;
    LOG_D(LOG_STD,"order to stop manual mode")
}

bool_t manual_mode_is_running(void)
{
   return mode_running;
}


void manual_mode_process(void) {

	motor_profil_t *ptr = &motors_instance.profil;
    uint8_t remote = 0x00;
    uint8_t remote_state=0x00;
    sequence_result_t sequence_result;
    c_linked_list_t *current_list=0x00;
    c_timespan_t ts;
    bool_t ts_elasped;
    h_time_update(&ts);
    mode_running = TRUE;

    if(m12_enrh) remote = remote | 0x01;
    if(m12_enrl) remote = remote | 0x02;
    if(m12_derh) remote = remote | 0x04;

    bool_t end = FALSE;
    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    current_list = &ptr->sequences.off_no_brake;

    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);

    do
    {


        /*h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);

            int32_t pulsesH;
            int32_t pulsesL;
            motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
            motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,&pulsesL);

            LOG_D(LOG_STD,"pulsesH: %d  dirH: %d  pulsesL: %d dirL: %d",pulsesH,motors_instance.motorH->hall_vars->real_direction,pulsesL,motors_instance.motorL->hall_vars->real_direction);
        }
*/

        // Si une demande d'arrêt du modem manuel est présente
        if(check_stop_request() == TRUE) return;

        // Récupération de l'état de la télécommande
        remote = 0x00;
        if(m12_enrh) remote = remote | 0x01;
        if(m12_enrl) remote = remote | 0x02;
        if(m12_derh) remote = remote | 0x04;

        // Comparaison avec l'état précédent.
        // Traitement si l'état à changé.
        if(remote != remote_state)
        {
            LOG_D(LOG_STD,"Remote: %d - previous %d",remote,remote_state);
            // Sauvegarde du nouvel état en cours de la télécommande
            remote_state = remote;

            // Arrêt propre (rampes par exemple) du mode en cours si besoin
            if(current_list == &ptr->sequences.manual.enrh)
            {
                LOG_D(LOG_STD,"enrh_stop");
                motor_drive_sequence(&ptr->sequences.manual.enrh_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            }
            else if(current_list == &ptr->sequences.manual.enrl)
            {
                LOG_D(LOG_STD,"enrl_stop");
                motor_drive_sequence(&ptr->sequences.manual.enrl_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            }

            // Traitement de la nouvelle consigne
            if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                LOG_D(LOG_STD,"enrh");
                motor_drive_sequence(&ptr->sequences.manual.enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.enrh;
            }
            else if(m12_enrh == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                LOG_D(LOG_STD,"enrl");
                motor_drive_sequence(&ptr->sequences.manual.enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.enrl;
            }
            else if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == REMOTECTRL_ACTIVE_LEVEL)
            {
                LOG_D(LOG_STD,"derh");
                motor_drive_sequence(&ptr->sequences.manual.derh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.derh;
            }
            else
            {
                LOG_D(LOG_STD,"off_no_brake");
                motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.off_no_brake;
            }
        }




        /*if(current_list == &ptr->sequences.manual.enrh)
        {
            motor_drive_sequence(&ptr->sequences.manual.enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
        }*/

        tx_thread_sleep(1);


    }while(!end);

    mode_running = FALSE;
}

#endif /* APPLICATION_MOTOR_MODES_MANUAL_MODE_C_ */
