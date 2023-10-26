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
#define LOG_MODULE    "manual"


static bool_t manual_mode_running = FALSE;
static bool_t manual_mode_stop_order = FALSE;


bool_t check_stop_request(motor_profil_t *ptr);

bool_t check_stop_request(motor_profil_t *ptr)
{
    sequence_result_t sequence_result;

    if(manual_mode_stop_order == TRUE)
    {
        // Arrêt des moteurs
        motor_drive_sequence(&ptr->sequences.manual.off,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
        // RAZ des flags du mode manuel
        manual_mode_stop_order = FALSE;
        manual_mode_running = FALSE;
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
    manual_mode_stop_order = TRUE;
    LOG_D(LOG_STD,"order to stop manual mode")
}

bool_t manual_mode_is_running(void)
{
   return manual_mode_running;
}


void manual_mode_process(void) {
	motor_profil_t *ptr = &motors_instance.profil;
    uint8_t remote = 0x00;
    uint8_t remote_state=0x00;
    sequence_result_t sequence_result;
    c_linked_list_t *current_list=0x00;
    manual_mode_running = TRUE;

    if(m12_enrh) remote = remote | 0x01;
    if(m12_enrl) remote = remote | 0x02;
    if(m12_derh) remote = remote | 0x04;

    bool_t end = FALSE;
    motor_drive_sequence(&ptr->sequences.manual.off,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    current_list = &ptr->sequences.manual.off;

    do
    {

        // Si une demande d'arrêt du modem manuel est présente
        if(check_stop_request(ptr) == TRUE) return;

        // Récupération de l'état de la télécommande
        remote = 0x00;
        if(m12_enrh) remote = remote | 0x01;
        if(m12_enrl) remote = remote | 0x02;
        if(m12_derh) remote = remote | 0x04;

        // Comparaison avec l'état précédent.
        // Traitement si l'état à changé.
        if(remote != remote_state)
        {
            // Sauvegarde du nouvel état en cours de la télécommande
            remote_state = remote;

            // Arrêt propre (rampes par exemple) du mode en cours si besoin
            if(current_list == &ptr->sequences.manual.enrh)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrh_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            }
            else if(current_list == &ptr->sequences.manual.enrl)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrl_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
            }

            // Traitement de la nouvelle consigne
            if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.enrh;
            }
            else if(m12_enrh == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.enrl;
            }
            else if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.derh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.derh;
            }
            else
            {
                motor_drive_sequence(&ptr->sequences.manual.off,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                current_list = &ptr->sequences.manual.off;
            }
        }

        tx_thread_sleep(1);



    }while(!end);

    manual_mode_running = FALSE;
}

#endif /* APPLICATION_MOTOR_MODES_MANUAL_MODE_C_ */
