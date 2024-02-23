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
#include <motor/check/motor_check.h>
#include <remotectrl/remotectrl.h>
#include <adc/adc.h>
#include <leds/leds.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "MANUAL"






void manual_mode_process(void) {
    drive_control.running = TRUE;
	motor_profil_t *ptr = &motors_instance.profil;
    uint8_t remote = 0x00;
    uint8_t remote_state=0x00;
    sequence_result_t sequence_result;
    c_linked_list_t *current_list=0x00;
    c_timespan_t ts;
    bool_t ts_elasped;
    h_time_update(&ts);


    if(m12_enrh) remote = remote | 0x01;
    if(m12_enrl) remote = remote | 0x02;
    if(m12_derh) remote = remote | 0x04;

    // RAZ de la LED dédiée aux erreurs moteur
    led_error_motor_off();

    // Routine pour vérifier la présence de défaut éléctrique sur la partie moteur.
    // Si une erreur éléctrique est présente alors le mode manuel est inactif.
    return_t ret = motor_check(FALSE);
    while(ret != X_RET_OK)
    {

        if(drive_stop_request() == TRUE) return;
        h_time_is_elapsed_ms(&ts, 3000, &ts_elasped);
        if(ts_elasped)
        {
            h_time_update(&ts);
            ret = motor_check(FALSE);
        }
        tx_thread_sleep(1);
    }


    bool_t end = FALSE;
    motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    current_list = &ptr->sequences.off_no_brake;

    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);

    do
    {


        h_time_is_elapsed_ms(&ts, 500, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            h_time_update(&ts);


            /*float ic = (-adc_inst.motorH.iu_ad-adc_inst.motorH.iw_ad);
            float diffH = (adc_inst.motorH.iu_ad-ic);
            diffH = diffH * 0.577350269f;

            ic = (-adc_inst.motorL.iu_ad-adc_inst.motorL.iw_ad);
            float diffL = (adc_inst.motorL.iu_ad-ic);
            diffL = diffL * 0.577350269f;

            LOG_D(LOG_STD,"%f %f - IIN:%d",diffH,diffL,adc_inst.average.iin);*/

        }


        // Si une demande d'arrêt du modem manuel est présente
        if(drive_stop_request() == TRUE) return;

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


}

#endif /* APPLICATION_MOTOR_MODES_MANUAL_MODE_C_ */
