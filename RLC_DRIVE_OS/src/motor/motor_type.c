/*
 * motor_type.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Christophe
 */

#include <_core/c_salloc/c_salloc.h>
#include <rm_motor_extension.h>
#include <motor/motor.h>
#include "motor_type.h"
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "drive"


void motor_itoh_brake_init(void);


return_t motor_init_type(motor_type_t type)
{
    return_t ret = X_RET_OK;

    if(type == MOTOR_TYPE_UNKNOWN || type >= MOTOR_TYPE_COUNT)
        ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);


    if(motors_instance.profil.initialised  == MOTOR_PROFIL_INITIALISED)
        return X_RET_OK;

    switch(type)
    {
        case MOTOR_TYPE_RM_ITOH_BRAKE:
            motor_itoh_brake_init();
            break;

        case MOTOR_TYPE_RM_ALCOM:
            ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
            break;

        default:
            ERROR_SET_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
            break;
    }


    return ret;
}


void motor_itoh_brake_init(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    memset(ptr,0x00,sizeof(motor_profil_t));
    ptr->initialised = MOTOR_PROFIL_INITIALISED;
    c_linked_list_init(&ptr->sequences.error);
    c_linked_list_init(&ptr->sequences.manual.off);
    c_linked_list_init(&ptr->sequences.manual.enrh);
    c_linked_list_init(&ptr->sequences.manual.enrl);
    c_linked_list_init(&ptr->sequences.manual.enrh);
    c_linked_list_init(&ptr->sequences.manual.enrh_stop);
    c_linked_list_init(&ptr->sequences.manual.enrl_stop);
    //=====================================================================
	// configuration et identification du moteur
	//=====================================================================
    ptr->type = MOTOR_TYPE_RM_ITOH_BRAKE;
    ptr->technology = MOTOR_TECH_BLDC;

    ptr->cfg_motorH.motor_technology = MOTOR_TECH_BLDC;
    ptr->cfg_motorH.pulses_counting_reverse = 1;
    ptr->cfg_motorH.speed_reverse = 1;

    ptr->cfg_motorL.motor_technology = MOTOR_TECH_BLDC;
    ptr->cfg_motorL.pulses_counting_reverse = 1;
    ptr->cfg_motorL.speed_reverse = 1;

    motor_phase_t *phase;
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
	phase->condition_timeout_ms = 0;
	phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
	phase->params_motors[0].mode = MOTOR_BRAKE_MODE;
	phase->params_motors[1].mode = MOTOR_BRAKE_MODE;
	c_linked_list_append(&ptr->sequences.error,phase);
    //=====================================================================
    // initialisation des paramètres du mode manuel
    //=====================================================================

    // OFF
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
	phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
	phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
	phase->params_motors[0].non_regulated.settings.percent = 0;

	phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
	phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
	phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
	phase->params_motors[1].non_regulated.settings.percent = 0;
	c_linked_list_append(&ptr->sequences.manual.off,phase);


    // ENRH
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 50;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.enrh,phase);

    // ENRH STOP



    // ENRL
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -50;
    c_linked_list_append(&ptr->sequences.manual.enrl,phase);

    // ENRL STOP
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -20;
    phase->post_tempo_ms = 250;
    c_linked_list_append(&ptr->sequences.manual.enrl_stop,phase);


    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.enrl_stop,phase);

    // DERH
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = -20;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.derh,phase);




}
