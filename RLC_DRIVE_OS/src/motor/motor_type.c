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
#define LOG_MODULE    "DRIVE"


void motor_itoh_brake_init(void);


return_t motor_init_type(motor_type_t type)
{
    return_t ret = X_RET_OK;

    if(type == MOTOR_TYPE_UNKNOWN || type >= MOTOR_TYPE_COUNT)
        ERROR_LOG_AND_RETURN(F_RET_MOTOR_BAD_TYPE);


    if(motors_instance.profil.initialised  == MOTOR_PROFIL_INITIALISED)
        return X_RET_OK;

    switch(type)
    {
        case MOTOR_TYPE_RM_ITOH_BRAKE:
            motor_itoh_brake_init();
            break;

        case MOTOR_TYPE_RM_ALCOM:
            ERROR_LOG_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
            break;

        default:
            ERROR_LOG_AND_RETURN(F_RET_MOTOR_BAD_TYPE);
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
    c_linked_list_init(&ptr->sequences.off_no_brake);
    c_linked_list_init(&ptr->sequences.off_brake);

    //=====================================================================
	// configuration et identification du moteur
	//=====================================================================
    ptr->type = MOTOR_TYPE_RM_ITOH_BRAKE;
    ptr->technology = MOTOR_TECH_BLDC;

    ptr->cfg_motorH.motor_technology = MOTOR_TECH_BLDC;
    ptr->cfg_motorH.pulses_counting_reverse = 0;
    ptr->cfg_motorH.speed_reverse = 1;

    ptr->cfg_motorL.motor_technology = MOTOR_TECH_BLDC;
    ptr->cfg_motorL.pulses_counting_reverse = 0;
    ptr->cfg_motorL.speed_reverse = 1;

    ptr->poster_showtime = 3500;
    ptr->current_stop = 3000;
    //=====================================================================
    // configuration de la taille des bandes mères et de la valeur moyenne
    // d'une affiche (en nombre de points codeurs)
    //=====================================================================
    ptr->sizes.prime_band_upper_size = 1870;//1890;
    ptr->sizes.prime_band_lower_size = 230;//225;
    ptr->sizes.poster_size = 1150;

    //=====================================================================
    // Positions par défaut des affiches (en points codeur H)
    //=====================================================================
    ptr->panels.positions_default[0] = ptr->sizes.prime_band_upper_size-20;
    ptr->panels.positions_default[1] = 3200-20;
    ptr->panels.positions_default[2] = 4450-20;
    ptr->panels.positions_default[3] = 5640-20;
    ptr->panels.positions_default[4] = 6770-20;


    //=====================================================================
    // configuration des phases génériques
    //=====================================================================
    motor_phase_t *phase_off_no_brake;
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase_off_no_brake);
    memset(phase_off_no_brake,0x00,sizeof(motor_phase_t));
    phase_off_no_brake->condition_timeout_ms = 0;
    phase_off_no_brake->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase_off_no_brake->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase_off_no_brake->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase_off_no_brake->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase_off_no_brake->params_motors[0].non_regulated.settings.percent = 0;

    phase_off_no_brake->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase_off_no_brake->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase_off_no_brake->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase_off_no_brake->params_motors[1].non_regulated.settings.percent = 0;

    motor_phase_t *phase_brake_full;
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase_brake_full);
    memset(phase_brake_full,0x00,sizeof(motor_phase_t));
    phase_brake_full->condition_timeout_ms = 0;
    phase_brake_full->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase_brake_full->params_motors[0].mode = MOTOR_BRAKE_MODE;
    phase_brake_full->params_motors[0].brake.mask = 0x0;
    phase_brake_full->params_motors[1].mode = MOTOR_BRAKE_MODE;
    phase_brake_full->params_motors[1].brake.mask = 0x0;


    motor_phase_t *phase;
    //=====================================================================
    // configuration des phases
    //=====================================================================
	c_linked_list_append(&ptr->sequences.error,phase_brake_full);

	// OFF NO BRAKE
    c_linked_list_append(&ptr->sequences.off_no_brake,phase_off_no_brake);

    // OFF BRAKE
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[0].brake.mask = 0x80;
    phase->params_motors[1].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[1].brake.mask = 0x80;
    c_linked_list_append(&ptr->sequences.off_brake,phase);

    //=====================================================================
    // initialisation des paramètres du mode manuel
    //=====================================================================
    c_linked_list_init(&ptr->sequences.manual.enrh);
    c_linked_list_init(&ptr->sequences.manual.enrl);
    c_linked_list_init(&ptr->sequences.manual.enrh);
    c_linked_list_init(&ptr->sequences.manual.enrh_stop);
    c_linked_list_init(&ptr->sequences.manual.enrl_stop);

    // ENRH
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.slope = 1;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 60;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.enrh,phase);

    // ENRH STOP
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->post_tempo_ms = 250;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.manual.enrh_stop,phase);


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
    phase->params_motors[1].non_regulated.settings.slope = 1;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -60;
    c_linked_list_append(&ptr->sequences.manual.enrl,phase);

    // ENRL STOP
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->post_tempo_ms = 250;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[0].brake.mask = 0x08;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.slope = 1;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -40;
    c_linked_list_append(&ptr->sequences.manual.enrl_stop,phase);

    c_linked_list_append(&ptr->sequences.manual.enrl_stop,phase_off_no_brake);

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


    //=====================================================================
    // initialisation des paramètres du mode INIT
    //=====================================================================
    c_linked_list_init(&ptr->sequences.init.stretch1);
    c_linked_list_init(&ptr->sequences.init.stretch2);
    c_linked_list_init(&ptr->sequences.init.enrl_start);
    c_linked_list_init(&ptr->sequences.init.enrl_finish);
    c_linked_list_init(&ptr->sequences.init.enrl);
    c_linked_list_init(&ptr->sequences.init.end);





    // STRETCH
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->post_tempo_ms = 0;//500;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = -15;
    phase->params_motors[0].non_regulated.settings.slope=1;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 15;
    phase->params_motors[1].non_regulated.settings.slope=1;
    c_linked_list_append(&ptr->sequences.init.stretch1,phase);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->post_tempo_ms = 0;//1500;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[0].non_regulated.settings.percent = 15;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[1].non_regulated.settings.percent = -15;
    c_linked_list_append(&ptr->sequences.init.stretch2,phase);


    // ENRL START
    c_linked_list_append(&ptr->sequences.init.enrl_start,phase_off_no_brake);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[0].non_regulated.settings.percent = 0;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[1].non_regulated.settings.percent = -30;
    phase->params_motors[1].non_regulated.settings.slope=1;
    c_linked_list_append(&ptr->sequences.init.enrl_start,phase);


    // ENRL FINISH
    c_linked_list_append(&ptr->sequences.init.enrl_finish,phase_off_no_brake);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 15;
    phase->params_motors[0].non_regulated.settings.slope=1;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -30;//-20;
    phase->params_motors[1].non_regulated.settings.slope=0;
    c_linked_list_append(&ptr->sequences.init.enrl_finish,phase);

    /*C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 10;//15;
    phase->params_motors[0].non_regulated.settings.slope=1;
    phase->params_motors[1].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[1].brake.mask=0;
    c_linked_list_append(&ptr->sequences.init.enrl_finish,phase);*/


    // ENRL
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[0].non_regulated.settings.percent = 0;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[1].non_regulated.settings.percent = -55;//-55;//-45;
    phase->params_motors[1].non_regulated.settings.slope=1;
    c_linked_list_append(&ptr->sequences.init.enrl,phase);

    // END
    c_linked_list_append(&ptr->sequences.init.end,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 5;
    phase->params_motors[1].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[1].brake.mask = 0x0;
    c_linked_list_append(&ptr->sequences.init.end,phase);



    //=====================================================================
    // initialisation des paramètres du mode AUTO
    //=====================================================================
    c_linked_list_init(&ptr->sequences.automatic.poster_enrh);
    c_linked_list_init(&ptr->sequences.automatic.poster_enrh_slow);
    c_linked_list_init(&ptr->sequences.automatic.poster_enrl);
    c_linked_list_init(&ptr->sequences.automatic.poster_stop);
    c_linked_list_init(&ptr->sequences.automatic.lower_band_enrh);
    c_linked_list_init(&ptr->sequences.automatic.lower_band_enrl);
    c_linked_list_init(&ptr->sequences.automatic.poster_init_enrh_slow);
    c_linked_list_init(&ptr->sequences.automatic.poster_init_enrh);


    // POSTER ENRH
    c_linked_list_append(&ptr->sequences.automatic.poster_enrh,phase_off_no_brake);


    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_REGULATED_MODE;
    phase->params_motors[0].regulated.rpm = 1700.0f;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.automatic.poster_enrh,phase);


    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_REGULATED_MODE;
    phase->params_motors[0].regulated.rpm = 800.0f;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.automatic.poster_enrh_slow,phase);




    // POSTER ENRL
    c_linked_list_append(&ptr->sequences.automatic.poster_enrl,phase_off_no_brake);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->post_tempo_ms = 150;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;
    phase->params_motors[1].mode = MOTOR_REGULATED_MODE;
    phase->params_motors[1].regulated.rpm = -1700.0f;
    c_linked_list_append(&ptr->sequences.automatic.poster_enrl,phase);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;
    phase->params_motors[1].mode = MOTOR_REGULATED_MODE;
    phase->params_motors[1].regulated.rpm = -1700.0f;
    c_linked_list_append(&ptr->sequences.automatic.poster_enrl,phase);



    // INIT ENRH SLOW
    c_linked_list_append(&ptr->sequences.automatic.poster_init_enrh_slow,phase_off_no_brake);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 300;
    phase->params_motors[0].non_regulated.settings.percent = 50;
    phase->params_motors[1].non_regulated.settings.slope=1;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.automatic.poster_init_enrh_slow,phase);

    // INIT ENRH
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 300;
    phase->params_motors[0].non_regulated.settings.percent = 70;
    phase->params_motors[1].non_regulated.settings.slope=1;

    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.automatic.poster_init_enrh,phase);



    // POSTER STOP
    c_linked_list_append(&ptr->sequences.automatic.poster_stop,phase_off_no_brake);

    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 6;
    phase->params_motors[1].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[1].brake.mask = 0x0;
    c_linked_list_append(&ptr->sequences.automatic.poster_stop,phase);


    // LOWERBAND ENRH
    c_linked_list_append(&ptr->sequences.automatic.lower_band_enrh,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 15;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    c_linked_list_append(&ptr->sequences.automatic.lower_band_enrh,phase);



    // LOWERBAND ENRL
    c_linked_list_append(&ptr->sequences.automatic.lower_band_enrl,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->params_motors[0].mode = MOTOR_BRAKE_MODE;
    phase->params_motors[0].brake.mask = 0x04;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 200;
    phase->params_motors[1].non_regulated.settings.percent = -30;//-30;
    phase->params_motors[1].non_regulated.settings.slope=0;
    c_linked_list_append(&ptr->sequences.automatic.lower_band_enrl,phase);


    //=====================================================================
    // initialisation des paramètres du mode ERREUR
    //=====================================================================
    c_linked_list_init(&ptr->sequences.error_check.test1H);
    c_linked_list_init(&ptr->sequences.error_check.test1L);
    c_linked_list_init(&ptr->sequences.error_check.test2);


    // TEST1H
    c_linked_list_append(&ptr->sequences.error_check.test1H,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->post_tempo_ms = 0;//500;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 250;
    phase->params_motors[0].non_regulated.settings.percent = -25;
    phase->params_motors[0].non_regulated.settings.slope=0;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 0;
    phase->params_motors[1].non_regulated.settings.slope=0;
    c_linked_list_append(&ptr->sequences.error_check.test1H,phase);

    // TEST1L
    c_linked_list_append(&ptr->sequences.error_check.test1L,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->post_tempo_ms = 0;//500;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 0;
    phase->params_motors[0].non_regulated.settings.slope=0;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = 25;
    phase->params_motors[1].non_regulated.settings.slope=0;
    c_linked_list_append(&ptr->sequences.error_check.test1L,phase);

    // TEST2
    c_linked_list_append(&ptr->sequences.error_check.test2,phase_off_no_brake);
    C_SALLOC(sizeof(motor_phase_t),(void**)&phase);
    memset(phase,0x00,sizeof(motor_phase_t));
    phase->condition_timeout_ms = 0;
    phase->next_condition = MOTOR_NEXT_CONDITION_NONE;
    phase->post_tempo_ms = 0;
    phase->params_motors[0].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[0].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[0].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[0].non_regulated.settings.percent = 15;
    phase->params_motors[0].non_regulated.settings.slope=1;
    phase->params_motors[1].mode = MOTOR_NON_REGULATED_MODE;
    phase->params_motors[1].non_regulated.settings.current_max = 0.0f;
    phase->params_motors[1].non_regulated.settings.timeout_hall_ms = 0;
    phase->params_motors[1].non_regulated.settings.percent = -15;
    phase->params_motors[1].non_regulated.settings.slope=1;
    c_linked_list_append(&ptr->sequences.error_check.test2,phase);



}
