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
#include <leds/leds.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "ERROR"

static return_motor_cplx_t error_test_1H(void);
static return_motor_cplx_t error_test_1L(void);
static return_motor_cplx_t error_test_2(void);
static return_motor_cplx_t error_test_3(void);
static return_t error_loop(void);
static void error_log_results(st_system_motor_t *ptr_system_motor);
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
        h_time_is_elapsed_ms(&ts, 500, &ts_elasped);
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
    LOG_D(LOG_STD,"Counter %d",pulses);

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
        h_time_is_elapsed_ms(&ts, 500, &ts_elasped);
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
    LOG_D(LOG_STD,"Counter %d",pulses);
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
    c_timespan_t ts_finished[2];
    volatile uint8_t index = 0;
    const uint8_t stat_process = 1;
    const uint8_t stat_finished = 2;
    uint32_t error_fsp_counter[2] = {0,0};
    uint16_t error_fsp[2] = {0,0};
    uint8_t status[2];
    int32_t pulses1[2];
    int32_t pulses2[2];
    uint8_t no_pulses_count[2];
    uint8_t reverse_count[2];
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
    reverse_count[0] = 0;
    reverse_count[1] = 0;
    no_pulses_count[0] = 0;
    no_pulses_count[1] = 0;



    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
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
                       h_time_update(&ts_finished[index]);
                       //LOG_E(LOG_STD,"Error FSP 0x%02x",mot->error);
                    }
                    else
                    {
                       LOG_W(LOG_STD,"Error index %d -> FSP 0x%02x",index,mot->error);
                       mot->error = 0;
                       motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                       delay_ms(200);
                       h_time_update(&ts_pulses[0]);
                       h_time_update(&ts_pulses[1]);
                       motor_drive_sequence(&ptr->sequences.error_check.test2,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
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
                        no_pulses_count[index]++;
                        if(no_pulses_count[index] >= 3)
                        {
                            LOG_D(LOG_STD,"Motor %d no more pulses",index);
                            status[index] = stat_finished;
                            h_time_update(&ts_finished[index]);
                        }
                    }
                    pulses1[index] = pulses2[index];
                }
            }
        }

        if(status[0] != stat_finished && motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW)
        {
            if(reverse_count[0] >= 5)
            {
               LOG_D(LOG_STD,"MotorH reverse detected");
               status[0] = stat_finished;
               h_time_update(&ts_finished[0]);
            }
            else
                reverse_count[0]++;
        }

        if(status[1] != stat_finished && motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
        {
            if(reverse_count[1] >= 5)
            {
               LOG_D(LOG_STD,"MotorL reverse detected");
               status[1] = stat_finished;
               h_time_update(&ts_finished[1]);
            }
            else
                reverse_count[1]++;
        }

        bool_t ts_elasped;
        h_time_is_elapsed_ms(&ts, 30000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            scroll_stop();
            LOG_E(LOG_STD,"Panel damaged");
            return_motor_cplx_update(&ret,F_RET_PANELS_DAMAGED);
            return ret;
        }

        if((status[0] == stat_finished && status[1] == stat_finished))
        {
            scroll_stop();
            if(error_fsp[0] != 0x00 || error_fsp[1]!= 0x00)
            {
                if(error_fsp[0] != 0x00){
                    LOG_E(LOG_STD,"Error FSP motorH");}
                if(error_fsp[1] != 0x00){
                    LOG_E(LOG_STD,"Error FSP motorL");}
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
        tx_thread_sleep(10);
    }
    return_motor_cplx_update(&ret,F_RET_OK);
}


static return_motor_cplx_t error_test_3(void)
{
    return_motor_cplx_t ret;
    return_motor_cplx_update(&ret,X_RET_OK);

    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    c_timespan_t ts;
    c_timespan_t ts_pulses[2];
    c_timespan_t ts_finished[2];
    volatile uint8_t index = 0;
    const uint8_t stat_process = 1;
    const uint8_t stat_finished = 2;
    uint32_t error_fsp_counter[2] = {0,0};
    uint16_t error_fsp[2] = {0,0};
    uint8_t status[2];
    int32_t pulses1[2];
    int32_t pulses2[2];
    uint8_t no_pulses_count[2];
    uint8_t reverse_count[2];
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
    reverse_count[0] = 0;
    reverse_count[1] = 0;
    no_pulses_count[0] = 0;
    no_pulses_count[1] = 0;



    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
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
                       h_time_update(&ts_finished[index]);
                       //LOG_E(LOG_STD,"Error FSP 0x%02x",mot->error);
                    }
                    else
                    {
                       LOG_W(LOG_STD,"Error index %d -> FSP 0x%02x",index,mot->error);
                       mot->error = 0;
                       motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
                       delay_ms(200);
                       h_time_update(&ts_pulses[0]);
                       h_time_update(&ts_pulses[1]);
                       motor_drive_sequence(&ptr->sequences.error_check.test2,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
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
                        no_pulses_count[index]++;
                        if(no_pulses_count[index] >= 3)
                        {
                            LOG_D(LOG_STD,"Motor %d no more pulses",index);
                            status[index] = stat_finished;
                            h_time_update(&ts_finished[index]);
                        }
                    }
                    pulses1[index] = pulses2[index];
                }
            }
        }

        if(status[0] != stat_finished && motors_instance.motorH->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CCW)
        {
            if(reverse_count[0] >= 5)
            {
               LOG_D(LOG_STD,"MotorH reverse detected");
               status[0] = stat_finished;
               h_time_update(&ts_finished[0]);
            }
            else
                reverse_count[0]++;
        }

        if(status[1] != stat_finished && motors_instance.motorL->hall_vars->real_direction == MOTOR_120_CONTROL_ROTATION_DIRECTION_CW)
        {
            if(reverse_count[1] >= 5)
            {
               LOG_D(LOG_STD,"MotorL reverse detected");
               status[1] = stat_finished;
               h_time_update(&ts_finished[1]);
            }
            else
                reverse_count[1]++;
        }

        bool_t ts_elasped;
        h_time_is_elapsed_ms(&ts, 5000, &ts_elasped);
        if(ts_elasped == TRUE)
        {
            scroll_stop();

            if(status[0] != stat_finished && status[1] != stat_finished)
            {
                return_motor_cplx_update(&ret,F_RET_OK);
                return ret;
            }
            else if(status[0] == stat_finished && status[1] != stat_finished)
            {
                return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVING_H);
                return ret;
            }
            else if(status[0] != stat_finished && status[1] == stat_finished)
            {
                return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVING_L);
                return ret;
            }
            else if(status[0] == stat_finished && status[1] == stat_finished)
            {
                return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVING_HL);
                return ret;
            }

            return_motor_cplx_update(&ret,F_RET_OK);
            return ret;
        }

        if((status[0] == stat_finished && status[1] == stat_finished))
        {
            scroll_stop();
            if(error_fsp[0] != 0x00 || error_fsp[1]!= 0x00)
            {
                if(error_fsp[0] != 0x00){
                    LOG_E(LOG_STD,"Error FSP motorH");}
                if(error_fsp[1] != 0x00){
                    LOG_E(LOG_STD,"Error FSP motorL");}
                ret.code = F_RET_MOTOR_ERROR_API_FSP;
                ret.fsp_motorH_error_code = error_fsp[0];
                ret.fsp_motorL_error_code = error_fsp[1];
                return ret;
            }
        }
        tx_thread_sleep(10);
    }
    return_motor_cplx_update(&ret,F_RET_OK);
}

static void error_log_results(st_system_motor_t *ptr_system_motor)
{
    if(ptr_system_motor->error_lvl1.value != 0x00 ||
       ptr_system_motor->error_lvl2.value != 0x00 ||
       ptr_system_motor->error_lvl3.value != 0x00)
    {
        LOG_E(LOG_STD,"Error detected");

        if(ptr_system_motor->error_lvl1.bits.overcurrent_vm != 0x00){
            LOG_E(LOG_STD,"Lvl1 overcurrent_vm");}
        if(ptr_system_motor->error_lvl1.bits.vcc_hall_h != 0x00){
            LOG_E(LOG_STD,"Lvl1 vcc_hall_h");}
        if(ptr_system_motor->error_lvl1.bits.vcc_hall_l != 0x00){
            LOG_E(LOG_STD,"Lvl1 vcc_hall_l");}
        if(ptr_system_motor->error_lvl1.bits.fsp_h != 0x00){
            LOG_E(LOG_STD,"Lvl1 fsp_h");}
        if(ptr_system_motor->error_lvl1.bits.fsp_l != 0x00){
            LOG_E(LOG_STD,"Lvl1 fsp_l");}
        if(ptr_system_motor->error_lvl1.bits.config_driver_h != 0x00){
            LOG_E(LOG_STD,"Lvl1 config_driver_h");}
        if(ptr_system_motor->error_lvl1.bits.config_driver_l != 0x00){
            LOG_E(LOG_STD,"Lvl1 config_driver_l");}
        if(ptr_system_motor->error_lvl2.bits.error_pattern_h != 0x00){
            LOG_E(LOG_STD,"Lvl2 error_pattern_h");}
        if(ptr_system_motor->error_lvl2.bits.error_pattern_l != 0x00){
            LOG_E(LOG_STD,"Lvl2 error_pattern_l");}
        if(ptr_system_motor->error_lvl2.bits.timeout_pulses_h != 0x00){
            LOG_E(LOG_STD,"Lvl2 timeout_pulses_h");}
        if(ptr_system_motor->error_lvl2.bits.timeout_pulses_l != 0x00){
            LOG_E(LOG_STD,"Lvl2 timeout_pulses_l");}
        if(ptr_system_motor->error_lvl2.bits.unknown != 0x00){
            LOG_E(LOG_STD,"Lvl2 unknown");}
        if(ptr_system_motor->error_lvl3.bits.damaged_panels != 0x00){
             LOG_E(LOG_STD,"Lvl3 damaged_panels");}
        if(ptr_system_motor->error_lvl3.bits.motor_driving_h != 0x00){
             LOG_E(LOG_STD,"Lvl3 motor_driving_h");}
        if(ptr_system_motor->error_lvl3.bits.motor_driving_l != 0x00){
             LOG_E(LOG_STD,"Lvl3 motor_driving_l");}
    }
}


static return_t error_loop(void)
{
    return_t ret = X_RET_OK;
    bool_t end=FALSE;
    do
    {
        led_error_motor_on();
        CHECK_STOP_REQUEST_NESTED();
        tx_thread_sleep(1);
    }while(!end);
    return ret;
}



return_t error_mode_process(void)
{
    drive_control.running = TRUE;
    return_t ret = X_RET_OK;

    // Verification pour savoir si le mode erreur demarre avec une erreur déjà présente
    // Dans ce cas seul le mode manuel peut acquitter le défaut et il n'est pas souhaitable de relancer
    // la procédure de vérification
    // On passe directement dans la boucle erreur
    if(system_check_error() == TRUE)
    {
        LOG_E(LOG_STD,"System already in error");
        error_loop();
        CHECK_STOP_REQUEST();
    }


    st_system_motor_t sys_mot;
    memset(&sys_mot,0x00,sizeof(sys_mot));

    LOG_I(LOG_STD,"Checking motors level1...");
    ret = motor_check(TRUE);
    if(ret != X_RET_OK)
    {
        LOG_E(LOG_STD,"Check motors level1 NOK")
        error_log_results(&sys_mot);
        system_set_motor(&sys_mot);
        error_loop();
        CHECK_STOP_REQUEST();
    }
    else
    {
        LOG_I(LOG_STD,"Check motors level1 OK")
    }


    LOG_I(LOG_STD,"Checking motors level2...");
    volatile return_motor_cplx_t retcplx = error_test_1H();
    CHECK_STOP_REQUEST();
    if(flag_overcurrent_vm == TRUE)
    {
       sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
       error_log_results(&sys_mot);
       system_set_motor(&sys_mot);
       error_loop();
       CHECK_STOP_REQUEST();
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
    //system_set_motor(sys_mot);

    retcplx = error_test_1L();
    CHECK_STOP_REQUEST();
    if(flag_overcurrent_vm == TRUE)
    {
       sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
       error_log_results(&sys_mot);
       system_set_motor(&sys_mot);
       error_loop();
       CHECK_STOP_REQUEST();
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
    //system_set_motor(sys_mot);


    if(sys_mot.error_lvl1.value == 0x00 &&
           sys_mot.error_lvl2.value == 0x00)
    {
        LOG_I(LOG_STD,"Check motors level2 OK");
        LOG_I(LOG_STD,"Checking motors level3...");
        retcplx = error_test_2();
        CHECK_STOP_REQUEST();

        if(flag_overcurrent_vm == TRUE)
        {
           sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
           error_log_results(&sys_mot);
           system_set_motor(&sys_mot);
           error_loop();
           CHECK_STOP_REQUEST();
        }

        if(retcplx.code != F_RET_OK)
        {
            LOG_E(LOG_STD,"Check motors level3 NOK");
            if(retcplx.code == F_RET_PANELS_DAMAGED)
                sys_mot.error_lvl3.bits.damaged_panels = TRUE;
            else if(retcplx.code == F_RET_MOTOR_ERROR_API_FSP)
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
                 else  if(retcplx.fsp_motorL_error_code != 0x00)
                     sys_mot.error_lvl2.bits.unknown = TRUE;

                if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_HALL_PATTERN ||
                                 retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_PATTERN)
                 {
                     sys_mot.error_lvl2.bits.error_pattern_h = TRUE;
                 }
                 else if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_TIMEOUT)
                 {
                     sys_mot.error_lvl2.bits.timeout_pulses_h = TRUE;
                 }
                 else if(retcplx.fsp_motorH_error_code != 0x00)
                     sys_mot.error_lvl2.bits.unknown = TRUE;
            }
            system_set_motor(&sys_mot);
        }
        else
        {
            LOG_I(LOG_STD,"Check motors level3 OK");
        }
    }
    else
    {
        LOG_E(LOG_STD,"Check motors level2 NOK");
    }


    if(sys_mot.error_lvl3.bits.damaged_panels == TRUE)
    {
        LOG_I(LOG_STD,"Checking motors level3 advanced...");
        retcplx = error_test_3();
        CHECK_STOP_REQUEST();
        if(flag_overcurrent_vm == TRUE)
        {
           sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
           error_log_results(&sys_mot);
           system_set_motor(&sys_mot);
           error_loop();
           CHECK_STOP_REQUEST();
        }

        if(retcplx.code != F_RET_OK)
        {
            LOG_E(LOG_STD,"Check motors level3 advanced NOK");
            if(retcplx.code == F_RET_MOTOR_DRIVING_H)
                sys_mot.error_lvl3.bits.motor_driving_h = TRUE;
            else if(retcplx.code == F_RET_MOTOR_DRIVING_L)
                sys_mot.error_lvl3.bits.motor_driving_l = TRUE;
            else if(retcplx.code == F_RET_MOTOR_DRIVING_HL)
            {
                sys_mot.error_lvl3.bits.motor_driving_h = TRUE;
                sys_mot.error_lvl3.bits.motor_driving_l = TRUE;
            }
            else if(retcplx.code == F_RET_MOTOR_ERROR_API_FSP)
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
                 else  if(retcplx.fsp_motorL_error_code != 0x00)
                     sys_mot.error_lvl2.bits.unknown = TRUE;

                if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_HALL_PATTERN ||
                                 retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_PATTERN)
                 {
                     sys_mot.error_lvl2.bits.error_pattern_h = TRUE;
                 }
                 else if(retcplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_TIMEOUT)
                 {
                     sys_mot.error_lvl2.bits.timeout_pulses_h = TRUE;
                 }
                 else if(retcplx.fsp_motorH_error_code != 0x00)
                     sys_mot.error_lvl2.bits.unknown = TRUE;
            }
            system_set_motor(&sys_mot);
        }
        else
        {
            LOG_I(LOG_STD,"Check motors level3 advanced OK");
        }
    }

    CHECK_STOP_REQUEST();
    if(sys_mot.error_lvl1.value == 0x00 &&
       sys_mot.error_lvl2.value == 0x00 &&
       sys_mot.error_lvl3.value == 0x00)
    {
        system_set_motor(&sys_mot);
        LOG_W(LOG_STD,"No error detected -> starting init mode");
        CHECK_STOP_REQUEST();
        drive_control.running = FALSE;

        set_drive_mode(MOTOR_INIT_MODE);
        return X_RET_OK;
    }
    else
    {
        error_log_results(&sys_mot);
        error_loop();
        CHECK_STOP_REQUEST();
    }
    drive_control.running = FALSE;
    return ret;
}


