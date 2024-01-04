/*
 * auto_mode.c
 *
 *  Created on: 22 déc. 2023
 *      Author: Ch.Leclercq
 */


#include "auto_mode.h"
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
#define LOG_MODULE    "AUTO"


#define AUTO_ENRH     0
#define AUTO_ENRL     1


static bool_t mode_running = FALSE;
static bool_t mode_stop_order = FALSE;

static bool_t check_stop_request(void);
static void scroll_stop(void);
static return_t poster_change_to_position(uint8_t direction,uint8_t index);


static void scroll_stop(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    motor_drive_sequence(&ptr->sequences.automatic.poster_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
}

static return_t poster_change_to_position(uint8_t direction,uint8_t index)
{
   return_t ret = X_RET_OK;
   motor_profil_t *ptr = &motors_instance.profil;
   sequence_result_t sequence_result;
   c_timespan_t ts;
   bool_t ts_elasped;
   bool_t end = FALSE;
   int32_t pulsesH;

   h_time_update(&ts);

   if(direction == AUTO_ENRH)
   {
       motor_drive_sequence(&ptr->sequences.automatic.poster_enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }
   else if(direction  == AUTO_ENRL)
   {
       motor_drive_sequence(&ptr->sequences.automatic.poster_enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }


   while(!end)
   {
       if(mode_stop_order == TRUE) return F_RET_MOTOR_AUTO_CANCELLED;
       h_time_is_elapsed_ms(&ts, 5000, &ts_elasped);
       if(ts_elasped == TRUE)
       {
           h_time_update(&ts);
           motor_drive_sequence(&ptr->sequences.off_no_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
           MOTORS_SET_ERROR_AND_RETURN(MOTORS_ERROR_TIMEOUT_CHANGING_POSTER,F_RET_MOTOR_AUTO_TIMEOUT_POSTER);
       }

       motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);

       if(direction == AUTO_ENRH)
       {
           if(abs(pulsesH) >= ptr->panels.positions[index]-7)
           {
               scroll_stop();
               end = TRUE;
           }
       }
       else
       {
           if(abs(pulsesH) <= ptr->panels.positions[index]+7)
           {
               scroll_stop();
               end = TRUE;
           }
       }
       tx_thread_sleep(1);
   }
   delay_ms(10);
   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
   LOG_D(LOG_STD,"PulsesH: %d",pulsesH);
   return ret;
}


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
        return TRUE;
    }
    else
        return FALSE;
}

void auto_mode_stop(void)
{
    mode_stop_order = TRUE;
}

bool_t auto_mode_is_running(void)
{
   return mode_running;
}


return_t auto_mode_process(void)
{
    return_t ret = X_RET_OK;
    volatile motor_profil_t *ptr = &motors_instance.profil;
    volatile uint8_t direction;
    return_t ret_nested;
    c_timespan_t ts;
    if(ptr->panels.index == ptr->panels.count-1)
        direction = AUTO_ENRL;
    else
        direction = AUTO_ENRH;
    volatile uint32_t delay;



    while(1)
    {
        if(check_stop_request()) return F_RET_MOTOR_AUTO_CANCELLED;

        if(direction == AUTO_ENRH)
        {
            ret_nested = poster_change_to_position(AUTO_ENRH,ptr->panels.index+1);
            if(check_stop_request()) return F_RET_MOTOR_AUTO_CANCELLED;
            ptr->panels.index++;
            if(ptr->panels.index == ptr->panels.count-1)
               direction = AUTO_ENRL;
        }
        else
        {
            ret_nested = poster_change_to_position(AUTO_ENRL,ptr->panels.index-1);
            if(check_stop_request()) return F_RET_MOTOR_AUTO_CANCELLED;
            ptr->panels.index--;
            if(ptr->panels.index == 0)
               direction = AUTO_ENRH;
        }

        h_time_update(&ts);
        delay = ptr->poster_showtime;
        if(ptr->panels.count > 2)
        {
            if((ptr->panels.index == 0) || (ptr->panels.index == ptr->panels.count-1))
                delay = delay*2;
        }

        bool_t ts_elasped = FALSE;
        do
        {
            h_time_is_elapsed_ms(&ts, delay, &ts_elasped);
            if(check_stop_request()) return F_RET_MOTOR_AUTO_CANCELLED;

        }while(ts_elasped == FALSE);


        tx_thread_sleep(1);
    }

    return ret;
}













