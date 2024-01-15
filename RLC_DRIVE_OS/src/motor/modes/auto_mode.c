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
#include <adc/adc.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "AUTO"


#define AUTO_ENRH     0
#define AUTO_ENRL     1


typedef struct st_return_motor_cplx_t
{
    return_t code;
    uint32_t fsp_motorH_error_code;
    uint32_t fsp_motorL_error_code;
}return_motor_cplx_t;

static void return_motor_cplx_update(return_motor_cplx_t *ptr,return_t code);

static void scroll_stop(void);
static return_motor_cplx_t poster_change_to_position(uint8_t direction,uint8_t index);
static void poster_comp(uint8_t direction,uint8_t index);

static void scroll_stop(void)
{
    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    motor_drive_sequence(&ptr->sequences.automatic.poster_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
}

static void poster_comp(uint8_t direction,uint8_t index)
{
    motor_profil_t *ptr = &motors_instance.profil;
    int32_t pulsesH;
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);

    volatile int32_t diff = ptr->panels.positions[index] - pulsesH;


    if(direction == AUTO_ENRH)
    {
        ptr->panels.positions_compH[index] = ptr->panels.positions_compH[index]+diff;
        //LOG_D(LOG_STD,"compH panel%d -> %d",index,ptr->panels.positions_compH[index]);
    }
    else
    {
        ptr->panels.positions_compL[index] = ptr->panels.positions_compL[index]+diff;
        //LOG_D(LOG_STD,"compL panel%d -> %d",index,ptr->panels.positions_compL[index]);
    }
}


static void return_motor_cplx_update(return_motor_cplx_t *ptr,return_t code)
{
    ptr->code = code;
    ptr->fsp_motorH_error_code = motors_instance.motorH->error;
    ptr->fsp_motorL_error_code = motors_instance.motorL->error;
}

static return_motor_cplx_t poster_change_to_position(uint8_t direction,uint8_t index)
{
   return_motor_cplx_t ret;
   return_motor_cplx_update(&ret,X_RET_OK);

   motor_profil_t *ptr = &motors_instance.profil;
   sequence_result_t sequence_result;
   c_timespan_t ts;
   bool_t ts_elasped;
   bool_t end = FALSE;
   int32_t pulsesH;
   h_time_update(&ts);

   /*motors_instance.motorH->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,0);
   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);*/

   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
   LOG_I(LOG_STD,"start PulsesH: %d",pulsesH);
   if(direction == AUTO_ENRH)
   {
       motor_drive_sequence(&ptr->sequences.automatic.poster_enrh_slow,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }
   else if(direction  == AUTO_ENRL)
   {
       motor_drive_sequence(&ptr->sequences.automatic.poster_enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }

   CHECK_STOP_REQUEST_NESTED_CPLX();
   while(!end)
   {
       CHECK_STOP_REQUEST_NESTED_CPLX();
       h_time_is_elapsed_ms(&ts, 8000, &ts_elasped);
       if(ts_elasped == TRUE)
       {
           h_time_update(&ts);
           scroll_stop();
           return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_TIMEOUT_POSTER);
           LOG_E(LOG_STD,"timeout");
           return ret;
       }

       motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);

       int32_t pos;
       if(direction == AUTO_ENRH)
       {
           pos = ptr->panels.positions[index]+ptr->panels.positions_compH[index];
           if(abs(pulsesH) >= pos)
           {
              scroll_stop();
              end = TRUE;
           }
       }
       else
       {
           pos = ptr->panels.positions[index]+ptr->panels.positions_compL[index];
           if(abs(pulsesH) <= pos)
           {
               scroll_stop();
               end = TRUE;
           }
       }



       if(adc_inst.instantaneous.iin > ptr->current_stop)
       {
           scroll_stop();
           end = TRUE;
           return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_OVERCURRENT);
           LOG_W(LOG_STD,"Overcurrent");
           return ret;
       }

       if(motors_instance.motorH->error != 0x00 || motors_instance.motorL->error != 0x00)
       {
           scroll_stop();
           end = TRUE;
           return_motor_cplx_update(&ret,F_RET_MOTOR_ERROR_API_FSP);
           LOG_W(LOG_STD,"Error FSP");
           return ret;
       }
       tx_thread_sleep(1);
   }
   tx_thread_sleep(1);

   return_motor_cplx_update(&ret,X_RET_OK);
   return ret;
}





return_t auto_mode_process(void)
{
    drive_control.running = TRUE;
    LOG_D(LOG_STD,"START");
    return_t ret = X_RET_OK;
    motor_profil_t *ptr = &motors_instance.profil;
    volatile uint8_t direction;
    volatile return_motor_cplx_t ret_cplx;
    c_timespan_t ts;
    if(ptr->panels.index == ptr->panels.count-1)
        direction = AUTO_ENRL;
    else
        direction = AUTO_ENRH;
    volatile uint32_t delay;

    bool_t end=FALSE;
    uint8_t count = 0;
    delay_ms(500);
    ptr->panels.index = 0;

    ptr->panels.positions[0] = ptr->panels.positions_default[0];
    ptr->panels.positions[1] = ptr->panels.positions_default[1];
    ptr->panels.positions[2] = ptr->panels.positions_default[2];
    ptr->panels.positions[3] = ptr->panels.positions_default[3];
    ptr->panels.positions[4] = ptr->panels.positions_default[4];

    do
    {
        ret_cplx = poster_change_to_position(AUTO_ENRH,ptr->panels.index+count);
        CHECK_STOP_REQUEST();
        if(ret_cplx.code != X_RET_OK)
        {
            if(ret_cplx.code == F_RET_MOTOR_AUTO_OVERCURRENT)
            {
                LOG_D(LOG_STD,"%d panels detected",count);
                end = TRUE;
            }
            else if(ret_cplx.code == F_RET_MOTOR_ERROR_API_FSP)
            {
               if(ret_cplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_PATTERN)
               {
                   LOG_D(LOG_STD,"%d panels detected",count);
                   end = TRUE;
               }
               else
               {
                   LOG_E(LOG_STD,"Error %d",ret_cplx.code);
                   end = TRUE;
               }
            }
            else
            {
                LOG_E(LOG_STD,"Error %d",ret_cplx.code);
                end = TRUE;
            }
        }
        else
        {
            count++;

            h_time_update(&ts);
            bool_t ts_elasped = FALSE;
            do
            {
                h_time_is_elapsed_ms(&ts, ptr->poster_showtime, &ts_elasped);
                CHECK_STOP_REQUEST();
                tx_thread_sleep(1);
            }while(ts_elasped == FALSE);
        }
    }while(!end);


    sequence_result_t sequence_result;
    LOG_E(LOG_STD,"A");
    motor_drive_sequence(&ptr->sequences.automatic.lower_band_enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
    delay_ms(2000);
    LOG_E(LOG_STD,"B");
    motor_drive_sequence(&ptr->sequences.automatic.poster_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);


    int32_t pulsesH;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
    LOG_I(LOG_STD,"Total PulsesH: %d",pulsesH);

    ptr->panels.count = count;


    while(1)
    {
        CHECK_STOP_REQUEST();
        tx_thread_sleep(1);
    }





/*
    while(1)
    {
        if(drive_stop_request()) return F_RET_MOTOR_CANCELLED;

        if(direction == AUTO_ENRH)
        {
            ret_nested = poster_change_to_position(AUTO_ENRH,ptr->panels.index+1);
            if(ret_nested != X_RET_OK) return ret_nested;

            delay_ms(500);
            poster_comp(AUTO_ENRH,ptr->panels.index+1);

            ptr->panels.index++;
            if(ptr->panels.index == ptr->panels.count-1)
               direction = AUTO_ENRL;
        }
        else
        {
            ret_nested = poster_change_to_position(AUTO_ENRL,ptr->panels.index-1);
            if(ret_nested != X_RET_OK) return ret_nested;

            delay_ms(500);
            poster_comp(AUTO_ENRL,ptr->panels.index-1);

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

        delay = delay-500;

        bool_t ts_elasped = FALSE;
        do
        {
            h_time_is_elapsed_ms(&ts, delay, &ts_elasped);
            if(drive_stop_request()) return F_RET_MOTOR_CANCELLED;
            tx_thread_sleep(1);
        }while(ts_elasped == FALSE);


        tx_thread_sleep(1);
    }*/

    return ret;
}













