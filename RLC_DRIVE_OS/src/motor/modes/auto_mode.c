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


static bool_t init_phase = TRUE;


static void return_motor_cplx_update(return_motor_cplx_t *ptr,return_t code);
static void scroll_stop(void);
static void positions_process(void);
static void poster_comp(uint8_t direction,uint8_t index);
static return_motor_cplx_t poster_change_to_position(uint8_t direction,uint8_t index);
static return_motor_cplx_t low_band_enrh(void);
static return_motor_cplx_t low_band_enrl(void);


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

    //LOG_I(LOG_STD,"%d / %d (%d)",pulsesH,ptr->panels.positions[index],diff);

    if(direction == AUTO_ENRH)
    {
        ptr->panels.positions_compH[index] = ptr->panels.positions_compH[index]+diff;
        if(abs(ptr->panels.positions_compH[index]) > 100 )
        {
            LOG_W(LOG_STD,"Comp init ENRH on poster %d",index);
            ptr->panels.positions_compH[index] = 0;
        }
        //LOG_D(LOG_STD,"compH panel%d -> %d",index,ptr->panels.positions_compH[index]);
        LOG_I(LOG_STD,"%d / %d (%+04d) pos compH: %+04d",pulsesH,ptr->panels.positions[index],diff,ptr->panels.positions_compH[index]);
    }
    else
    {
        ptr->panels.positions_compL[index] = ptr->panels.positions_compL[index]+diff;
        if(abs(ptr->panels.positions_compL[index]) > 100 )
        {
            LOG_W(LOG_STD,"Comp init ENRL on poster %d",index);
            ptr->panels.positions_compL[index] = 0;
        }

        //LOG_D(LOG_STD,"compL panel%d -> %d",index,ptr->panels.positions_compL[index]);
        LOG_I(LOG_STD,"%d / %d (%+04d) pos compL: %+04d",pulsesH,ptr->panels.positions[index],diff,ptr->panels.positions_compL[index]);
    }




}


static void return_motor_cplx_update(return_motor_cplx_t *ptr,return_t code)
{
    ptr->code = code;
    ptr->fsp_motorH_error_code = motors_instance.motorH->error;
    ptr->fsp_motorL_error_code = motors_instance.motorL->error;
}


static void positions_process(void)
{

    motor_profil_t *ptr = &motors_instance.profil;
    memset(ptr->panels.positions_compH,0x00,sizeof(ptr->panels.positions_compH));
    memset(ptr->panels.positions_compL,0x00,sizeof(ptr->panels.positions_compL));

    int32_t last_panel_pulses;
    motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&last_panel_pulses);



    memset(ptr->panels.positions,0x00,sizeof(ptr->panels.positions));
    LOG_I(LOG_STD,"Panels count: %d",ptr->panels.count);
    if(ptr->panels.count == 0) return;

    float coeff=0.0f;
    switch(ptr->panels.count)
    {
        case 1:
            ptr->panels.positions[0] = ptr->sizes.prime_band_upper_size;
            break;

        case 2:
            ptr->panels.positions[0] = ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[1] = last_panel_pulses;
            break;

        case 3:
            coeff = (float)last_panel_pulses - (float)ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[0] = ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[1] = (int32_t)(coeff * 0.5132f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[2] = last_panel_pulses;
            break;

        case 4:
            coeff = (float)last_panel_pulses - (float)ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[0] = ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[1] = (int32_t)(coeff * 0.35054f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[2] = (int32_t)(coeff * 0.68305f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[3] = last_panel_pulses;
            break;

        case 5:
            coeff = (float)last_panel_pulses - (float)ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[0] = ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[1] = (int32_t)(coeff * 0.26777f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[2] = (int32_t)(coeff * 0.52269f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[3] = (int32_t)(coeff * 0.76631f) + ptr->sizes.prime_band_upper_size;
            ptr->panels.positions[4] = last_panel_pulses;
            break;

        default:
            break;
    }



    int32_t i=0;
    for(i=0;i<ptr->panels.count;i++)
        LOG_I(LOG_STD,"Pos%d = %d",i,ptr->panels.positions[i]);


    ptr->panels.index = ptr->panels.count-1;

    volatile uint8_t end=1;
}

static return_motor_cplx_t low_band_enrh(void)
{
   return_motor_cplx_t ret;
   return_motor_cplx_update(&ret,X_RET_OK);

   motor_profil_t *ptr = &motors_instance.profil;
   sequence_result_t sequence_result;
   c_timespan_t ts;
   bool_t ts_elasped;
   bool_t end = FALSE;
   h_time_update(&ts);
   int32_t pulsesH = 0;
   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
   LOG_I(LOG_STD,"low_band_enrh");

   motor_drive_sequence(&ptr->sequences.automatic.lower_band_enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   CHECK_STOP_REQUEST_NESTED_CPLX();
   do
   {
       CHECK_STOP_REQUEST_NESTED_CPLX();
       h_time_is_elapsed_ms(&ts, 1000, &ts_elasped);
       if(ts_elasped)
       {
           //motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
           //LOG_I(LOG_STD,"pulsesH: %d",pulsesH);

           end = TRUE;
           motor_drive_sequence(&ptr->sequences.automatic.poster_stop,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
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
   }while(!end);


   delay_ms(300);
   //motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
   //LOG_I(LOG_STD,"pulsesH: %d",pulsesH);

   return_motor_cplx_update(&ret,X_RET_OK);
   return ret;
}

static return_motor_cplx_t low_band_enrl(void)
{
   return_motor_cplx_t ret;
   return_motor_cplx_update(&ret,X_RET_OK);

   motor_profil_t *ptr = &motors_instance.profil;
   sequence_result_t sequence_result;
   c_timespan_t ts;
   c_timespan_t ts_error;
   bool_t error_flag = FALSE;
   bool_t ts_elasped;
   bool_t end = FALSE;
   int32_t pulsesL = 0;
   uint8_t error_count=0;
   LOG_I(LOG_STD,"low_band_enrl");
   h_time_update(&ts);
   h_time_update(&ts_error);

   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,0);
   motor_drive_sequence(&ptr->sequences.automatic.lower_band_enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);

   CHECK_STOP_REQUEST_NESTED_CPLX();
   do
   {
       CHECK_STOP_REQUEST_NESTED_CPLX();
       h_time_is_elapsed_ms(&ts, 3000, &ts_elasped);
       if(ts_elasped)
       {
           scroll_stop();
           return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_TIMEOUT_POSTER);
           LOG_E(LOG_STD,"timeout");
           return ret;
       }

       motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,&pulsesL);
       if(abs(pulsesL)>= ptr->sizes.prime_band_lower_size)
       {
           scroll_stop();
           end = TRUE;
       }


       h_time_is_elapsed_ms(&ts_error, 250, &ts_elasped);
       if(ts_elasped && error_flag==FALSE)
       {
           error_flag = TRUE;
           motors_instance.motorL->error = 0x10;
       }

       if(motors_instance.motorH->error != 0x00 || motors_instance.motorL->error != 0x00)
       {
          scroll_stop();
          error_count++;
          if(error_count >= 2)
          {
              end = TRUE;
              return_motor_cplx_update(&ret,F_RET_MOTOR_ERROR_API_FSP);
              LOG_E(LOG_STD,"Error FSP");
              return ret;
          }
          else
          {
              motors_instance.motorH->error=0x00;
              motors_instance.motorL->error=0x00;
              LOG_W(LOG_STD,"Error FSP, retry");
              motor_drive_sequence(&ptr->sequences.automatic.lower_band_enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
          }
       }
       tx_thread_sleep(1);

   }while(!end);

   delay_ms(300);
   return_motor_cplx_update(&ret,X_RET_OK);
   return ret;
}


static return_motor_cplx_t poster_change_to_position(uint8_t direction,uint8_t index)
{
   return_motor_cplx_t ret;
   return_motor_cplx_update(&ret,X_RET_OK);

   c_timespan_t ts_error;
   bool_t error_flag = FALSE;
   uint8_t error_count=0;

   motor_profil_t *ptr = &motors_instance.profil;
   sequence_result_t sequence_result;
   c_timespan_t ts;
   c_timespan_t ts2;
   bool_t ts_elasped;
   bool_t end = FALSE;
   uint32_t ovc_counter=0;
   int32_t pulsesH_start;
   int32_t pulsesH;
   int32_t pulsesH1;
   int32_t pulsesH2;
   bool_t init_speed_finished;
   poster_change_start:
   h_time_update(&ts_error);

   init_speed_finished = FALSE;
   h_time_update(&ts);

   motors_instance.motorL->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH_start);

   pulsesH1 = pulsesH_start;
   if(direction == AUTO_ENRH)
   {
       if(init_phase)
           motor_drive_sequence(&ptr->sequences.automatic.poster_init_enrh_slow,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
       else
           motor_drive_sequence(&ptr->sequences.automatic.poster_enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }
   else if(direction  == AUTO_ENRL)
   {
       motor_drive_sequence(&ptr->sequences.automatic.poster_enrl,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
   }

   h_time_update(&ts2);

   CHECK_STOP_REQUEST_NESTED_CPLX();
   while(!end)
   {
       CHECK_STOP_REQUEST_NESTED_CPLX();


       // Gestion d'un timeout sur le changement d'affiche
       uint32_t timeout_error;
       if(init_phase == TRUE)
           timeout_error = 5000;
       else
           timeout_error = 3500;

       h_time_is_elapsed_ms(&ts, timeout_error, &ts_elasped);
       if(ts_elasped == TRUE)
       {
           h_time_update(&ts);
           scroll_stop();
           return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_TIMEOUT_POSTER);
           LOG_E(LOG_STD,"timeout");
           return ret;
       }

       // Récupération du compteur de points hauts
       motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);

       // Gestion de la petite/grande vitesse pendant la phase d'init (comptage des affiches).
       if(init_phase == TRUE && init_speed_finished == FALSE)
       {
           if(abs(pulsesH) >= (pulsesH_start + ptr->sizes.prime_band_lower_size*2))
           {
               motor_drive_sequence(&ptr->sequences.automatic.poster_init_enrh,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);
               init_speed_finished = TRUE;
           }
       }


       // Gestion de l'arrêt sur position
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

       // Surveillance du courant maximum

       if(init_phase == TRUE)
       {
          uint32_t iin =  adc_inst.instantaneous.iin;
          h_time_is_elapsed_ms(&ts, 150, &ts_elasped);
          if(iin > ptr->current_stop && ts_elasped)
          {
              ovc_counter++;
              uint32_t ovc_max_value = 0;
              if(init_speed_finished == FALSE) ovc_max_value = 3;
              else ovc_max_value = 3;
              if(ovc_counter >= ovc_max_value)
              {
                  scroll_stop();
                  end = TRUE;
                  return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_OVERCURRENT);
                  LOG_W(LOG_STD,"Overcurrent");
                  return ret;
              }
          }
          else ovc_counter = 0;
       }

       /*uint32_t iin =  adc_inst.instantaneous.iin;
       h_time_is_elapsed_ms(&ts, 150, &ts_elasped);
       if(iin > ptr->current_stop && ts_elasped)
       {
           ovc_counter++;
           uint32_t ovc_max_value = 0;
           if (init_phase == TRUE && init_speed_finished == FALSE) ovc_max_value = 2;
           else ovc_max_value = 5;
           if(ovc_counter >= ovc_max_value)
           {
               scroll_stop();
               end = TRUE;
               return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_OVERCURRENT);
               LOG_W(LOG_STD,"Overcurrent");
               return ret;
           }
       }
       else ovc_counter = 0;*/

       /*h_time_is_elapsed_ms(&ts_error, 1500, &ts_elasped);
       if(ts_elasped && error_flag==FALSE)
       {
           error_flag = TRUE;
           motors_instance.motorL->error = 0x10;
       }*/

       // Surveillance des erreurs remontées par la librairie RENESAS
       if(motors_instance.motorH->error != 0x00 || motors_instance.motorL->error != 0x00)
       {
           scroll_stop();

           error_count++;
           if(error_count >= 2)
           {
               end = TRUE;
               return_motor_cplx_update(&ret,F_RET_MOTOR_ERROR_API_FSP);
               LOG_E(LOG_STD,"Error FSP");
               return ret;
           }
           else
           {
               motors_instance.motorH->error = 0;
               motors_instance.motorL->error = 0;
               LOG_W(LOG_STD,"Error FSP");
               goto poster_change_start;
           }
       }

       // Gestion de l'évolution du codeur haut.
       // Cela permet de detecter un train en butée.
       bool_t ts2_elasped;
       if(init_phase == TRUE && init_speed_finished == FALSE)
           h_time_is_elapsed_ms(&ts2, 400, &ts2_elasped);
       else
           h_time_is_elapsed_ms(&ts2, 400, &ts2_elasped);

       if(ts2_elasped == TRUE && init_phase == TRUE)
       {
           h_time_update(&ts2);
           motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH2);
           if(abs(pulsesH2 - pulsesH1) <= 3)
           {
               scroll_stop();
               LOG_W(LOG_STD,"no pulses");
               return_motor_cplx_update(&ret,F_RET_MOTOR_AUTO_TIMEOUT_PULSES);
               return ret;
           }
           pulsesH1 = pulsesH2;
       }
       tx_thread_sleep(1);
   }

   /*c_timespan_t final_ts;
   h_time_get_elapsed(&ts, &final_ts);
   uint32_t local_index = index;
   if(direction == AUTO_ENRH)
   {
       LOG_I(LOG_STD,"%dms panel %d ENRH",(uint32_t)final_ts.ms,(uint32_t)local_index);
   }
   else
   {
       LOG_I(LOG_STD,"%dms panel %d ENRL",(uint32_t)final_ts.ms,(uint32_t)local_index);
   }*/
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
    sequence_result_t sequence_result;
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
    ptr->panels.index = 0;
    ptr->panels.positions[0] = ptr->panels.positions_default[0];
    ptr->panels.positions[1] = ptr->panels.positions_default[1];
    ptr->panels.positions[2] = ptr->panels.positions_default[2];
    ptr->panels.positions[3] = ptr->panels.positions_default[3];
    ptr->panels.positions[4] = ptr->panels.positions_default[4];

    // Flag global permettant à la fonction 'poster_change_to_position' de piloter le moteur haut de manière différente
    // lors du comptage d'affiches
    init_phase = TRUE;

    do
    {
        ret_cplx = poster_change_to_position(AUTO_ENRH,ptr->panels.index+count);
        CHECK_STOP_REQUEST();
        if(ret_cplx.code != X_RET_OK)
        {
            if(ret_cplx.code == F_RET_MOTOR_AUTO_OVERCURRENT || ret_cplx.code ==F_RET_MOTOR_AUTO_TIMEOUT_PULSES)
            {
                LOG_I(LOG_STD,"%d panels detected",count);
                end = TRUE;
            }
            else if(ret_cplx.code == F_RET_MOTOR_ERROR_API_FSP)
            {
               if(ret_cplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_PATTERN || ret_cplx.fsp_motorH_error_code == MOTOR_ERROR_BEMF_TIMEOUT)
               {
                   LOG_I(LOG_STD,"%d panels detected",count);
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


    // A la fin du comptage d'affiches on tire à nouveau sur le train pour s'assurer qu'il est bien tendu
    ret_cplx = low_band_enrh();
    CHECK_STOP_REQUEST();
    if(ret_cplx.code != X_RET_OK)
    {
        LOG_E(LOG_STD,"Error %d",ret_cplx.code);
    }

    // Lecture du compteur de points du moteur haut. Le nombre de points correspond au total de tout le train d'affiches.
    int32_t pulsesH;
    motors_instance.motorH->motor_ctrl_instance->p_api->pulsesGet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,&pulsesH);
    LOG_I(LOG_STD,"Total PulsesH: %d",pulsesH);
    // On se positionne sur la dernière affiche
    ret_cplx = low_band_enrl();
    CHECK_STOP_REQUEST();
    if(ret_cplx.code != X_RET_OK)
    {
        LOG_E(LOG_STD,"Error %d",ret_cplx.code);
    }

    // On indique à la fonction 'poster_change_to_position' que le moteur haut peut maintenant être piloter en mode normal.
    init_phase = FALSE;

    ptr->panels.count = count;
    ptr->panels.index = count-1;
    direction = AUTO_ENRL;
    positions_process();
    delay_ms(500);

    while(1)
    {
        CHECK_STOP_REQUEST();
        if(ptr->panels.count > 1)
        {
            if(direction == AUTO_ENRH)
            {
                ret_cplx = poster_change_to_position(AUTO_ENRH,ptr->panels.index+1);
                CHECK_STOP_REQUEST();


                delay_ms(500);
                poster_comp(AUTO_ENRH,ptr->panels.index+1);

                ptr->panels.index++;
                if(ptr->panels.index == ptr->panels.count-1)
                   direction = AUTO_ENRL;
            }
            else
            {
                ret_cplx = poster_change_to_position(AUTO_ENRL,ptr->panels.index-1);
                CHECK_STOP_REQUEST();

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
                CHECK_STOP_REQUEST();
                tx_thread_sleep(1);
            }while(ts_elasped == FALSE);
        }
        tx_thread_sleep(1);
    }
    return ret;
}













