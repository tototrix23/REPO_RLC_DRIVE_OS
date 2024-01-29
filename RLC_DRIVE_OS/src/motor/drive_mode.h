/*
 * motor_mode.h
 *
 *  Created on: 17 oct. 2023
 *      Author: Christophe
 */

#ifndef APPLICATION_MOTOR_DRIVING_MODE_H_
#define APPLICATION_MOTOR_DRIVING_MODE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <system/system.h>

#define CHECK_STOP_REQUEST()              {\
                                          if(drive_stop_request())\
                                             {\
                                                 LOG_D(LOG_STD,"stop request");\
                                                 return F_RET_MOTOR_DRIVE_CANCELLED;\
                                             }\
                                          }\

#define CHECK_STOP_REQUEST_NESTED()       {\
                                          if(drive_control.stop_order == TRUE) return F_RET_MOTOR_DRIVE_CANCELLED;\
                                          }\

#define CHECK_STOP_REQUEST_NESTED_CPLX()  {\
                                          if(drive_control.stop_order == TRUE) \
                                             {\
                                              return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVE_CANCELLED);\
                                              return ret;\
                                             }\
                                             else if(system_instance.error_hw.overcurrent_hw == TRUE)\
                                             {\
                                              return_motor_cplx_update(&ret,F_RET_MOTOR_DRIVE_CANCELLED);\
                                              return ret;\
                                             }\
                                          }\


typedef enum  e_drive_mode
{
    MOTOR_UNKNOWN_MODE = 0,
    MOTOR_MANUAL_MODE  = 1,
    MOTOR_INIT_MODE    = 2,
    MOTOR_AUTO_MODE    = 3,
    MOTOR_ERROR_MODE   = 4
} drive_mode_t;


typedef struct st_drive_control_t
{
    bool_t stop_order;
    bool_t running;
    bool_t changing;
}drive_control_t;

extern drive_control_t drive_control;

bool_t drive_stop_request(void);
return_t set_drive_mode(drive_mode_t mode);


#endif /* APPLICATION_MOTOR_DRIVING_MODE_H_ */
