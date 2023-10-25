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

typedef enum e_manual_states_t
{
   MANUAL_NOP,
   MANUAL_START,
   MANUAL_WAIT,
   MANUAL_IDLE,
   MANUAL_WAIT_PROCESS,
}manual_states_t;

static manual_states_t state = MANUAL_NOP;
static uint8_t remote_state = 0;

void manual_mode_start(void)
{
	remote_state = 0;
	motors_instance.mode = MOTOR_MANUAL_MODE;
	state = MANUAL_START;
}

void manual_mode_process(void) {
	motor_profil_t *ptr = &motors_instance.profil;
    bool_t finished=FALSE;
    uint8_t remote = 0x00;
    if(m12_enrh) remote = remote | 0x01;
    if(m12_enrl) remote = remote | 0x02;
    if(m12_derh) remote = remote | 0x04;

    bool_t end = FALSE;
    motor_drive_sequence(&ptr->sequences.manual.off_sequence);
    if(motors_instance.mode != MOTOR_MANUAL_MODE)
        return;

    do
    {
        remote = 0x00;
        if(m12_enrh) remote = remote | 0x01;
        if(m12_enrl) remote = remote | 0x02;
        if(m12_derh) remote = remote | 0x04;
        if(remote != remote_state)
        {
            remote_state = remote;

            if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                                   m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrh_sequence);
            }
            else if(m12_enrh == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.enrl_sequence);
            }
            else if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
                    m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
                    m12_derh == REMOTECTRL_ACTIVE_LEVEL)
            {
                motor_drive_sequence(&ptr->sequences.manual.derh_sequence);
            }
            else
            {
                motor_drive_sequence(&ptr->sequences.manual.off_sequence);
            }
        }

        tx_thread_sleep(1);

        if(motors_instance.mode != MOTOR_MANUAL_MODE)
          end = TRUE;

    }while(!end);



	/*switch (state) {
	case MANUAL_NOP:
		break;

	case MANUAL_START:
		motor_drive_sequence_start(&ptr->sequences.manual.off_sequence);
		state = MANUAL_WAIT;
		break;

	case MANUAL_WAIT:
		motor_drive_sequence_finished(&finished);
		if(finished == TRUE)
		{
	      remote_state = 0x00;
          state = MANUAL_IDLE;
		}
		break;

	case MANUAL_IDLE:
		if(remote != remote_state)
		{
			remote_state = remote;


			if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
			           m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
					   m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
			{
				motor_drive_sequence_start(&ptr->sequences.manual.enrh_sequence);
			}
			else if(m12_enrh == !REMOTECTRL_ACTIVE_LEVEL &&
					m12_enrl == REMOTECTRL_ACTIVE_LEVEL &&
					m12_derh == !REMOTECTRL_ACTIVE_LEVEL)
			{
				motor_drive_sequence_start(&ptr->sequences.manual.enrl_sequence);
			}
			else if(m12_enrh == REMOTECTRL_ACTIVE_LEVEL &&
					m12_enrl == !REMOTECTRL_ACTIVE_LEVEL &&
					m12_derh == REMOTECTRL_ACTIVE_LEVEL)
			{
				motor_drive_sequence_start(&ptr->sequences.manual.derh_sequence);
			}
			else
			{
				motor_drive_sequence_start(&ptr->sequences.manual.off_sequence);
			}

			state = MANUAL_WAIT_PROCESS;
		}

		break;

	case MANUAL_WAIT_PROCESS:
		motor_drive_sequence_finished(&finished);
		if(finished == TRUE && (remote != remote_state))
		{
		  state = MANUAL_START;
		}
		break;
	}*/
}

#endif /* APPLICATION_MOTOR_MODES_MANUAL_MODE_C_ */
