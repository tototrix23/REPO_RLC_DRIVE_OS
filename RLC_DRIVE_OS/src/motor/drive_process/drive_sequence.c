/*
 * drive_sequence.c
 *
 *  Created on: 17 oct. 2023
 *      Author: Ch.Leclercq
 */
#include "motor/drive_process/drive_sequence.h"
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>

static c_linked_list_t *sequence=0x00;
static uint32_t phase_count = 0x0;
static uint32_t phase_index = 0x0;
static motor_phase_t *phase = 0x00;
static c_timespan_t ts;
static uint8_t state = 0x00;
static bool_t finished = FALSE;



return_t motor_drive_sequence(c_linked_list_t *list)
{
    return_t ret = X_RET_OK;
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(list  != NULL)
#endif


    c_timespan_init(&ts);
    h_time_update(&ts);
    sequence = list;
    c_linked_list_get_count(sequence,&phase_count);
    if(phase_count == 0)
        return ret;

    motor_phase_t *phase = 0x00;
    uint8_t i=0;
    for(i=0;i<phase_count;i++)
    {
        c_linked_list_get_by_index(sequence,phase_index,(void**)&phase);


        for (i = 0; i < 2; i++)
        {
            motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
            switch(motors_instance.motors[i]->status)
            {
            case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                break;

            case MOTOR_120_DEGREE_CTRL_STATUS_RUN:

                break;

            case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:

                break;

            case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:

                motor_process(motors_instance.motors[i]);
                motors_instance.motors[i]->motor_ctrl_instance->p_api->reset(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                motor_wait_stop(motors_instance.motors[i]);
                motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);

                break;
            }
            switch(phase->params_motors[i].mode)
            {
                case MOTOR_REGULATED_MODE:
                    if (phase->params_motors[i].regulated.rpm != 0.0f)
                        motors_instance.motors[i]->motor_ctrl_instance->p_api->speedSet(
                                motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
                                phase->params_motors[i].regulated.rpm);
                    else
                        motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(
                                motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                    break;

                case MOTOR_NON_REGULATED_MODE:
                    if(phase->params_motors[i].non_regulated.settings.percent != 0)
                    motors_instance.motors[i]->motor_ctrl_instance->p_api->speedSetOpenLoop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
                            phase->params_motors[i].non_regulated.settings);
                    else
                        motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(
                                                    motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                    break;

                case MOTOR_BRAKE_MODE:
                    motors_instance.motors[i]->motor_ctrl_instance->p_api->brake(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
                    break;
            }
        }

        delay_ms(150);
        h_time_update(&ts);

        bool_t end = FALSE;
        do
        {

            if(phase->condition_timeout_ms != 0x00)
            {
                bool_t end = FALSE;
                h_time_is_elapsed_ms(&ts, phase->condition_timeout_ms, &end);
                if(end == TRUE)
                {
                    h_time_update(&ts);
                    end = TRUE;
                }
            }


            bool_t speeds_achieved[2]={0};
            for(i=0;i<2;i++)
            {
                motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
                switch(motors_instance.motors[i]->status)
                {
                case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
                    speeds_achieved[i] = TRUE;
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
                    motor_is_speed_achieved(motors_instance.motors[i],&speeds_achieved[i]);
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
                    speeds_achieved[i] = TRUE;
                    break;

                case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:

                    break;
                }
            }

            volatile motor_phase_t *pcond = phase;
            switch (pcond->next_condition)
            {
                case MOTOR_NEXT_CONDITION_NONE:
                    end = TRUE;
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDH:
                    if (speeds_achieved[0] == TRUE) {
                        end = TRUE;
                    }
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDL:
                    if (speeds_achieved[1] == TRUE) {
                        end = TRUE;
                    }
                    break;

                case MOTOR_NEXT_CONDITION_SPEEDHL:
                    if (speeds_achieved[0] == TRUE && speeds_achieved[1] == TRUE) {
                        end = TRUE;
                    }
                    break;
            }
            tx_thread_sleep(1);
        }while(!end);
    }
    return ret;
}



return_t motor_drive_sequence_start(c_linked_list_t *list)
{
    return_t ret = X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(list  != NULL)
#endif

    c_timespan_init(&ts);
    h_time_update(&ts);
    sequence = list;
    c_linked_list_get_count(sequence,&phase_count);
    finished = FALSE;
    phase_index = 0;
    if(phase_count >= 1)
    state = 1;
    else
    state = 0;

	return ret;
}


return_t motor_drive_sequence_finished(bool_t *result)
{
    return_t ret = X_RET_OK;

#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(result  != NULL)
#endif
    *result = finished;

	return ret;
}

void motor_drive_sequence_process(void) {
	uint32_t i = 0;
	switch (state) {
	case 0:
		finished = TRUE;
		break;
	case 1: {

		c_linked_list_get_by_index(sequence,phase_index,(void**)&phase);
		for (i = 0; i < 2; i++) {


			motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
			switch(motors_instance.motors[i]->status)
			{
			case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
				motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_RUN:

				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:

				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:
				motors_instance.motors[i]->motor_ctrl_instance->p_api->reset(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
				motor_wait_stop(motors_instance.motors[i]);
				motors_instance.motors[i]->motor_ctrl_instance->p_api->run(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);

				break;
			}

			switch(phase->params_motors[i].mode)
			{
			case MOTOR_REGULATED_MODE:
				if (phase->params_motors[i].regulated.rpm != 0.0f)
					motors_instance.motors[i]->motor_ctrl_instance->p_api->speedSet(
							motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
							phase->params_motors[i].regulated.rpm);
				else
					motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(
							motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
				break;

			case MOTOR_NON_REGULATED_MODE:
				if(phase->params_motors[i].non_regulated.settings.percent != 0)
				motors_instance.motors[i]->motor_ctrl_instance->p_api->speedSetOpenLoop(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl,
						phase->params_motors[i].non_regulated.settings);
				else
					motors_instance.motors[i]->motor_ctrl_instance->p_api->stop(
												motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
				break;

			case MOTOR_BRAKE_MODE:
				motors_instance.motors[i]->motor_ctrl_instance->p_api->brake(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl);
				break;
			}



		}
		h_time_update(&ts);
		state = 2;
	}
		break;

	case 2: {
		bool_t end = FALSE;
		h_time_is_elapsed_ms(&ts, 150, &end);
		if (end == TRUE) {
			h_time_update(&ts);
			state = 3;
		}
	}
		break;

	case 3:
		if(phase->condition_timeout_ms != 0x00)
		{
			bool_t end = FALSE;
			h_time_is_elapsed_ms(&ts, phase->condition_timeout_ms, &end);
			if(end == TRUE)
			{
				h_time_update(&ts);
				state = 4;
			}
		}



		bool_t speeds_achieved[2]={0};
		for(i=0;i<2;i++)
		{
			motors_instance.motors[i]->motor_ctrl_instance->p_api->statusGet(motors_instance.motors[i]->motor_ctrl_instance->p_ctrl, (uint8_t *)&motors_instance.motors[i]->status);
			switch(motors_instance.motors[i]->status)
			{
			case MOTOR_120_DEGREE_CTRL_STATUS_STOP:
				speeds_achieved[i] = TRUE;
				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_RUN:
				motor_is_speed_achieved(motors_instance.motors[i],&speeds_achieved[i]);
				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_BRAKE:
				speeds_achieved[i] = TRUE;
				break;

			case MOTOR_120_DEGREE_CTRL_STATUS_ERROR:

				break;
			}
		}




		switch (phase->next_condition) {
		case MOTOR_NEXT_CONDITION_NONE:
			state = 4;
			break;

		case MOTOR_NEXT_CONDITION_SPEEDH:
			if (speeds_achieved[0] == TRUE) {
				state = 4;
			}
			break;

		case MOTOR_NEXT_CONDITION_SPEEDL:
			if (speeds_achieved[1] == TRUE) {
				state = 4;
			}
			break;

		case MOTOR_NEXT_CONDITION_SPEEDHL:
			if (speeds_achieved[0] == TRUE && speeds_achieved[1] == TRUE) {
				state = 4;
			}
			break;
		}
		break;

		case 4:
			phase_index++;
			if(phase_index >= phase_count)
			{
				state = 0;
			}
			else
			{
				state = 1;
			}
			break;
	}
}
