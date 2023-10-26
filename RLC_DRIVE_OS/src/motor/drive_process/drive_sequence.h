/*
 * drive_sequence.h
 *
 *  Created on: 17 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_MOTOR_DRIVE_PROCESS_DRIVE_SEQUENCE_H_
#define APPLICATION_MOTOR_DRIVE_PROCESS_DRIVE_SEQUENCE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_linked_list/c_linked_list.h>
#include <motor/motor.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>


typedef enum e_sequence_finish_status
{
    SEQUENCE_FINISH_OK,
    SEQUENCE_FINISH_TIMEOUT,
    SEQUENCE_FINISH_ERROR,
}sequence_finish_status_t;

typedef struct st_sequence_result
{
    sequence_finish_status_t status;
    uint16_t errorH;
    uint16_t errorL;
}sequence_result_t;

#define MOTOR_SEQUENCE_CHECK_NONE           0x0000
#define MOTOR_SEQUENCE_CHECK_ERROR_START    0x0001
#define MOTOR_SEQUENCE_CHECK_ERROR_RUN      0x0002
#define MOTOR_SEQUENCE_CHECK_TIMEOUT        0x0003



return_t motor_drive_sequence(c_linked_list_t *list,uint16_t behaviour,sequence_result_t *result);


#endif /* APPLICATION_MOTOR_DRIVE_PROCESS_DRIVE_SEQUENCE_H_ */
