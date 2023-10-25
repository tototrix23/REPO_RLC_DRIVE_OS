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


return_t motor_drive_sequence_start(c_linked_list_t *list);
void motor_drive_sequence_process(void);
return_t motor_drive_sequence_finished(bool_t *result);

#endif /* APPLICATION_MOTOR_DRIVE_PROCESS_DRIVE_SEQUENCE_H_ */
