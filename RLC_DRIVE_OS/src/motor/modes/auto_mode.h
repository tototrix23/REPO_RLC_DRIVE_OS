/*
 * auto_mode.h
 *
 *  Created on: 22 déc. 2023
 *      Author: Ch.Leclercq
 */

#ifndef MOTOR_MODES_AUTO_MODE_H_
#define MOTOR_MODES_AUTO_MODE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


void auto_mode_stop(void);
bool_t auto_mode_is_running(void);
return_t auto_mode_process(void);


#endif /* MOTOR_MODES_AUTO_MODE_H_ */
