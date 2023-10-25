/*
 * init_mode.c
 *
 *  Created on: 18 oct. 2023
 *      Author: Ch.Leclercq
 */
#include "init_mode.h"
#include <motor/motor.h>
#include <motor/motor_type.h>
#include <motor/drive_mode.h>
#include <motor/drive_process/drive_process.h>
#include <motor/drive_process/drive_sequence.h>

typedef enum e_init_states_t
{
   INIT_NOP,
   INIT_START,

}init_states_t;

static init_states_t state = INIT_NOP;


void init_mode_start(void)
{
	motors_instance.mode = MOTOR_INIT_MODE;
	state = INIT_START;

}

void init_mode_process(void) {
	motor_profil_t *ptr = &motors_instance.profil;

	switch (state) {
	case INIT_NOP:
		break;

	case INIT_START:
		break;
	}
}
