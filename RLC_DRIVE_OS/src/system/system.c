/*
 * system.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "system.h"
bool_t flag_overcurrent_vm;
st_system_t system_inst;

void system_init(void)
{
    flag_overcurrent_vm = FALSE;
    memset(&system_inst,0x00,sizeof(system_inst));
}
