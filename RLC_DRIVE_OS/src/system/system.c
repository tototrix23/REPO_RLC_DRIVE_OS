/*
 * system.c
 *
 *  Created on: 29 janv. 2024
 *      Author: Ch.Leclercq
 */
#include "system.h"

st_system_t system_instance;

void system_init(void)
{
    memset(&system_instance,0x00,sizeof(system_instance));
}

