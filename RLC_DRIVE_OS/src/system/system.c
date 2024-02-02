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
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    flag_overcurrent_vm = FALSE;
    memset(&system_inst,0x00,sizeof(system_inst));
    tx_mutex_put(&g_mutex_system);
}


void system_set_motor(st_system_motor_t value)
{
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    system_inst.motor = value;
    tx_mutex_put(&g_mutex_system);
}

st_system_t system_get(void)
{
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    st_system_t ret = system_inst;
    tx_mutex_put(&g_mutex_system);
    return ret;
}
