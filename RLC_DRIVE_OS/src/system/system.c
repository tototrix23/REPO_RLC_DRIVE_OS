/*
 * system.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "system.h"
#include <vee/vee.h>
bool_t flag_overcurrent_vm;
st_system_t system_inst;

void system_init(void)
{
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    flag_overcurrent_vm = FALSE;
    memset(&system_inst,0x00,sizeof(system_inst));
    tx_mutex_put(&g_mutex_system);
}


void system_set_motor(st_system_motor_t *ptr_value)
{
    st_system_t current = system_get();
    if(memcmp(ptr_value,&current.motor,sizeof(st_system_motor_t)) != 0)
    {
        tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
        memcpy(&system_inst.motor,ptr_value,sizeof(st_system_motor_t));
        //system_inst.motor = value;
        vee_write_by_ptr(&system_inst);
        tx_mutex_put(&g_mutex_system);
    }
}

st_system_t system_get(void)
{
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    st_system_t ret = system_inst;
    tx_mutex_put(&g_mutex_system);
    return ret;
}

void system_clear_all(void)
{
    system_init();
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    vee_write_by_ptr(&system_inst);
    tx_mutex_put(&g_mutex_system);
}

void system_clear_motor(void)
{
    if(system_check_error() == TRUE)
    {
        tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
        LOG_I(LOG_STD,"Reset des erreurs moteur");
        flag_overcurrent_vm = FALSE;
        memset(&system_inst.motor,0x00,sizeof(st_system_motor_t));
        vee_write_by_ptr(&system_inst);
        tx_mutex_put(&g_mutex_system);
    }
}

bool_t system_check_error(void)
{
    bool_t ret = FALSE;
    tx_mutex_get(&g_mutex_system,TX_WAIT_FOREVER);
    st_system_t sys = system_inst;
    if(sys.motor.error_lvl1.value != 0x00 ||
       sys.motor.error_lvl2.value != 0x00 ||
       sys.motor.error_lvl3.value != 0x00)
    {
        ret = TRUE;
    }
    tx_mutex_put(&g_mutex_system);
    return ret;
}
