/*
 * motor_check.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "motor_check.h"
#include <motor/motor.h>
#include <motor/drive_process/drive_sequence.h>
#include <motor/config_spi/config_spi.h>
#include <system/system.h>
#include <adc/adc.h>
#include <return_codes.h>

return_t motor_check(bool_t long_vm_cuttof)
{
    return_t ret = X_RET_OK;

    st_system_motor_t sys_mot;
    memset(&sys_mot,0x00,sizeof(st_system_motor_t));


    // Desactivation de la tension moteur
    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
    // Fermeture du FSP
    //motor_deinit_fsp();

    if(long_vm_cuttof == TRUE)
       delay_ms(3000);
    else
       delay_ms(500);

    flag_overcurrent_vm = FALSE;
    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
    delay_ms(500);
    if(flag_overcurrent_vm == TRUE)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_LOW );
        sys_mot.error_lvl1.bits.overcurrent_vm = TRUE;
        system_set_motor(sys_mot);
        return F_RET_MOTOR_CHECK_ERROR;
    }
    else sys_mot.error_lvl1.bits.overcurrent_vm = FALSE;

    // Ouverture du FSP
    motor_init_fsp();
    motors_instance.motorH->motor_ctrl_instance->p_api->configSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorH);
    motors_instance.motorL->motor_ctrl_instance->p_api->configSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorL);
    delay_ms(50);
    // Vérification des alimentations des codeurs
    if(adc_inst.instantaneous.vhall1 < 11000 || adc_inst.instantaneous.vhall1 > 13000)
        sys_mot.error_lvl1.bits.vcc_hall_h = TRUE;
    else
        sys_mot.error_lvl1.bits.vcc_hall_h = FALSE;

    if(adc_inst.instantaneous.vhall2 < 11000 || adc_inst.instantaneous.vhall1 > 13000)
        sys_mot.error_lvl1.bits.vcc_hall_l = TRUE;
    else
        sys_mot.error_lvl1.bits.vcc_hall_l = FALSE;

    // Configuration du drivers haut
    ret = motor_config_spi_init();
    ret = motor_config_spi(&drv_mot1);
    if(ret != X_RET_OK)
        sys_mot.error_lvl1.bits.config_driver_h = TRUE;

    // Configuration du drivers bas
    ret = motor_config_spi(&drv_mot2);
    if(ret != X_RET_OK)
        sys_mot.error_lvl1.bits.config_driver_l = TRUE;

    system_set_motor(sys_mot);

    if(sys_mot.error_lvl1.value != 0x0)
        return F_RET_MOTOR_CHECK_ERROR;


    motor_profil_t *ptr = &motors_instance.profil;
    sequence_result_t sequence_result;
    motor_drive_sequence(&ptr->sequences.off_brake,MOTOR_SEQUENCE_CHECK_NONE,&sequence_result);


    return ret;
}

