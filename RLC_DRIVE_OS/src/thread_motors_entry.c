#include "thread_motors.h"
#include <_core/c_common.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_hal/h_drv8316/h_drv8316.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "motor thread"


h_drv8316_t drv_mot1;
h_drv8316_t drv_mot2;
i_spi_t interface_mot1;
i_spi_t interface_mot2;



void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);


    }
}


/* Motors Thread entry function */
void thread_motors_entry(void)
{
    volatile return_t ret = X_RET_OK;

    // Initialisation POEG
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);

    // Initialisation des interfaces SPI
    i_spi_init(&interface_mot1, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
    i_spi_init(&interface_mot2, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);
    // Initialisation des fonctions relatives aux drivers
    ret = h_drv8316_init(&drv_mot1,&interface_mot1,FALSE);
    ret = h_drv8316_init(&drv_mot2,&interface_mot2,FALSE);

    // Activation de l'alimentation du moteur
    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
    // Temporisation pour la stabilisation
    delay_ms(10);

    ret = h_drv8316_read_all_registers(&drv_mot1);
    drv_mot1.registers.ctrl2.bits.SLEW = 3;
    drv_mot1.registers.ctrl2.bits.PWM_MODE = 0;
    drv_mot1.registers.ctrl4.bits.OCP_MODE = 0;
    drv_mot1.registers.ctrl4.bits.OCP_DEG = 0;
    drv_mot1.registers.ctrl4.bits.OCP_LVL = 0;
    drv_mot1.registers.ctrl5.bits.CSA_GAIN = 0;
    drv_mot1.registers.ctrl5.bits.EN_AAR = 0;
    drv_mot1.registers.ctrl5.bits.EN_ASR = 0;
    drv_mot1.registers.ctrl10.bits.DLY_TARGET = 0x5;
    drv_mot1.registers.ctrl10.bits.DLYCMP_EN = 1;
    ret = h_drv8316_write_all_registers(&drv_mot1);
    ret = h_drv8316_read_all_registers(&drv_mot1);

    ret = h_drv8316_read_all_registers(&drv_mot2);
    drv_mot2.registers.ctrl2.bits.SLEW = 3;
    drv_mot2.registers.ctrl2.bits.PWM_MODE = 0;
    drv_mot2.registers.ctrl4.bits.OCP_MODE = 0;
    drv_mot2.registers.ctrl4.bits.OCP_DEG = 0;
    drv_mot2.registers.ctrl4.bits.OCP_LVL = 0;
    drv_mot2.registers.ctrl5.bits.CSA_GAIN = 0;
    drv_mot2.registers.ctrl5.bits.EN_AAR = 0;
    drv_mot2.registers.ctrl5.bits.EN_ASR = 0;
    drv_mot2.registers.ctrl10.bits.DLY_TARGET = 0x5;
    drv_mot2.registers.ctrl10.bits.DLYCMP_EN = 1;
    ret = h_drv8316_write_all_registers(&drv_mot2);
    ret = h_drv8316_read_all_registers(&drv_mot2);



    // Demarrage de la boucle de traitement
    while (1)
    {

        //tx_thread_sleep (1);
        delay_ms(500);
        //LOG_D(LOG_STD,"while motor");

    }
}
