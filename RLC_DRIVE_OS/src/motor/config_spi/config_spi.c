/*
 * config_spi.c
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#include "config_spi.h"

h_drv8316_t drv_mot1;
h_drv8316_t drv_mot2;
i_spi_t interface_mot1;
i_spi_t interface_mot2;


return_t motor_config_spi_init(void)
{
  return_t ret = X_RET_OK;

  // Initialisation des interfaces SPI
  i_spi_init(&interface_mot1, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot1_cs_inactive, spi_motor_mot1_cs_active);
  i_spi_init(&interface_mot2, spi_motor_open, spi_motor_close, spi_motor_read, spi_motor_write, spi_motor_mot2_cs_inactive, spi_motor_mot2_cs_active);
  // Initialisation des fonctions relatives aux drivers
  ret = h_drv8316_init(&drv_mot1,&interface_mot1,FALSE);
  if(ret != X_RET_OK) return ret;
  ret = h_drv8316_init(&drv_mot2,&interface_mot2,FALSE);
  if(ret != X_RET_OK) return ret;

  return ret;
}

return_t motor_config_spi(h_drv8316_t *ptr)
{
    return_t ret = X_RET_OK;

    ret = h_drv8316_read_all_registers(ptr);
    if(ret != X_RET_OK) return ret;

    ptr->registers.ctrl2.bits.SLEW = 1;
    ptr->registers.ctrl2.bits.PWM_MODE = 0;
    ptr->registers.ctrl4.bits.OCP_MODE = 0;
    ptr->registers.ctrl4.bits.OCP_DEG = 0;
    ptr->registers.ctrl4.bits.OCP_LVL = 0;
    ptr->registers.ctrl5.bits.CSA_GAIN = 0;
    ptr->registers.ctrl5.bits.EN_AAR = 0;
    ptr->registers.ctrl5.bits.EN_ASR = 0;
    ptr->registers.ctrl10.bits.DLY_TARGET = 0xB;
    ptr->registers.ctrl10.bits.DLYCMP_EN = 1;

    ret = h_drv8316_write_all_registers(ptr);
    if(ret != X_RET_OK) return ret;

    return ret;
}
