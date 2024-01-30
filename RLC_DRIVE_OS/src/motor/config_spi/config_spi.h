/*
 * config_spi.h
 *
 *  Created on: 30 janv. 2024
 *      Author: Ch.Leclercq
 */

#ifndef MOTOR_CONFIG_SPI_CONFIG_SPI_H_
#define MOTOR_CONFIG_SPI_CONFIG_SPI_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_hal/h_drv8316/h_drv8316.h>
#include <_lib_impl_cust/impl_spi_motors/impl_spi_motors.h>


extern h_drv8316_t drv_mot1;
extern h_drv8316_t drv_mot2;
extern i_spi_t interface_mot1;
extern i_spi_t interface_mot2;

return_t motor_config_spi_init(void);
return_t motor_config_spi(h_drv8316_t *ptr);


#endif /* MOTOR_CONFIG_SPI_CONFIG_SPI_H_ */
