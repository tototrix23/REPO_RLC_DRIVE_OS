/*
 * impl_spi_motors.h
 *
 *  Created on: 25 sept. 2023
 *      Author: Ch.Leclercq
 */

#ifndef LIB_IMPL_CUST_IMPL_SPI_MOTORS_IMPL_SPI_MOTORS_H_
#define LIB_IMPL_CUST_IMPL_SPI_MOTORS_IMPL_SPI_MOTORS_H_

#include <_core/c_typedefs.h>
#include <_core/c_return_codes.h>
#include <_core/c_error/c_error.h>
#include <_hal/h_return_codes.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_return_codes.h>


return_t spi_motor_open(void);
return_t spi_motor_close(void);
return_t spi_motor_read(char *buffer_tx,char* buffer_rx,uint16_t count);
return_t spi_motor_write(char *buffer_tx,char* buffer_rx,uint16_t count);
return_t spi_motor_mot1_cs_active(void);
return_t spi_motor_mot1_cs_inactive(void);
return_t spi_motor_mot2_cs_active(void);
return_t spi_motor_mot2_cs_inactive(void);


#endif /* LIB_IMPL_CUST_IMPL_SPI_MOTORS_IMPL_SPI_MOTORS_H_ */
