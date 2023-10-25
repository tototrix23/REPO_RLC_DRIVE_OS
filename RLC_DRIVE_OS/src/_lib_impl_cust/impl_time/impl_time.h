/*
 * impl_time.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef LIB_IMPL_CUST_IMPL_TIME_IMPL_TIME_H_
#define LIB_IMPL_CUST_IMPL_TIME_IMPL_TIME_H_

#include <_core/c_typedefs.h>
#include <_core/c_return_codes.h>
#include <_core/c_error/c_error.h>
#include <_hal/h_return_codes.h>
#include <_interfaces/i_time/i_time.h>
#include <_interfaces/i_return_codes.h>

#define TIMER_LIB_MS  1
return_t impl_time_init(void);
return_t impl_time_update(c_timespan_h handler);
void impl_time_function(ULONG param);

#endif /* LIB_IMPL_CUST_IMPL_TIME_IMPL_TIME_H_ */
