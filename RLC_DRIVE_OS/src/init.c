/*
 * init.c
 *
 *  Created on: 24 oct. 2023
 *      Author: Ch.Leclercq
 */

#include "init.h"
#include <_lib_impl_cust/impl_time/impl_time.h>

void tx_application_define_user(void *first_unused_memory)
{
    PARAMETER_NOT_USED(first_unused_memory);
    impl_time_init();
}
