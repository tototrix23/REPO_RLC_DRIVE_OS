/*
 * h_time.c
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */
#include <hal_data.h>
#include "impl_time.h"
#include <adc/adc.h>
#include <init.h>
#include <rtc/rtc.h>
static uint64_t impl_time_ms_global = 0;

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "impl_time"

TX_TIMER global_timer;
static uint8_t global_timer_initialised = 0;

return_t impl_time_init(void)
{
    return_t ret = X_RET_OK;
    if(global_timer_initialised == TRUE)
        return ret;

    UINT status;
    status = tx_timer_create(&global_timer,"global_timer",
                             impl_time_function, 0x0, GLOBAL_TIMER_TICKS_FIRST, GLOBAL_TIMER_TICKS,
                             TX_AUTO_ACTIVATE);
    impl_time_ms_global = 0;
    global_timer_initialised = TRUE;
    return ret;
}




return_t impl_time_update(c_timespan_h handler)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(handler != NULL);
#endif
    return_t err = X_RET_OK;

    uint64_t v1,v2;
    do{
        v1 = impl_time_ms_global;
        v2 = impl_time_ms_global;
    }while(v1 != v2);

    handler->ms = v1;
    return err;
}


void impl_time_function(ULONG param)
{
    PARAMETER_NOT_USED(param);
    uint64_t delta = (uint64_t)((1000/THREADX_TICKS) * GLOBAL_TIMER_TICKS);
    impl_time_ms_global += delta;
    //rtc_time
    bool_t adc_ready=FALSE;
    adc_is_ready(&adc_ready);
    if(adc_ready == TRUE)
        adc_capture();

    tx_mutex_get(&g_mutex_rtc,TX_WAIT_FOREVER);
    rtc.time_ms += delta;
    tx_mutex_put(&g_mutex_rtc);
}


