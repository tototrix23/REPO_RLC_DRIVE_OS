/*
 * rtc.c
 *
 *  Created on: 26 févr. 2024
 *      Author: Ch.Leclercq
 */

#include "rtc.h"


st_rtc_t rtc = {.time_ms=0,.configured=FALSE};


void rtc_init(void)
{
    tx_mutex_get(&g_mutex_rtc,TX_WAIT_FOREVER);
    memset(&rtc,0x00,sizeof(st_rtc_t));
    tx_mutex_put(&g_mutex_rtc);
}

return_t rtc_set(uint64_t epoch)
{
    return_t ret = X_RET_OK;

    tx_mutex_get(&g_mutex_rtc,TX_WAIT_FOREVER);
    rtc.time_ms = epoch;
    rtc.configured = TRUE;
    tx_mutex_put(&g_mutex_rtc);
    return ret;
}

st_rtc_t rtc_get(void)
{
    st_rtc_t ret;
    memset(&ret,0x00,sizeof(st_rtc_t));
    tx_mutex_get(&g_mutex_rtc,TX_WAIT_FOREVER);
    memcpy(&ret,&rtc,sizeof(st_rtc_t));
    tx_mutex_put(&g_mutex_rtc);
    return ret;
}
