/*
 * rtc.h
 *
 *  Created on: 26 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef RTC_RTC_H_
#define RTC_RTC_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


typedef struct st_rtc_t
{
    uint64_t time_ms;
    bool_t configured;
}st_rtc_t;


extern st_rtc_t rtc;

void rtc_init(void);
return_t rtc_set(uint64_t epoch);
st_rtc_t rtc_get(void);

#endif /* RTC_RTC_H_ */
