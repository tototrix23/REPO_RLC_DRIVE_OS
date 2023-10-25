/*
 * adc.h
 *
 *  Created on: 24 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <return_codes.h>


typedef struct st_adc_t
{
    struct
    {
        uint16_t iin;
        uint16_t vin;
        uint16_t vbatt;
        uint16_t vhall1;
        uint16_t vhall2;
    }average;
    struct
    {
        uint16_t iin;
        uint16_t vin;
        uint16_t vbatt;
        uint16_t vhall1;
        uint16_t vhall2;
    }instantaneous;
}st_adc_t;

extern st_adc_t adc_inst;


return_t adc_init(void);
return_t adc_capture(void);
return_t adc_is_ready(bool_t *result);

#endif /* ADC_ADC_H_ */
