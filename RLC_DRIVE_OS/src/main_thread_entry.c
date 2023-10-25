#include "main_thread.h"

#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_protected/c_protected.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_time/i_time.h>
#include <_hal/h_time/h_time.h>
#include <_lib_impl_cust/impl_time/impl_time.h>
#include <_lib_impl_cust/impl_log/impl_log.h>
#include <adc/adc.h>
#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main thread"

extern TX_THREAD thread_motors;
i_time_t i_time_interface_t;


/* Main Thread entry function */
void main_thread_entry(void)
{
    volatile return_t ret = X_RET_OK;
    i_log.write_e = impl_log_write_e;
    i_log.write_d = impl_log_write_d;
    i_log.write_i = impl_log_write_i;
    i_log.write_w = impl_log_write_w;


    i_time_init(&i_time_interface_t,impl_time_init, impl_time_update);
    h_time_init(&i_time_interface_t);



    ret = adc_init();
    if(ret != X_RET_OK){
        LOG_E(LOG_STD,"INIT ADC ERROR");}
    else{
        LOG_I(LOG_STD,"INIT ADC SUCCESS");}




    tx_thread_resume(&thread_motors);
    LOG_D(LOG_STD,"Start");


    /* TODO: add your own code here */
    while (1)
    {
        delay_ms(500);

        st_adc_t adc_copy;
        c_protected_get_object(&adc_inst,&adc_copy,sizeof(st_adc_t));


        /*LOG_D(LOG_STD,"ADC inst: IIN=%d,VIN=%d,VH1=%d,VH2=%d",
                adc_copy.instantaneous.iin,
                adc_copy.instantaneous.vin,
                adc_copy.instantaneous.vhall1,
                adc_copy.instantaneous.vhall2);*/

        LOG_D(LOG_STD,"ADC avg: IIN=%d,VIN=%d,VH1=%d,VH2=%d",
                        adc_copy.average.iin,
                        adc_copy.average.vin,
                        adc_copy.average.vhall1,
                        adc_copy.average.vhall2);




    }
}
