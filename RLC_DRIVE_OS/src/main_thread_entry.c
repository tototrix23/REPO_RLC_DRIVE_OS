#include "main_thread.h"
#include "log_thread.h"

#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_protected/c_protected.h>
#include <_interfaces/i_spi/i_spi.h>
#include <_interfaces/i_time/i_time.h>
#include <_hal/h_time/h_time.h>
#include <_lib_impl_cust/impl_time/impl_time.h>
#include <_lib_impl_cust/impl_log/impl_log.h>
#include <adc/adc.h>
#include <remotectrl/remotectrl.h>
#include <motor/motor.h>
#include <leds/leds.h>
#include <vee/vee.h>
#include <rtc/rtc.h>
#include <cJSON/cJSON.h>
#include <cJSON/JSON_process.h>
#include <comms_packet/comms_packet.h>
#include <serial/serial.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "main thread"

extern TX_THREAD thread_motors;
extern TX_THREAD log_thread;

i_time_t i_time_interface_t;

const char json_file[] = "{\"type\":\"get_datetime\",\"data\":{\"status_code\":0,\"timestamp_unix\":1708955460000,\"timestamp\":\"2024-02-26T13:51:00Z\"}}";

void test_json(void)
{

    const cJSON *data = NULL;


    return_t ret = json_process_get_datetime(json_file);

    int status = 0;
    cJSON *ptr_json = cJSON_Parse(json_file);
    if(ptr_json == NULL) goto end;

    data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(data == NULL) goto end;

    cJSON *ts_unix = cJSON_GetObjectItemCaseSensitive(data, "timestamp_unix");
    if(ts_unix == NULL) goto end;

    volatile uint8_t end = 1;

    end:
    cJSON_Delete(ptr_json);

}


/* Main Thread entry function */
void main_thread_entry(void)
{
    volatile return_t ret = X_RET_OK;
    i_log.write_e = impl_log_write_e;
    i_log.write_d = impl_log_write_d;
    i_log.write_i = impl_log_write_i;
    i_log.write_w = impl_log_write_w;




    // Configuration de l'interface de gestion du temps
    i_time_init(&i_time_interface_t,impl_time_init, impl_time_update);
    h_time_init(&i_time_interface_t);
    // Configuration de la RTC virtuelle interne
    rtc_init();
    // Demarrage du Thread dédié aux LOGs
    tx_thread_resume(&log_thread);

    /*volatile uint32_t cnt=0;
    do
    {
        volatile return_t rcomms = comms_packet_add(COMMS_PAYLOAD,json_file);
        if(rcomms != X_RET_OK)
        {
           volatile uint8_t crash=1;
           crash=0;
        }
        else
        {
            cnt++;
        }
        delay_ms(20);
    }while(1);*/


    serials_set_name("patr1");
    serials_set_name(0x0);
    serials_set_name("patr2");



    // Initialisation de la partie moteurs (partie logicielle)
    motor_structures_init();
    motor_init_type(MOTOR_TYPE_RM_ITOH_BRAKE);

    // Initialisation de la partie ADC
    ret = adc_init();
    if(ret != X_RET_OK){
        LOG_E(LOG_STD,"INIT ADC ERROR");}
    else{
        LOG_I(LOG_STD,"INIT ADC SUCCESS");}

    // Initialisation de la partie moteurs (partie API)
    /*LOG_I(LOG_STD,"VM ON");
    R_IOPORT_PinWrite(&g_ioport_ctrl, VM_CMD,BSP_IO_LEVEL_HIGH );
    delay_ms(1000);
    LOG_I(LOG_STD,"INIT FSP MOTOR");
    motor_init_fsp();
    motors_instance.motorH->motor_ctrl_instance->p_api->configSet(motors_instance.motorH->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorH);
    motors_instance.motorL->motor_ctrl_instance->p_api->configSet(motors_instance.motorL->motor_ctrl_instance->p_ctrl,motors_instance.profil.cfg_motorL);
   */

    // Initialisation de la VEE (EEPROM virtuelle)
    vee_init();
    //


    // Initalisation des LEDs
    leds_init();
    //
    set_drive_mode(MOTOR_INIT_MODE);
    // Demarrage de la partie moteur
    tx_thread_resume(&thread_motors);

    c_timespan_t ts;
    h_time_update(&ts);



    /*uint32_t cnt=0;
    while(1)
    {
        LOG_D(LOG_STD,"Test %d",cnt);
        cnt++;
        tx_thread_sleep(100);
    }*/

    //set_drive_mode(MOTOR_INIT_MODE);

    /* TODO: add your own code here */
    while (1)
    {

        remotectrl_process();
        tx_thread_sleep(10);


        bool_t elapsed = FALSE;
        h_time_is_elapsed_ms(&ts, 1000, &elapsed);
        if(elapsed == TRUE)
        {
            h_time_update(&ts);
            /*st_adc_t adc_copy;
            c_protected_get_object(&adc_inst,&adc_copy,sizeof(st_adc_t));




            LOG_D(LOG_STD,"ADC avg: IIN=%d,VIN=%d,VH1=%d,VH2=%d",
                            adc_copy.average.iin,
                            adc_copy.average.vin,
                            adc_copy.average.vhall1,
                            adc_copy.average.vhall2);*/
        }

        /*delay_ms(500);

        st_adc_t adc_copy;
        c_protected_get_object(&adc_inst,&adc_copy,sizeof(st_adc_t));




        LOG_D(LOG_STD,"ADC avg: IIN=%d,VIN=%d,VH1=%d,VH2=%d",
                        adc_copy.average.iin,
                        adc_copy.average.vin,
                        adc_copy.average.vhall1,
                        adc_copy.average.vhall2);
*/



    }
}
