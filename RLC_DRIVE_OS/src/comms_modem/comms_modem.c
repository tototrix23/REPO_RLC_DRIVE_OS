/*
 * comms_modem.c
 *
 *  Created on: 26 févr. 2024
 *      Author: Ch.Leclercq
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <hal_data.h>
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include <cJSON/cJSON.h>
#include <cJSON/JSON_process.h>
#include <return_codes.h>
#include "comms_modem.h"


//https://176.142.122.240:5078/swagger/index.html
//https://176.142.122.240:5068/swagger/index.html

//BSP_DATA_FLASH_SIZE_BYTES
//BSP_FEATURE_FLASH_DATA_FLASH_START

typedef struct tx_info_t{
    bool_t in_progress;
    return_t code;
}tx_info_t;

typedef struct rx_info_t{
    bool_t in_progress;
    return_t code;
    bool_t waiting_for_process;
}rx_info_t;

#define RX_MAX_CHAR 4096



static bool_t outgoing_process_in_progress = FALSE;
static bool_t incoming_process_in_progress = FALSE;
static tx_info_t tx_info;
static rx_info_t rx_info;
static char rx_array[RX_MAX_CHAR];
static uint16_t rx_index = 0;
static void modem_send(char *data);
static return_t process_outgoing(char *type,char *data,uint8_t retry,uint16_t timeout_ms);
static void rx_clear(void);
static return_t verify_received_json_type(char *type);



void comms_modem_init(void)
{
    R_SCI_B_UART_Open (&g_uart_modem_ctrl, &g_uart_modem_cfg);
    rx_clear();
    outgoing_process_in_progress = FALSE;
    incoming_process_in_progress = FALSE;
}

void comms_modem_process_incoming(void)
{
    if(outgoing_process_in_progress == TRUE) return;
    if(rx_info.waiting_for_process == FALSE) return;
    incoming_process_in_progress = TRUE;
    goto end;



    end:
    rx_clear();
    incoming_process_in_progress = FALSE;
}

return_t comms_modem_get_datetime(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    strcpy(tx_array,"{\"type\": \"get_datetime\",\"data\":null }");

    ret =   process_outgoing("get_datetime",tx_array,5,500);
    if(ret != X_RET_OK)
    {
        goto end;
    }

    // Traitement de la réponse
    ret = json_process_get_datetime(rx_array);


    end:
    rx_clear();
    outgoing_process_in_progress = FALSE;
    return ret;
}

return_t comms_modem_get_serial(void)
{
    return_t ret = X_RET_OK;
    char tx_array[128];
    strcpy(tx_array,"{\"type\": \"get_serial\",\"data\":null }");

    ret =   process_outgoing("get_serial",tx_array,5,500);
    if(ret != X_RET_OK)
    {
        goto end;
    }

    // Traitement de la réponse
    ret = json_process_get_datetime(rx_array);


    end:
    rx_clear();
    outgoing_process_in_progress = FALSE;
    return ret;
}
















static return_t verify_received_json_type(char *type)
{
    return_t ret = X_RET_OK;
    const cJSON *json_type = NULL;
    cJSON *ptr_json = cJSON_Parse(rx_array);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }
    json_type = cJSON_GetObjectItemCaseSensitive(ptr_json, "type");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    if(!cJSON_IsString(json_type))
    {
        ret = X_RET_ERR_GENERIC;
        goto end;
    }

    if(strcmp(json_type->valuestring,type) != 0x0)
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }


    end:
    cJSON_Delete(ptr_json);
    return ret;
}


static return_t process_outgoing(char *type,char *data,uint8_t retry,uint16_t timeout_ms)
{
    return_t ret = X_RET_OK;
    outgoing_process_in_progress = TRUE;
    c_timespan_t ts;
    h_time_update(&ts);

    uint8_t r=0;
    bool_t finished = FALSE;
    do
    {
        rx_clear();
        modem_send(data);
        h_time_update(&ts);

        bool_t rx_finished = FALSE;
        bool_t rx_error = FALSE;
        do
        {
            bool_t elasped = FALSE;
            h_time_is_elapsed_ms(&ts, timeout_ms, &elasped);
            if(elasped)
            {
                if(r>=retry)
                {
                    ret = F_RET_COMMS_OUT_TIMEOUT;
                    goto end;
                }
                else
                {
                    r++;
                    rx_finished = TRUE;
                    rx_error = TRUE;
                }
            }
            else if(rx_info.waiting_for_process == TRUE)
            {
                if(verify_received_json_type(type) != X_RET_OK)
                {
                    if(r>=retry)
                    {
                        ret = F_RET_COMMS_OUT_BAD_RESPONSE;
                        goto end;
                    }
                    else
                    {
                        r++;
                        rx_finished = TRUE;
                        rx_error = TRUE;
                    }
                }
                else
                {
                    rx_finished = TRUE;
                    rx_error = FALSE;
                }
            }
        }while(!rx_finished);

        if(rx_error == TRUE)
        {
            delay_ms(100);
        }
        else
        {
            finished = TRUE;
            ret = X_RET_OK;
        }
    }while(!finished);



    end:
    outgoing_process_in_progress = FALSE;
    return ret;
}







static void modem_send(char *data)
{
    tx_info.in_progress=TRUE;
    R_SCI_B_UART_Write(&g_uart_modem_ctrl, (uint8_t*)data, strlen(data));
    while(tx_info.in_progress == TRUE)
        delay_ms(1);
}

static void rx_clear(void)
{
    tx_mutex_get(&g_mutex_uart_modem,TX_WAIT_FOREVER);
    rx_info.in_progress = FALSE;
    rx_index = 0;
    rx_info.waiting_for_process = FALSE;
    tx_mutex_put(&g_mutex_system);
}


void uart_modem_callback(uart_callback_args_t *p_args)
{
    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        rx_info.in_progress = TRUE;
        if(rx_index < RX_MAX_CHAR)
        {
            rx_array[rx_index++] = (char)p_args->data;
            rx_info.code = X_RET_OK;
            if(p_args->data == 0x00) rx_info.waiting_for_process = TRUE;
        }
        else
        {
            rx_info.code = X_RET_CONTAINER_FULL;
        }
    }
    else if(UART_EVENT_TX_DATA_EMPTY == p_args->event)
    {
        tx_info.in_progress = FALSE;
        tx_info.code = X_RET_OK;
    }
    else if((UART_EVENT_ERR_PARITY == p_args->event || UART_EVENT_ERR_FRAMING == p_args->event ||
            UART_EVENT_ERR_OVERFLOW == p_args->event || UART_EVENT_BREAK_DETECT == p_args->event)
            )
    {
        tx_info.in_progress = FALSE;
        tx_info.code = X_RET_ERR_GENERIC;
    }
    else
    {

    }

}
