#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <hal_data.h>
#include "common_data.h"
#include "tx_api.h"

#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include <_hal/h_log/h_log.h>
#include <_target/t_delay.h>
#include "impl_log.h"



void impl_log_init(void);
void impl_log_mode_CRLF(char* source, uint16_t size_source, char* dest, uint16_t size_dest);
void impl_log_write(uint8_t mode,uint8_t color,char*module,  char* file,const char* func, uint16_t line, char *va_string);
void impl_printf(char *buffer);


struct tx_info_t{
    volatile bool_t in_progress;
    return_t code;
};

volatile struct tx_info_t tx_info;


void impl_log_init(void)
{
   R_SCI_B_UART_Open (&g_uart_log_ctrl, &g_uart_log_cfg);
}

void impl_printf(char *buffer)
{
    tx_info.in_progress=TRUE;
    R_SCI_B_UART_Write(&g_uart_log_ctrl, (uint8_t*)buffer, strlen(buffer));
    tx_info.code = X_RET_OK;
    while(tx_info.in_progress) delay_ms(1);
    //R_BSP_SoftwareDelay(2, BSP_DELAY_UNITS_MILLISECONDS);

}

void impl_log_mode_CRLF(char* source, uint16_t size_source, char* dest, uint16_t size_dest)
{
    memset(dest, 0, size_dest);
    char* x = dest;
    uint16_t i = 0;
    for (i = 0; i < size_source; i++)
    {
        char c = *source;
        if (c == 13)
        {
            strcat(dest, "<CR>");
        }
        else if (c == 10)
        {
            strcat(dest, "<LF>");
        }
        else
        {
            uint16_t l = (uint16_t)strlen(dest);
            *(x + l) = c;
            *(x + l + 1) = 0;
        }
        source++;
    }
}


void impl_log_write(uint8_t mode,uint8_t color,char*module,  char* file,const char* func, uint16_t line, char *va_string)
{
    PARAMETER_NOT_USED(file);


    static bool_t log_initialised = FALSE;
    static c_timespan_t ts;
    char buffer_func[128];


    if(log_initialised == FALSE)
    {
        impl_log_init();
        c_timespan_init(&ts);
        log_initialised = TRUE;
    }

    h_time_update(&ts);

    uint64_t x = ts.ms;
    uint16_t hours = (uint16_t)(x/3600000);
    x = x%3600000;
    uint16_t minuts = (uint16_t)(x/60000);
    x = x%60000;
    uint16_t seconds = (uint16_t)(x/1000);
    x = x%1000;
    uint16_t ms = (uint16_t)(x%1000);


    snprintf(buffer_func,sizeof(buffer_func), "[%03d:%02d:%02d:%03d] ",hours,minuts,seconds,ms);
    impl_printf(buffer_func);

    switch(color)
        {
            case LOG_COLOR_ERROR:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<err> ");
                break;
            case LOG_COLOR_WARN:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<wrn> ");
                break;
            case LOG_COLOR_INFO:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<inf> ");
                break;
            case LOG_COLOR_DEBUG:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<dbg> ");
                break;
            case LOG_COLOR_DEFAULT:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<def> ");
                break;
            default:
                snprintf(buffer_func,sizeof(buffer_func), "%s","<def> ");
                break;
        }
    impl_printf(buffer_func);

    snprintf(buffer_func,sizeof(buffer_func), "%s: ",module);
    impl_printf(buffer_func);

    snprintf(buffer_func,sizeof(buffer_func), "%s(%d): ",func,line);
    impl_printf(buffer_func);

    if (mode == LOG_STD)
    {
        snprintf(buffer_func,sizeof(buffer_func), "%s \r\n",va_string);
        impl_printf(buffer_func);
    }
    else if (mode == LOG_CRLF)
    {
        memset(buffer_func, 0, sizeof(buffer_func));
        impl_log_mode_CRLF(va_string, (uint16_t)strlen(va_string), buffer_func, sizeof(buffer_func));
        impl_printf(buffer_func);
        impl_printf("\r\n");
    }



}




void impl_log_write_e(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp)
{
    PARAMETER_NOT_USED(file);
    log_t *ptr = (log_t*)malloc(sizeof(log_t));
    if(ptr != 0x00)
    {
        memset(ptr, 0, sizeof(log_t));
        vsnprintf(ptr->text,sizeof(ptr->text)-1, s, argp);
        strcpy(ptr->func,func);
        strcpy(ptr->module,module);
        ptr->line = line;
        ptr->mode = mode;
        ptr->color = LOG_COLOR_ERROR;

        uint32_t status = tx_queue_send(&log_queue, &ptr, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            free(ptr);
        }
    }
}

void impl_log_write_w(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp)
{
    PARAMETER_NOT_USED(file);
    log_t *ptr = (log_t*)malloc(sizeof(log_t));
    if(ptr != 0x00)
    {
        memset(ptr, 0, sizeof(log_t));
        vsnprintf(ptr->text,sizeof(ptr->text)-1, s, argp);
        strcpy(ptr->func,func);
        strcpy(ptr->module,module);
        ptr->line = line;
        ptr->mode = mode;
        ptr->color = LOG_COLOR_WARN;

        uint32_t status = tx_queue_send(&log_queue, &ptr, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            free(ptr);
        }
    }
}

void impl_log_write_i(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp)
{
    PARAMETER_NOT_USED(file);
    log_t *ptr = (log_t*)malloc(sizeof(log_t));
    if(ptr != 0x00)
    {
        memset(ptr, 0, sizeof(log_t));
        vsnprintf(ptr->text,sizeof(ptr->text)-1, s, argp);
        strcpy(ptr->func,func);
        strcpy(ptr->module,module);
        ptr->line = line;
        ptr->mode = mode;
        ptr->color = LOG_COLOR_INFO;

        uint32_t status = tx_queue_send(&log_queue, &ptr, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            free(ptr);
        }
    }
}

void impl_log_write_d(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp)
{
    PARAMETER_NOT_USED(file);
    log_t *ptr = (log_t*)malloc(sizeof(log_t));
    if(ptr != 0x00)
    {
        memset(ptr, 0, sizeof(log_t));
        vsnprintf(ptr->text,sizeof(ptr->text)-1, s, argp);
        strcpy(ptr->func,func);
        strcpy(ptr->module,module);
        ptr->line = line;
        ptr->mode = mode;
        ptr->color = LOG_COLOR_DEBUG;

        uint32_t status = tx_queue_send(&log_queue, &ptr, TX_NO_WAIT);
        if(status != TX_SUCCESS)
        {
            free(ptr);
        }
    }
}


void uart_log_callback(uart_callback_args_t *p_args)
{
    if(UART_EVENT_RX_CHAR == p_args->event)
    {

    }
    else if(UART_EVENT_TX_DATA_EMPTY == p_args->event)
    {
        tx_info.in_progress = FALSE;
        tx_info.code = X_RET_OK;

    }
    else if((UART_EVENT_ERR_PARITY == p_args->event || UART_EVENT_ERR_FRAMING == p_args->event ||
            UART_EVENT_ERR_OVERFLOW == p_args->event || UART_EVENT_BREAK_DETECT == p_args->event)
            /*&& tx_info.in_progress == TRUE*/)
    {
        tx_info.in_progress = FALSE;
        tx_info.code = X_RET_ERR_GENERIC;
    }
    else
    {
        tx_info.in_progress = FALSE;
        tx_info.code = X_RET_OK;
    }

}

