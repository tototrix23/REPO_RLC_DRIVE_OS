#include "_lib_impl_cust/impl_log/impl_log.h"

#include "log_thread.h"

/* Log Thread entry function */
void log_thread_entry(void)
{
    log_t *ptr = 0x00;
    while (1)
    {
        volatile uint32_t status = tx_queue_receive(&log_queue, &ptr, TX_WAIT_FOREVER);
        if(status == TX_SUCCESS)
        {

            impl_log_write(ptr->mode,ptr->color,ptr->module, 0x00, ptr->func, ptr->line, ptr->text);
            free(ptr);
        }
        tx_thread_sleep (1);
    }
}
