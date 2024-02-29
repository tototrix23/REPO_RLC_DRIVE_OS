/*
 * comms_packet.c
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */


#include "comms_packet.h"


return_t comms_packet_add(comms_packet_type_t type,char *ptr)
{
    return_t ret = X_RET_OK;

    tx_mutex_get(&g_comms_mutex,TX_WAIT_FOREVER);

    st_comms_packet_t *p = (st_comms_packet_t*)malloc(sizeof(st_comms_packet_t));
    if(p == NULL) return X_RET_ASSERTION;

    p->type = type;
    st_rtc_t r = rtc_get();
    memcpy(&p->rtc_info,&r,sizeof(st_rtc_t));
    size_t length = (size_t)strlen(ptr);
    if(length == 0)
    {
       free(p);
       tx_mutex_put(&g_comms_mutex);
       return X_RET_INVALID_PARAM;
    }

    char *ptr_text = (char *)malloc(length);
    if(ptr_text == NULL)
    {
        free(p);
        tx_mutex_put(&g_comms_mutex);
        return X_RET_ASSERTION;
    }

    p->ptr_json = ptr_text;
    volatile char *dest = p->ptr_json;
    memcpy(dest,ptr,length);

    uint32_t status = tx_queue_send(&comms_queue, &p, TX_NO_WAIT);
    if(status != TX_SUCCESS)
    {
        free(ptr_text);
        free(p);
        tx_mutex_put(&g_comms_mutex);
        return X_RET_CONTAINER_FULL;
    }

    tx_mutex_put(&g_comms_mutex);

    return ret;
}
