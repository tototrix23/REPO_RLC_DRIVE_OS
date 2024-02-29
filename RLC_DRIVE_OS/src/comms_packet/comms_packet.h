/*
 * comms_packet.h
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef COMMS_PACKET_COMMS_PACKET_H_
#define COMMS_PACKET_COMMS_PACKET_H_

#include <stdint.h>
#include <_core/c_common.h>
#include <rtc/rtc.h>
typedef enum e_comms_packet_type
{
    COMMS_EVENT,
    COMMS_PAYLOAD,
}comms_packet_type_t;

typedef struct st_comms_packet_t
{
    st_rtc_t rtc_info;
    comms_packet_type_t type;
    char *ptr_json;
}st_comms_packet_t;


return_t comms_packet_add(comms_packet_type_t type,char *ptr);


#endif /* COMMS_PACKET_COMMS_PACKET_H_ */
