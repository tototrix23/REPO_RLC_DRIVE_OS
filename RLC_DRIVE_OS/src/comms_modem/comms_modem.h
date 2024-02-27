/*
 * comms_modem.h
 *
 *  Created on: 26 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef COMMS_MODEM_COMMS_MODEM_H_
#define COMMS_MODEM_COMMS_MODEM_H_

#include <stdint.h>
#include <_core/c_common.h>

void comms_modem_init(void);
void comms_modem_process_incoming(void);
return_t comms_modem_get_datetime(void);
return_t comms_modem_get_serial(void);
#endif /* COMMS_MODEM_COMMS_MODEM_H_ */
