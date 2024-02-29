/*
 * serial.h
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

#include <stdint.h>
#include <_core/c_common.h>

typedef struct st_serials_t
{
    char imei[20];
    char name[32];
}st_serials_t;

extern st_serials_t system_serials;

void serials_init(void);
void serials_set_imei(char *text);
void serials_set_name(char *text);



#endif /* SERIAL_SERIAL_H_ */
