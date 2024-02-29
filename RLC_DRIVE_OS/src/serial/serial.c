/*
 * serial.c
 *
 *  Created on: 28 févr. 2024
 *      Author: Ch.Leclercq
 */

#include "serial.h"

st_serials_t system_serials;

void serials_init(void)
{
    memset(&system_serials,0x00,sizeof(st_serials_t));
}

void serials_set_imei(char *text)
{
    if(text == 0x00 || strlen(text)==0x0)
    {
        size_t s = sizeof(system_serials.imei);
        memset(system_serials.imei,0x00,s);
    }
    else
    {
        strcpy(system_serials.imei,text);
    }
}


void serials_set_name(char *text)
{
    if(text == 0x00 || strlen(text)==0x0)
    {
        size_t s = sizeof(system_serials.name);
        memset(system_serials.name,0x00,s);
    }
    else
    {
        strcpy(system_serials.name,text);
    }
}

