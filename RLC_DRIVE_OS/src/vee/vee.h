/*
 * vee.h
 *
 *  Created on: 21 févr. 2024
 *      Author: Ch.Leclercq
 */

#ifndef VEE_VEE_H_
#define VEE_VEE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>


typedef enum
{
    EEPROM_PATTERN,                    // char[16]
    EEPROM_SYSTEM_STATUS,
    EEPROM_VAR_COUNT
}nv_var_id;

return_t vee_open(void);
return_t vee_close(void);
return_t vee_init(void);
return_t vee_read_by_id(uint16_t id);
return_t vee_write_by_id(uint16_t id);
return_t vee_read_by_ptr(void *ptr_param);
return_t vee_write_by_ptr(void *ptr_param);
return_t vee_refresh(void);

#endif /* VEE_VEE_H_ */
