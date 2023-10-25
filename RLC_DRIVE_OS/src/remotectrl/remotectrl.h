/*
 * telecommande.h
 *
 *  Created on: 13 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef APPLICATION_REMOTECTRL_REMOTECTRL_H_
#define APPLICATION_REMOTECTRL_REMOTECTRL_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>

extern bsp_io_level_t m12_auto;
extern bsp_io_level_t m12_enrl;
extern bsp_io_level_t m12_enrh;
extern bsp_io_level_t m12_derh;

return_t remotectrl_process(void);


#endif /* APPLICATION_REMOTECTRL_REMOTECTRL_H_ */
