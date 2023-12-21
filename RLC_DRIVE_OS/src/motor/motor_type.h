/*
 * motor_type.h
 *
 *  Created on: Oct 13, 2023
 *      Author: Christophe
 */

#ifndef APPLICATION_MOTOR_MOTOR_TYPE_H_
#define APPLICATION_MOTOR_MOTOR_TYPE_H_

#include <stdint.h>
#include <hal_data.h>
#include <_core/c_common.h>
#include <_core/c_linked_list/c_linked_list.h>

#define MOTOR_PROFIL_INITIALISED     0x12345678U

typedef enum e_motor_type
{
    MOTOR_TYPE_UNKNOWN = 0,
    MOTOR_TYPE_RM_ITOH_BRAKE = 1,
    MOTOR_TYPE_RM_ALCOM = 2,
    MOTOR_TYPE_COUNT    = 3,
}motor_type_t;








/*

               ___________________________
              /                           \
             / |                         | \
            /  |                         |  \
           /   |                         |   \
          /    |                         |    \
         /     |                         |     \_______________
        /      |                         |                     \
_______/       |                         |     |              | \____
               |                         |     |              |
       |       |                         |     |              | |    |
       |       |                         |     |              | |    |
       |       |                         |     |              | |    |
           S1               S2              S3        S4       S5  S6

*/

typedef enum e_motor_control_type
{
    MOTOR_REGULATED_MODE = 0,
    MOTOR_NON_REGULATED_MODE = 1,
    MOTOR_BRAKE_MODE = 2,
}motor_control_type_t;

typedef enum e_motor_next_phase_condition
{
    MOTOR_NEXT_CONDITION_NONE = 0,
	MOTOR_NEXT_CONDITION_SPEEDH = 1,
	MOTOR_NEXT_CONDITION_SPEEDL = 2,
	MOTOR_NEXT_CONDITION_SPEEDHL = 3,
}e_motor_next_phase_condition_t;

typedef struct st_motor_control
{
    motor_control_type_t mode;
    struct
    {
        motor_ext_settings_t settings;
    }non_regulated;

    struct
    {
        float rpm;
    }regulated;

    struct
    {
        uint16_t mask;
    }brake;


}motor_control_t;


typedef struct st_motor_phase
{
	e_motor_next_phase_condition_t next_condition;
	uint32_t condition_timeout_ms;
	uint32_t post_tempo_ms;
    motor_control_t params_motors[2];
}motor_phase_t;


typedef struct st_motor_profil_t
{
    uint32_t initialised; ///< Permet de ne pas initialiser plusieurs fois le bloc
    motor_type_t type; ///< Modèle de moteurs utilisés
    motor_ext_technology_t technology; ///< Technologie des moteurs
    motor_ext_cfg_t cfg_motorH; ///< Configuration du comportement du moteur haut
    motor_ext_cfg_t cfg_motorL; ///< Configuration du comportement du moteur bas


    struct
    {
    uint16_t prime_band_upper_size;
    uint16_t prime_band_lower_size;
    uint16_t poster_size;
    }sizes;

    struct
    {
    	c_linked_list_t error;
    	c_linked_list_t off_no_brake;
    	c_linked_list_t off_brake;

        struct
        {
            c_linked_list_t enrl;
            c_linked_list_t enrl_stop;
            c_linked_list_t enrh;
            c_linked_list_t enrh_stop;
            c_linked_list_t derh;
        }manual;

        struct
        {
            c_linked_list_t stretch1;
            c_linked_list_t stretch2;
            c_linked_list_t enrh;
            c_linked_list_t enrl;
            c_linked_list_t posterAccelerate;
            c_linked_list_t posterDecelerate;
            c_linked_list_t posterStop;
            c_linked_list_t lowerBand;

        }init;

        struct
        {

        }automatic;
    }sequences;

    struct
    {
        uint8_t count;
        uint8_t index;
        int32_t positions[5];
    }panels;

}motor_profil_t;

return_t motor_init_type(motor_type_t type);


#endif /* APPLICATION_MOTOR_MOTOR_TYPE_H_ */
