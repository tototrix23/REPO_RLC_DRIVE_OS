/*
 * rm_motor_extension.h
 *
 *  Created on: 10 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef FSP_INC_API_RM_MOTOR_EXTENSION_H_
#define FSP_INC_API_RM_MOTOR_EXTENSION_H_

#include "bsp_api.h"
//#include "rm_motor_api.h"
FSP_HEADER

//@formatter:off

/** Enumeration des differentes technologies à supporter. */
typedef enum e_motor_ext_technology
{
    MOTOR_TECH_UNKNOWN = 0,
    MOTOR_TECH_DC,
    MOTOR_TECH_BLDC,
} motor_ext_technology_t;

/** Structure de configuration statique pour le moteur.*/
typedef struct st_motor_ext_cfg
{
    motor_ext_technology_t   motor_technology;   ///< Indique la technologie du moteur.
    uint8_t                  pulses_counting_reverse;
    uint8_t                  speed_reverse;
} motor_ext_cfg_t;

/** Structure contenant la consigne de rotation en mode libre
 * Cette structure sera utilisée par la partie 'profil moteur'
 */
typedef struct st_motor_ext_settings
{
    uint16_t brake_mask;
    int16_t percent;                    ///< Vitesse en pourcentage.
    uint32_t timeout_hall_ms;           ///< Timeout max sur les codeurs HALL.
    float   current_max;                ///< Courant maximum admissible.
} motor_ext_settings_t;

/** Structure utilisée par la partie API de Renesas
 * Elle sera jamais utilisée par le firmware directement
 */
typedef struct st_motor_ext_settings_api
{
    uint8_t active;                     ///< Indique si le mode brut non régulé est actif.
    float voltage;                      ///< Variable interne au fonctionnement qui contient la tension.
    motor_ext_settings_t settings;      ///< Instance de la structure contenant les paramètres non régulés.
} motor_ext_settings_api_t;

typedef struct st_motor_ext_pulses
{
    int32_t pulses;                     ///< Compteur de points.
} motor_ext_pulses_t;



FSP_FOOTER
//@formatter:on

#endif /* FSP_INC_API_RM_MOTOR_EXTENSION_H_ */
