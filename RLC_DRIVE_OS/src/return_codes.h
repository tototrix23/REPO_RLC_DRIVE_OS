/*
 * return_codes.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef RETURN_CODES_H_
#define RETURN_CODES_H_

#define F_RET_OK                                  0
#define F_RET_ERROR_GENERIC                  -10000

#define F_RET_MOTOR_STOP_FLAG_TIMEOUT        -10100
#define F_RET_MOTOR_BAD_TYPE                 -10101

#define F_RET_MOTOR_SEQUENCE_ERROR_START     -10200
#define F_RET_MOTOR_SEQUENCE_ERROR_RUN       -10201
#define F_RET_MOTOR_SEQUENCE_ERROR_TIMEOUT   -10202

#define F_RET_MOTOR_CHECK_ERROR              -10300

#define F_RET_MOTOR_INIT_DAMAGED_PANELS      -10301
#define F_RET_MOTOR_INIT_STRETCH             -10302
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_L      -10303
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_H      -10304
#define F_RET_MOTOR_INIT_UNEXPECTED_ERROR    -10305
#define F_RET_MOTOR_INIT_TIMEOUT_POSTER      -10306

#define F_RET_MOTOR_DRIVE_CANCELLED          -10400
#define F_RET_MOTOR_OVERCURRENT_VM           -10401
#define F_RET_MOTOR_ERROR_API_FSP            -10402
#define F_RET_MOTOR_ERROR_PULSESH            -10403
#define F_RET_MOTOR_ERROR_PULSESL            -10404

#define F_RET_MOTOR_AUTO_TIMEOUT_POSTER      -10500
#define F_RET_MOTOR_AUTO_OVERCURRENT         -10501
#define F_RET_MOTOR_AUTO_TIMEOUT_PULSES      -10502
#define F_RET_PANELS_DAMAGED                 -10503
#define F_RET_MOTOR_DRIVING_H                -10504
#define F_RET_MOTOR_DRIVING_L                -10505
#define F_RET_MOTOR_DRIVING_HL               -10506

#define F_RET_ERROR_VEE_OPENING              -11000
#define F_RET_ERROR_VEE_CLOSING              -11001
#define F_RET_ERROR_VEE_NOT_FOUND            -11002
#define F_RET_ERROR_VEE_WRITING_OPERATION    -11003
#define F_RET_ERROR_VEE_WRITING_TIMEOUT      -11004
#define F_RET_ERROR_VEE_READING              -11005
#define F_RET_ERROR_VEE_FORMATING            -11006
#define F_RET_ERROR_VEE_GETTING_STATUS       -11007
#define F_RET_ERROR_VEE_NOT_ERASED           -11008

#endif /* RETURN_CODES_H_ */
