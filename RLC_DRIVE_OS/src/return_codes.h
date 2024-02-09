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


#endif /* RETURN_CODES_H_ */
