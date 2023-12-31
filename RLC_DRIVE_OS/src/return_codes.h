/*
 * return_codes.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef RETURN_CODES_H_
#define RETURN_CODES_H_


#define F_RET_ERROR_GENERIC                  -10000

#define F_RET_MOTOR_STOP_FLAG_TIMEOUT        -10100
#define F_RET_MOTOR_BAD_TYPE                 -10101

#define F_RET_MOTOR_SEQUENCE_ERROR_START     -10200
#define F_RET_MOTOR_SEQUENCE_ERROR_RUN       -10201
#define F_RET_MOTOR_SEQUENCE_ERROR_TIMEOUT   -10202

#define F_RET_MOTOR_INIT_CANCELLED           -10300
#define F_RET_MOTOR_INIT_DAMAGED_PANELS      -10301
#define F_RET_MOTOR_INIT_STRETCH             -10302
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_L      -10303
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_H      -10304
#define F_RET_MOTOR_INIT_UNEXPECTED_ERROR    -10305
#define F_RET_MOTOR_INIT_TIMEOUT_POSTER      -10306
#endif /* RETURN_CODES_H_ */
