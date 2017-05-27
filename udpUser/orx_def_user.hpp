/*
 * orx_def_user.hpp
 *
 *  Created on: May 27, 2017
 *      Author: Robert
 */

#ifndef ORX_DEF_USER_HPP_
#define ORX_DEF_USER_HPP_

// definition
// ==========================================================

/** max. amount of thruster + motor*/
#define MAX_THRUSTER_MOTOR_NUM 12


// udp communication
//===========================================================
/** buffer length */
#define BUFLEN_   2048

/** hard-coded port number */
#define SERVICE_PORT_  21234

/** period for sending the command */
#define PERIOD_SEND_ 100000 //10ms

/** udp type definition */
typedef unsigned char udp_uint8_t;  /*!< 1 byte unsigned */
typedef unsigned short udp_uint16_t; /*!< 2 byte unsigned */
typedef unsigned int udp_uint32_t; /*!< 4 byte unsigned */
typedef int udp_int32_t;  /*!< 4 byte signed */
typedef float udp_float_t;  /*!< 4 byte IEEE 754 float */


#endif /* ORX_DEF_USER_HPP_ */
