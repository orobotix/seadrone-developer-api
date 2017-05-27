/*
 * CUDPCommUser.hpp
 *
 *  Created on: May 27, 2017
 *      Author: Robert
 */

#ifndef CUDPCOMMUSER_HPP_
#define CUDPCOMMUSER_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <string>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>

#ifndef _WRS_KERNEL
#include <sys/time.h>
#endif

#include <time.h>

#include "udp_comm.hpp"
#include "SRobotDataUser.hpp"

namespace orobotix {

class CUDPCommUser {
public:
	CUDPCommUser();
	virtual ~CUDPCommUser();

	/** initialize the udp communication */
	bool init(SRobotDataUser* s_robot_data_user_);

	/** start running communication thread - receive data from the robot*/
	void thread_communication_rev();

	/** send commands to the robot*/
	void thread_communication_send();
    
    /** close the socket */
	void closeSocket();
    
    /** flag - initialization */
    bool f_initialization_;


protected:
	/** initialize the udp for user */
	bool initUser();

	/** send command to the robot */
	bool sendCommandToRobot();

	/** get Data from the robot */
	bool receiveDataFromRobot();


	/** print out the sensor data */
	void printSensorData(tUdpSensorData* data);

	/** socket address that we want to send*/
	struct sockaddr_in remAddr_;

	/** local socket address*/
	struct sockaddr_in localAddr_;

	/** file description from the socket */
	int fd_;

	/** thruster command */
	tUdpCommandData commandData_;

	/** buffer for receiving message */
	tUdpSensorData sensorData_;

	/** socket size */
	socklen_t slen_;

//	// ========================================
//	// watch dog
//	// ========================================
//	/** watch dog callback function */
//	static void callback_watchdog(sigval val);

	/** watch dog timer to check the connection */
	//timer_t wd_timer_;

	/** watch dog signal event */
	sigevent sig_;

	/** flag for hitting watch dog */
	bool f_hit_wd_;

  /** flag to indicate if connected */
  bool flag_connect_;

	// =======================================
	// robot information
	// =======================================
	/** update robot sensor information */
	void updateRobotSensorInformation();

	/** load command to the udp data struct */
	void updateUserCommandData();

	/** robot sensor and actuator data */
	orobotix::SRobotDataUser* robot_data_;

};

} /* namespace orobotix */
#endif /* CUDPCOMMUSER_HPP_ */
