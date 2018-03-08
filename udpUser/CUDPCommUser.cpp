/*
 * CUDPCommUser.cpp
 *
 *  Created on: May 27, 2017
 *      Author: Robert
 */

#include "CUDPCommUser.hpp"

#include <stdexcept>
#include <iostream>

namespace orobotix {

  CUDPCommUser::CUDPCommUser() {
    // TODO Auto-generated constructor stub
    robot_data_ = NULL;
    f_initialization_ = true;
	commandData_.packet_version = 1000.0;
  }

  CUDPCommUser::~CUDPCommUser() {
    // TODO Auto-generated destructor stub

    // kill watch dog timer
    //timer_delete(wd_timer_);

    // delete socket
    closeSocket();
  }

///** watch dog callback function */
//void CUDPCommUser::callback_watchdog(sigval val)
//{
//	 // get class pointer
//	 CUDPCommUser* com = (CUDPCommUser*) val.sival_ptr;
//
//	 // if watch dog has not been hit, the connect flag is reset to false.
//	 if(com->f_hit_wd_ == false)
//	 {
//		 flag_connect_ = false;
//		 std::cout<<"\n\nwarning: does not connect to the robot!!!";
//	 }
//	 else
//		 flag_connect_ = true;
//
//	 // reset watch dog flag
//	 com->f_hit_wd_ = false;
//}

/** initialize the udp communication */
  bool CUDPCommUser::init(SRobotDataUser *s_robot_data_user_) {
    try {
//		SDatabase* db = SDatabase::getInstance();
      if (robot_data_ == NULL) robot_data_ = s_robot_data_user_;
      if (initUser() == false) throw (std::runtime_error("cannot initialize the user udp!"));


//		// initialize watch dog timer
//		sig_.sigev_value.sival_ptr = (void*) this;
//		sig_.sigev_notify = SIGEV_THREAD;
//		sig_.sigev_notify_function = CUDPCommUser::callback_watchdog;
//		sig_.sigev_notify_attributes = NULL;
//		timer_create(CLOCK_REALTIME, &sig_, &wd_timer_);
//
//		itimerspec t = {{1,0},{1,0}};
//		timer_settime(wd_timer_, 0, &t, 0);

      f_hit_wd_ = false;
      f_initialization_ = true;

      return true;
    }
    catch (std::exception &e) {
      std::cout << "\nerror:CUDPCommUser::init():" << e.what();
    }

    return false;
  }

/** send commands to the robot*/
  void CUDPCommUser::thread_communication_send() {

//	while(flag_run_)
//	{
//        if(flag_background_) {sleep(1); continue;}

    // update command data
    updateUserCommandData();

    //send command to the robot
    if (sendCommandToRobot() == false) {
      flag_connect_ = false;
    } else {
      flag_connect_ = true; //this just means the network hardware let us sent a packet. Doesn't mean the robot is there.
    }
    //std::cout<<"\nsend data to the robot";

    usleep(PERIOD_SEND_); // 10ms
//	}
    //std::cout << "\nUDP send thread exiting.";
  }

/** start running communication thread */
  void CUDPCommUser::thread_communication_rev() {

//	while(flag_run_)
//	{
//        if(flag_background_) {sleep(1); continue;}
    //printf("receive data!\n");

    if (!receiveDataFromRobot()) {
      flag_connect_ = false;
      printf("no data received!\n");
      return;
    }
    //printf("update sensor info!\n");

    // update the sensor data
    updateRobotSensorInformation();

    // hit watch dog
    f_hit_wd_ = true;

    // print out the data
    printSensorData(&sensorData_);
//	}
  }


/** send command to the robot */
  bool CUDPCommUser::sendCommandToRobot() {

    if (sendto(fd_, (char *) &commandData_, sizeof(tUdpCommandData), 0, (struct sockaddr *) &remAddr_, slen_) < 0) {

      std::cout << "\nerror: CUDPCommUser::sendCommandToRobot()";

      return false;
    }

    return true;
  }

/** get Data from the robot */
  bool CUDPCommUser::receiveDataFromRobot() {
    /* now receive sensor data from the robot */
    //printf("receive sensor data from robot!\n"); //debug
    int recvlen = (int) recvfrom(fd_, (char *) &sensorData_, sizeof(tUdpSensorData), 0, (struct sockaddr *) &remAddr_,
                                 &slen_);
    if (recvlen != sizeof(tUdpSensorData)) {
      std::cout << "\nreceive message, but wrong size " << recvlen << ", expected size = " << sizeof(tUdpSensorData);
      return false;
    }


    return true;
  }

/** initialize the udp for user */
  bool CUDPCommUser::initUser() {
    try {
      slen_ = sizeof(remAddr_);

      std::string drone_ip = robot_data_->droneIP_;
      const char *server = drone_ip.c_str();

      /** start windows socket */
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
      WSADATA WSAData;
      WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

      /* create a socket */
      if ((fd_ = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        std::cout << "\nsocket created";

      /* bind it to all local addresses and pick any port number */
      memset((char *) &localAddr_, 0, sizeof(localAddr_));
      localAddr_.sin_family = AF_INET;
      localAddr_.sin_addr.s_addr = htonl(INADDR_ANY);
      localAddr_.sin_port = htons(0);

      if (bind(fd_, (struct sockaddr *) &localAddr_, sizeof(localAddr_)) < 0)
        throw (std::runtime_error("bind failed"));


      /* now define remaddr, the address to whom we want to send messages */
      /* For convenience, the host address is expressed as a numeric IP address */
      /* that we will convert to a binary format via inet_aton */
      memset((char *) &remAddr_, 0, sizeof(remAddr_));
      remAddr_.sin_family = AF_INET;
      remAddr_.sin_port = htons(SERVICE_PORT_);

#ifdef WIN32
      remAddr_.sin_addr.s_addr = inet_addr(server);
#else
      if (inet_aton(server, &remAddr_.sin_addr) == 0) throw (std::runtime_error("inet_aton() failed\n"));
#endif

      return true;
    }
    catch (std::exception &e) {
      std::cout << "\nerror:CUDPCommUser::initUser():" << e.what();
    }

    return false;
  }

/** close the socket */
  void CUDPCommUser::closeSocket() {
    if (fd_ >= 0) {
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
      closesocket(fd_);
      WSACleanup();
#else
      close(fd_);
#endif
      f_initialization_ = false;
    }
  }

/** update robot sensor information */
  void CUDPCommUser::updateRobotSensorInformation() {
    // depth
    robot_data_->depth_ = sensorData_.depth_;
    robot_data_->goal_depth_ = sensorData_.goal_depth_;

    // imu
    for (unsigned int i = 0; i < 6; i++)
      robot_data_->imu_[i] = sensorData_.imu_[i];

    // measured rpm
    for (int i = 0; i < robot_data_->thrusterN_; i++) {
      robot_data_->m_rpm_[i] = sensorData_.m_rpm_[i];
//        NSLog(@"th %d rpm %d",i,sensorData_.m_rpm_[i]);
    }

    // commanded rpm
    for (int i = 0; i < robot_data_->thrusterN_; i++)
      robot_data_->c_rpm_[i] = sensorData_.c_rpm_[i];

    // desired Eular Angle
    for (unsigned int i = 0; i < 3; i++)
      robot_data_->goal_Eular_[i] = sensorData_.goal_Eular_[i];

    // update sampling rate[control; imu; pressure sensor]; unit:Hz
    robot_data_->sampling_rate_control_ = sensorData_.sampling_rate_[0];
    robot_data_->sampling_rate_imu_ = sensorData_.sampling_rate_[1];
    robot_data_->sampling_rate_ps_ = sensorData_.sampling_rate_[2];

    // battery voltage
    // we actually want a low-pass-filtered version of battery voltage
    // Sensor values of 0.0 indicate that no voltage measurement has happened yet.
    if (sensorData_.battery_volt_ != 0.0) {
      if (robot_data_->batteryVolt_ == 0.0) { //this is our first valid measurement. Set to the instantaneous value.
        robot_data_->batteryVolt_ = sensorData_.battery_volt_;
      } else { //take a running average with past measurements
        robot_data_->batteryVolt_ = 0.98 * robot_data_->batteryVolt_ + 0.02 * sensorData_.battery_volt_;
      }
    }

    // total current
    robot_data_->total_current_ = sensorData_.total_current_;

    // temperature
    robot_data_->temperature_in_ = sensorData_.temperature_[0];
    robot_data_->temperature_out_ = sensorData_.temperature_[1];

    // control gain
    for (unsigned int i = 0; i < 6; i++) {
      robot_data_->cur_ctr_kp_[i] = sensorData_.cur_ctr_kp_[i];
      robot_data_->cur_ctr_kv_[i] = sensorData_.cur_ctr_kv_[i];
      robot_data_->cur_ctr_ki_[i] = sensorData_.cur_ctr_ki_[i];
    }

    // thruster acceleration
    robot_data_->cur_max_thruster_acc_ = sensorData_.cur_thruster_max_acc_;

    robot_data_->updateOrientationMatrix();

//    robot_data_->drone_Power_ = sensorData_.drone_Power_;
  }

/** load command to the udp data struct */
  void CUDPCommUser::updateUserCommandData() {
    for (int i = 0; i < robot_data_->thrusterN_; i++) {
      commandData_.c_rpm_[i] = robot_data_->t_rpm_[i];
    }

    for (int i = 0; i < 3; i++) {
      commandData_.c_forces_moments_[i] = robot_data_->c_forces_moments_[i];
    }

    // control gain
    for (int i = 0; i < 6; i++) {
      commandData_.ctr_kp_[i] = robot_data_->ctr_kp_[i];
      commandData_.ctr_kv_[i] = robot_data_->ctr_kv_[i];
      commandData_.ctr_ki_[i] = robot_data_->ctr_ki_[i];
    }

    // thruster acceleration
    commandData_.thruster_max_acc_ = robot_data_->max_thruster_acc_;

    // camera
    for (int i = 0; i < 3; i++) {
      commandData_.f_enable_camera_[i] = robot_data_->flag_enable_camera_[i];
    }

    commandData_.f_enable_individual_control_ = robot_data_->flag_enable_individual_control_;
    commandData_.f_enable_thruster_ = robot_data_->flag_enable_thruster_;
    commandData_.f_change_control_gain_ = robot_data_->flag_change_control_gain_;
    commandData_.f_reset_program_ = robot_data_->flag_resetProgram_;

    commandData_.led_brightness_top_ = robot_data_->led_brightness_top_;

    commandData_.led_brightness_bot_ = robot_data_->led_brightness_bot_;

    commandData_.gimbal_Pitch_ = robot_data_->gimbal_Pitch_;

    commandData_.depthLimit = robot_data_->depthLimit;

    commandData_.robot_config = robot_data_->config_id_;

    commandData_.please_shutdown = robot_data_->drone_please_power_off;
    if (robot_data_->drone_please_power_off) {
      printf("sending shutdown command.");
      //we'll only send the shutdown command once, so that the user will be able to turn the robot on again.
      robot_data_->drone_please_power_off = false;
    }
  }

/** print out the sensor data */
  void CUDPCommUser::printSensorData(tUdpSensorData *data) {
    std::cout << "-----------------------------" << std::endl;
    std::cout << "depth = " << data->depth_ << std::endl;
    std::cout << "imu =";
    for (int i = 0; i < 6; i++)
      std::cout << " " << data->imu_[i];
    std::cout << std::endl;
    printf("rx %d thrusters\n", robot_data_->thrusterN_);
    std::cout << "measured rpm =";
    for (int i = 0; i < robot_data_->thrusterN_; i++)
      std::cout << " " << data->m_rpm_[i];
    std::cout << std::endl;
    std::cout << "command rpm =";
    for (int i = 0; i < robot_data_->thrusterN_; i++)
      std::cout << " " << data->c_rpm_[i];
    std::cout << std::endl;
  }


} /* namespace orobotix */
