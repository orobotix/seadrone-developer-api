#include <iostream>

// udpUser
#include "CUDPCommUser.hpp"

using namespace std;


int main() {

  printf("\nudp test started!");

  /** udp communication */
  orobotix::CUDPCommUser udpUser_;
  orobotix::SRobotDataUser robotData;

  // initialize the flag
  robotData.flag_enable_camera_[0] = 0;
  robotData.flag_enable_camera_[1] = 0;
  robotData.flag_enable_camera_[2] = 0;
  robotData.flag_enable_individual_control_ = false;
  robotData.flag_enable_thruster_ = false;

  bool flag_run_ = true;

  // set communication
  robotData.beagleboneIP_ = "192.168.1.122";
  robotData.beaglebonePort_ = 8090;

  // thruster
  robotData.initThrusterInfo(5);

  // initialize the udp communication
  udpUser_.init(&robotData);

  while (flag_run_) {
  udpUser_.thread_communication_rev();

    udpUser_.thread_communication_send();
  }
  printf("\nMain: udp_.closeConnection() called!");
  //udpUser_.closeSocket();

  printf("\nTerminated robotDemo app!\n");
  return EXIT_SUCCESS;
}