#include <iostream>

// udpUser
#include "CUDPCommUser.hpp"

using namespace std;


int main() {

  printf("udp test started!\n");

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
  robotData.droneIP_ = "192.168.1.122";
  robotData.dronePort_ = 8090;

  robotData.drone_please_power_off = false;
  // thruster
  robotData.initThrusterInfo(5);

  // initialize the udp communication
  udpUser_.init(&robotData);

  while (flag_run_) {
    udpUser_.thread_communication_send();
    udpUser_.thread_communication_rev();

    //printf("while loop!");
    usleep(1000000);
  }
  printf("\nMain: udpUser_.closeConnection() called!");
  //udpUser_.closeSocket();

  printf("\nTerminated robotDemo app!\n");
  return EXIT_SUCCESS;
}