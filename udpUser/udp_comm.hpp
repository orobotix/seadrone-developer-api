/*
 * udp_comm.hpp
 *
 *  Edited on: May 27, 2017
 *      Author: Robert
 */

#ifndef UDP_COMM_HPP_
#define UDP_COMM_HPP_

#include "orx_def_user.hpp"

namespace orobotix
{

/** define the sensor information */
  typedef struct
  {
    // In old robot firmware, this position held depth.
    // That' why new packet version numbers are float32's starting at 1000.0, to avoid ambiguity.
    udp_float_t packet_version; //1000.0  
	//vs2013 not support in-class member initializers, move it to Class initialization function.

    /** depth sensor. unit: meter */
    udp_float_t depth_;

    /** desired depth. unit:meter */
    udp_float_t goal_depth_;

    /** IMU - position (rad) / velocity (rad/s)*/
    udp_float_t imu_[6];

    /** thruster+motor measured rpm */
    udp_int32_t m_rpm_[MAX_THRUSTER_MOTOR_NUM];

    /** thruster+motor commanded rpm */
    udp_int32_t c_rpm_[MAX_THRUSTER_MOTOR_NUM];

    /** goal posture [yaw, pitch, roll] : unit:degree */
    udp_float_t goal_Eular_[3];

    /** sampling rate [control; imu; pressure sensor]*/
    udp_float_t sampling_rate_[3];

    /** battery voltage. unit: Voltage */
    udp_float_t battery_volt_;

    /** total current value (Amp) */
    udp_float_t total_current_;

    /** temperature [ inside dome; outside dome]. unit: C  */
    udp_float_t temperature_[2];

    /** current control kp */
    udp_float_t cur_ctr_kp_[6];

    /** current control kv */
    udp_float_t cur_ctr_kv_[6];

    /** current control ki */
    udp_float_t cur_ctr_ki_[6];

    /** current thruster max acceleration (rpm/s^2) */
    udp_int32_t cur_thruster_max_acc_;

    /** magnetometer raw magnetic field vector. To use as a compass, the chip's offset must be calibrated out.*/
    udp_float_t mag_[3];

    /** seconds since the robot booted */
    udp_float_t robot_uptime; //the first 10 robots Eduardo shipped send a udp packet without robot_uptime. The iOS app assumes 100.0 if it's missing.

    udp_float_t robot_internal_pressure;//for future use. 0.0 for now.

    /** each bit is 1 if the corresponding jumper is in place on the PCB. Bit 0 (the LSB) is the jumper labeled "1"
     * For SeaDrone 1.0 PCBs, pcb_jumpers are all 0. For SeaDrone 2.0, pcb_jumpers=1. */
    udp_uint8_t pcb_jumpers;

    /** (higher bits are for future use)
     * bit 1 is 1 if the leak sensor detects water inside the robot. SeaDrone 1.0 robots do not have a leak sensor and always report 0.
     * bit 0 (LSB) is 1 if the IMU is still initializing. */
    udp_uint8_t status_flags;
  }tUdpSensorData;

/** define the command structure */
  typedef struct
  {
    udp_float_t packet_version;//=1000.0;
    /** controls individual thruster rpm if f_enable_individual_control_ is true*/
    udp_int32_t c_rpm_[MAX_THRUSTER_MOTOR_NUM];

    /** desired xyz force from thrusters */
    udp_float_t c_forces_moments_[3];
    
    /** goal orientation */
    udp_float_t goal_euler_angles_[3];
    /** camera flag; [cam0; cam1; cam2]
     * enabling low camera = 1;
     * high resolution camera = 2;
     * disabling camera = 0*/
    udp_uint8_t f_enable_camera_[3];

    /** If true, c_rpm_ will determine thruster rpm.
     * If false, robot determines thruster rpm to achieve the specified c_force_moments_ and goal_euler_angles_ */
    udp_uint8_t f_enable_individual_control_;

    /** set true to allow thrusters to move */
    udp_uint8_t f_enable_thruster_;

    /** flag for updating control gain */
    udp_uint8_t f_change_control_gain_;

    /** flag for reset the program */
    udp_uint8_t f_reset_program_;

    /** brightness of top LEDs*/
    udp_uint8_t led_brightness_top_;

    /** brightness of bottom LEDs*/
    udp_uint8_t led_brightness_bot_;

    /** control kp */
    udp_float_t ctr_kp_[6];

    /** control kv */
    udp_float_t ctr_kv_[6];

    /** control ki */
    udp_float_t ctr_ki_[6];

    /** thruster max acceleration (rpm/s^2) */
    udp_int32_t thruster_max_acc_;

    /** camera gimbal pitch (unit:rad) */
    udp_float_t gimbal_Pitch_;

    /** user-specified depth limit, e.g. 10.0 will limit depth to 10 meters. */
    udp_float_t depthLimit;

    /** thruster configuration. ("inspector"=1,"developer"=2).*/
    udp_uint8_t robot_config;

    /** 1 means the robot should turn off, 0 means stay on. The 1 will only be sent once.*/
    udp_uint8_t please_shutdown;

  }tUdpCommandData;


} // end of namespace

#endif /* UDP_COMM_HPP_ */
