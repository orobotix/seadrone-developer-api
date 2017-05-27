# orobotix-user-control
Example C++ code to allow developers to control SeaDrone via UDP. 
Contains udp library as well as udp comm message and robotdata structure.

# Compatability:
To be used on Linux. Tested with Ubuntu 16 and Cmake 3.5, system connected via WiFi to Drone, both are on teh same subnet. Drone has IP 192.168.1.122.

# How to compile:

$ git clone

$ mkdir build

$ cd build

$ cmake ..

$ make -j4

$ ./orobotix_user_control

# Example Print out
udp test started!
-----------------------------
depth = 0.0703606
imu = 30.2248 0.0634347 0.12043 0.00212845 0.00744957 0.00106422
rx 5 thrusters
measured rpm = 0 1 3 0 0
command rpm = 0 0 0 0 0
-----------------------------
depth = 0.0652621
imu = 30.2169 0.0572493 0.139018 -0.00212845 0.0042569 0
rx 5 thrusters
measured rpm = 0 0 0 0 0
command rpm = 0 0 0 0 0
-----------------------------
depth = 0.061183
imu = 30.2124 0.0792742 0.124481 0.0042569 -0.00106422 0
rx 5 thrusters
measured rpm = 0 -1 2 -1 -1
command rpm = 0 0 0 0 0
-----------------------------
...
