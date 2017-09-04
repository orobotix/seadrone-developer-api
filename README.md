# orobotix-user-control
Example C++ code to allow developers to control SeaDrone via UDP.
Contains udp library as well as udp comm message and robotdata structure.

# File Overview:
main.cpp: some basic initialization and then sending and receiving continuously

udpUser/udp_comm.hpp: contains the udp message format, important to adhere to for communication with drone

udpUser/TunedParameters.hpp: controller parameters, tuned values

udpUser/SRobotDataUser.hpp: robot data struct, useful for keeping track of the robot state on the user side

udpUser/orx_def_user.hpp: some general definitions

udpUser/CUDPCommUser.hpp/.cpp: udp communication class

# Compatibility:
To be used on Linux. Tested on a laptop with Ubuntu 16 and Cmake 3.5, laptop is connected via wifi to drone. Drone has the ip 192.168.0.122. Laptop can have any ip on the 255.255.255.0 subnet. Port 8090 is used.

# How to compile this test code after cloning it:
```
cd orobotix-user-control
mkdir build
cd build
cmake ..
make -j4
```

# How to run:
a) Start drone
b) start test program:
```
cd build
./orobotix_user_control
```

# Example Print Out
```
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
...
```

# Accessing video stream
The SeaDrone robot serves its camera video using the open-source mjpg-streamer utility. This is an HTTP server on port 8090 of the robot.

Here are the steps to stream video:

1) Connect your laptop to the tether's "SeaDrone 5.0 Ghz" wifi. Ensure the robot is on and connected.

2) Command the robot to begin video streaming. There are two ways to do this:
 - Option 1: Connect to the robot using the SeaDrone iPad app. The iPad app automatically commands the robot to begin streaming.
 - Option 2: Use the developer API to send a command packet to enable streaming of camera 0. To do that, the part of the struct named `udp_uint8_t f_enable_camera_[3]` should be set to values ``{1,0,0}`` for 1280x720 video, or ``{2,0,0}`` for 1920x1080 video.

3) To verify that it's working, open a browser, go to `http://192.168.0.122:8090/?action=snapshot` and you should see a screenshot from the camera.

4) More documentation on the video streamer is here:
 https://github.com/jacksonliam/mjpg-streamer/blob/master/mjpg-streamer-experimental/plugins/output_http/README.md (Note that SeaDrone uses port 8090, not 8080.)
