//
//  TunedParameters.h
//  Orobotix
//
//  Created by Eduardo Moreno on 4/9/17.
//

#ifndef TunedParameters_h
#define TunedParameters_h

// Eduardo, edit the values below to set the max speed in each direction and the default fraction of that.

// Vx, Vy, Vz, Wx, Wy, Wz.
const float MAX_SPEED_UPPER_BOUNDS[6] = { //"Max Speed" to set when the user puts the slider all the way to the right.
    20.0, //forward speed
    20.0, //lateral speed
    0.15, //diving speed
    90.0, 90.0, //Wx,Wy
    90.0}; // Wz a.k.a. "rotation speed"

const float MAX_SPEED_DEFAULT_FRACTION[6] = {//initial percentage of MAX_SPEED_UPPER_BOUND to be set when the app boots

    0.30, // fraction of the above forward speed
    0.30, // fraction of the above lateral speed
    0.50, // fraction of the above diving speed
    30.0 / 90.0, 30.0 / 90.0, //Wx,Wy. Not configurable in UI
    0.30}; // fraction of the above rotation speed

//original values:
//const float MAX_SPEED_DEFAULT_FRACTION[6] = {7.0/20.0,5.0/20.0,0.08/0.15,
//    30.0/90.0,30.0/90.0,30.0/90.0}; //initial percentage of MAX_SPEED_UPPER_BOUND to be set when the app boots

const float DEPTH_LIMIT_DEFAULT = 5.0; //Upper bound for this is set via the depth limit slider's attributes in main.storyboard

#endif /* TunedParameters_h */
