/*
    vehicle dynamics library 
    created for torque vectoring simulation
    Author: Drew McKinney
*/
#define _USE_MATH_DEFINES

#include<iostream>
#include<stdio.h>
#include<math.h>

using namespace std;

//constants
// K_u - understeer gradient K_u, 0 for neutral grad
const float K_u = 0.0;
// V_cg - velocity of center of gravity V_cg
const float V_cg = 1;
// Cy_f - cornering stiffness of the front tires in N/rad
const float Cy_f = 7085;
// Cy_r - cornering stiffness of rear tires in N/rad
const float Cy_r = 7302;
// l_r - distance between center of gravity and front axle, in meters
const float l_r = 0.77;
// l_f - distance between center of gravity and rear axle, meters
const float l_f = 0.885;
// m - mass of vehicle in kg

// full wheel radius in meters
const float W_r = 0.4572;
// gear ratio
const float G_r = 4.4;
// front track in meters
const float t_f = 1.3208;
// rear track in meters
const float t_r = 1.3208;

/*
    calculate desired yaw, rad/sec
    arguments, 
        steering angle in radians S, velocity V_cg
     
*/ 
float getDesiredYaw(float steerAngle, float vel_cg) {
    //calculate desired yaw from formula
    float desired_yaw_rate = (vel_cg / ((l_f + l_r) + K_u * pow(vel_cg, 2))) * steerAngle;
    return desired_yaw_rate;
}

/*

    Calculate delta torque 
    arguments,
        desired yaw,
        ,

*/

/*
    Calculate wheel velocities
    arguments,
        V_cg - velocity at center of gravity,
        yaw_rate - yaw rate around center of gravity
    return,
        vector - (V_fl, V_fr, V_rl, V_rr)
*/
float* calculateWheelVelocities(float* wheel_vels, float yaw_rate, float steer_angle){
    float V_fl = wheel_vels[0];
    float V_fr = wheel_vels[1];
    float V_rl = wheel_vels[2];
    float V_rr = wheel_vels[3];

    float V_x = 0.0;
    float V_y = 0.0;

    float R_fl = 0.0;
    float R_fr = 0.0;
    float R_rl = 0.0;
    float R_rr = 0.0;

    float e_fl = 0.0;
    float e_fr = 0.0;
    float e_rl = 0.0;
    float e_rr = 0.0;

    // calculate long and lat velocities
    V_x = cos(steer_angle) * V_cg;
    V_y = sin(steer_angle) * V_cg;

    // calculate distance from center of mass to wheel
    R_fl = sqrt(pow(t_f, 2.0) + pow(l_f, 2.0));
    R_fr = R_fl;
    R_rl = sqrt(pow(t_r, 2.0) + pow(l_r, 2.0));
    R_rr = R_rl;

    // calculate angles between wheel-ground contact point and undercarriage axis
    e_fl = atan(t_f / l_f);
    e_fr = atan(l_f / t_f);
    e_rl = atan(l_r / t_r);
    e_rr = atan(t_r / l_r);

    V_fl = sqrt(pow(V_x - (yaw_rate * R_fl * sin(e_fl)), 2) + pow(V_y + (yaw_rate * R_fl * cos(e_fl)), 2.0));
    V_fr = sqrt(pow(V_x + (yaw_rate * R_fr * sin(e_fr)), 2) + pow(V_y + (yaw_rate * R_fr * cos(e_fr)), 2.0));
    V_rl = sqrt(pow(V_x - (yaw_rate * R_rl * sin(e_rl)), 2) + pow(V_y - (yaw_rate * R_rl * cos(e_rl)), 2.0));
    V_rr = sqrt(pow(V_x + (yaw_rate * R_rr * sin(e_rr)), 2) + pow(V_y - (yaw_rate * R_rr * cos(e_rr)), 2.0));

    wheel_vels[0] = V_fl;
    wheel_vels[1] = V_fr;
    wheel_vels[2] = V_rl;
    wheel_vels[3] = V_rr;

    return wheel_vels;
}

/*
    Calculate slip ratio
    arguments,

*/


float calculateDeltaTorque(){
    return 0;
}

/*
    calculate actual yaw rate,
        steering angle,
        delta torque
*/

/*
    convert degrees to radians
*/
float degreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}