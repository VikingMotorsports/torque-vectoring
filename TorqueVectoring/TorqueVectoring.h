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
const double K_u = 0.0;
// V_cg - velocity of center of gravity V_cg
const double V_cg = 1.0;
// Cy_f - cornering stiffness of the front tires in N/rad
const double Cy_f = 7085;
// Cy_r - cornering stiffness of rear tires in N/rad
const double Cy_r = 7302;
// l_r - distance between center of gravity and front axle, in meters
const double l_r = 0.77;
// l_f - distance between center of gravity and rear axle, meters
const double l_f = 0.885;
// m - mass of vehicle in kg

// full wheel radius in meters
const double W_r = 0.4572;
// gear ratio
const double G_r = 4.4;
// front track in meters
const double t_f = 1.3208;
// rear track in meters
const double t_r = 1.3208;

/*
    calculate desired yaw, rad/sec
    arguments, 
        steering angle in radians S, velocity V_cg
     
*/ 
double getDesiredYaw(double steerAngle, double vel_cg) {
    //calculate desired yaw from formula
    double desired_yaw_rate = (vel_cg / ((l_f + l_r) + K_u * pow(vel_cg, 2))) * steerAngle;
    return desired_yaw_rate;
}

/*

    Calculate delta torque 
    arguments,
        desired yaw,
        ,

*/

/*
    Calculate wheel velocities and sideSlips
    arguments,
        V_cg - velocity at center of gravity,
        yaw_rate - yaw rate around center of gravity
    return,
        vector - (V_fl, V_fr, V_rl, V_rr, Sr_fl, Sr_fr, Sr_rl, Sr_rr)
*/
double* calculateWheelVelocities(double* wheel_attr, double yaw_rate, double steer_angle){
    double V_fl = wheel_attr[0];
    double V_fr = wheel_attr[1];
    double V_rl = wheel_attr[2];
    double V_rr = wheel_attr[3];

    // slip ratios
/*
    double Sr_fl = wheel_attr[4];
    double Sr_fr = wheel_attr[5];
    double Sr_rl = wheel_attr[6];
    double Sr_rr = wheel_attr[7];
*/
    double Sr_fl = 0;
    double Sr_fr = 0;
    double Sr_rl = 0;
    double Sr_rr = 0;

    // side slip angles
    double Sa_fl = 0;
    double Sa_fr = 0;
    double Sa_rl = 0;
    double Sa_rr = 0;


    double V_x = 0.0;
    double V_y = 0.0;

    double R_fl = 0.0;
    double R_fr = 0.0;
    double R_rl = 0.0;
    double R_rr = 0.0;

    double e_fl = 0.0;
    double e_fr = 0.0;
    double e_rl = 0.0;
    double e_rr = 0.0;

    double slong_fl = 0;
    double slong_fr = 0;
    double slong_rl = 0;
    double slong_rr = 0;

    double slat_fl = 0;
    double slat_fr = 0;
    double slat_rl = 0;
    double slat_rr = 0;

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

    wheel_attr[0] = V_fl;
    wheel_attr[1] = V_fr;
    wheel_attr[2] = V_rl;
    wheel_attr[3] = V_rr;


    // calc wheel side slip angles
    Sa_fl = steer_angle - (atan((V_y + (yaw_rate * R_fl * cos(e_fl)) ) / (V_x - (yaw_rate * R_fl * sin(e_fl)) )));
    Sa_fr = steer_angle - (atan((V_y + (yaw_rate * R_fr * sin(e_fr)) ) / (V_x + (yaw_rate * R_fr * cos(e_fr)) )));
    Sa_rl = steer_angle - (atan((V_y - (yaw_rate * R_rl * sin(e_rl)) ) / (V_x - (yaw_rate * R_rl * cos(e_rl)) )));
    Sa_rr = steer_angle - (atan((V_y - (yaw_rate * R_rr * sin(e_rr)) ) / (V_x + (yaw_rate * R_rr * sin(e_rr)) )));

    /*

    wheel_attr[4] = Sa_fl;
    wheel_attr[5] = Sa_fr;
    wheel_attr[6] = Sa_rl;
    wheel_attr[7] = Sa_rr;

    */


    // linear velocities
    double linv_fl = V_fl * W_r;
    double linv_fr = V_fr * W_r;
    double linv_rl = V_rl * W_r;
    double linv_rr = V_rr * W_r;

    // calculate long and lat slip
    slong_fl = (V_fl * W_r * cos(Sa_fl) -  linv_fl) / (V_fl * W_r * cos(Sa_fl) );
    slong_fr = (V_fr * W_r * cos(Sa_fr) -  linv_fr) / (V_fr * W_r * cos(Sa_fr) );
    slong_rl = (V_rl * W_r * cos(Sa_rl) -  linv_rl) / (V_rl * W_r * cos(Sa_rl) );
    slong_rr = (V_rr * W_r * cos(Sa_rr) -  linv_rr) / (V_rr * W_r * cos(Sa_rr) );
   

    slat_fl = tan(Sa_fl);
    slat_fr = tan(Sa_fr);
    slat_rl = tan(Sa_rl);
    slat_rr = tan(Sa_rr);


    Sr_fl = sqrt(pow(slong_fl, 2) + pow(slat_fl, 2));
    Sr_fr = sqrt(pow(slong_fr, 2) + pow(slat_fr, 2));
    Sr_rl = sqrt(pow(slong_rl, 2) + pow(slat_rl, 2));
    Sr_rr = sqrt(pow(slong_rr, 2) + pow(slat_rr, 2));
    
    wheel_attr[4] = Sr_fl;
    wheel_attr[5] = Sr_fr;
    wheel_attr[6] = Sr_rl;
    wheel_attr[7] = Sr_rr;
    

    return wheel_attr;
}

/*
    Calculate slip ratio
    arguments,
        wheel velocities, V_fl, V_fr, V_rl, Vrr

    
    output, 
        slip ratio [], [Sr_fl, Sr_fr, Sr_rl, Sr_rr]
*/
/*
double* calculateSlipRatio(double* wheel_vels, double* sideSlipAngles) {

    double slong_fl = 0;
    double slong_fr = 0;
    double slong_rl = 0;
    double slong_rr = 0;

    double slat_fl = 0;
    double slat_fr = 0;
    double slat_rl = 0;
    double slat_rr = 0;

    slong_fl = (wheel_vels[0] * W_r * cos(sideSlipAngles)  ) / (  )

}
*/


double calculateDeltaTorque(){
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
double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}