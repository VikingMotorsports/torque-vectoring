// testing vehicle dynamics/torque vectoring library

#include"TorqueVectoring.h"

using namespace std;

const double steer_deg =  180.0;

int main() {

    double des_yaw = 0;

    double steering_angle = degreesToRadians(steer_deg);

    des_yaw = getDesiredYaw(steering_angle, V_cg);

    cout << "Desired Yaw: " << des_yaw << endl;

    // test wheel velocities func
    double w_vels[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double* vels_ptr = w_vels;
    vels_ptr = calculateWheelVelocities(vels_ptr, des_yaw, steering_angle);

    for(int i = 0; i < 4; i++) {
        cout << "Velocity of wheel " << i << ": " << vels_ptr[i] << endl;
    }

    for(int i = 0; i < 4; i++) {
        int index = 4 + i;
        cout << "Slip ratio of wheel " << i << ": " << vels_ptr[index] << endl;
    }


    return 0;
}