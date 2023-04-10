// testing vehicle dynamics/torque vectoring library

#include"TorqueVectoring.h"

using namespace std;

const float steer_deg =  180.0;

int main() {

    float des_yaw = 0;

    float steering_angle = degreesToRadians(steer_deg);

    des_yaw = getDesiredYaw(steering_angle, V_cg);

    cout << "Desired Yaw: " << des_yaw << endl;

    // test wheel velocities func
    float w_vels[4] = {0, 0, 0, 0};
    float* vels_ptr = w_vels;
    vels_ptr = calculateWheelVelocities(vels_ptr, des_yaw, steering_angle);

    for(int i = 0; i < (sizeof(w_vels)/sizeof(w_vels[0])); i++) {
        cout << "Velocity of wheel " << i << ": " << vels_ptr[i] << endl;
    }

    return 0;
}