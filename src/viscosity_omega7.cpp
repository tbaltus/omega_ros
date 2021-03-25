// C++ library headers
#include <iostream>
#include <iomanip>
// project headers
#include "omega_ros/dhdc.h"
#include "ros/ros.h"
// constants
constexpr double B = 30.0;
constexpr double K = 100.0;

////////////////////////////////////////////////////////////////////////////////
int main(int argc,
         char* argv[])
{
    // open a connection to the device
    if (dhdOpen() < 0)
    {
        std::cout << "error: cannot open device" << std::endl;
        return -1;
    }

    dhdSetBrakes(DHD_OFF);
    dhdEnableForce(DHD_ON);

    // run haptic loop
    int done = 0;
    while (1)
    {
        // get end-effector position
        double vx, vy, vz;
        double px, py, pz;
        dhdGetLinearVelocity(&vx, &vy, &vz);
        dhdGetPosition(&px, &py, &pz);
        // compute spring model
        double fx, fy, fz;
        fx = -B * vx - K * px;
        fy = -B * vy - K * py;
        fz = -B * vz - K * pz;
        ROS_INFO("fx : %f, fy : %f, fz : %f", fx, fy, fz);
        // apply forces
        dhdSetForce(fx, fy, fz);
        // exit if the button is pushed
        done += dhdGetButton(0);
    }
    // close the connection
    dhdClose();
    return 0;
}
