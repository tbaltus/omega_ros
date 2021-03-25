// C++ library headers
#include <iostream>
#include <iomanip>
// project headers
#include "omega_ros/dhdc.h"
#include "ros/ros.h"

////////////////////////////////////////////////////////////////////////////////
int main(int argc,
         char* argv[])
{
    double x_err = 0.0;
    double x_prev_err = 0.0;

    double y_err = 0.0;
    double y_prev_err = 0.0;

    double z_err = 0.0;
    double z_prev_err = 0.0;

    double x_target = 0.0;
    double y_target = 0.0;
    double z_target = 0.0;

    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;

    double P = 30;
    double D = 10;
    double I = 35;

    double loop_freq = 200;

    ros::init(argc, argv, "PID_at_pose_omega7");
    ros::NodeHandle n;

    // open a connection to the device
    if (dhdOpen() < 0)
    {
        std::cout << "error: cannot open device" << std::endl;
        return -1;
    }

    dhdSetBrakes(DHD_OFF);
    dhdEnableForce(DHD_ON);


    ros::Rate loop_rate(loop_freq);
    
    // run haptic loop
    int done = 0;
    while (ros::ok())
    {
        // get end-effector position
        double px, py, pz;
        dhdGetPosition(&px, &py, &pz);

        // compute error
        x_err = x_target - px;
        y_err = y_target - py;
        z_err = z_target - pz;

        // compute integral
        x_sum += x_err * (1/loop_freq);
        y_sum += y_err * (1/loop_freq);
        z_sum += z_err * (1/loop_freq);


        // compute spring model
        double fx, fy, fz;
        fx = P * x_err + D * (x_err - x_prev_err)/(1/loop_freq) + I * x_sum;
        fy = P * y_err + D * (y_err - y_prev_err)/(1/loop_freq) + I * y_sum;
        fz = P * z_err + D * (z_err - z_prev_err)/(1/loop_freq) + I * z_sum;

        ROS_INFO("fx : %f, fy : %f, fz : %f", fx, fy, fz);
        // apply forces
        dhdSetForce(fx, fy, fz);

        //compute x_prev_err
        x_prev_err = x_err;
        y_prev_err = y_err;
        z_prev_err = z_err;

        // exit if the button is pushed
        done += dhdGetButton(0);

        loop_rate.sleep();
    }
    // close the connection
    dhdClose();
    return 0;
}
