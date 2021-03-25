#include "ros/ros.h"
#include "std_msgs/String.h"
#include "omega_ros/dhdc.h"
#include "omega_ros/drdc.h"
#include "std_msgs/Int8MultiArray.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>

void displayDeviceStatus(char ID);
void displayCartesianPosition(double *cartesian_position);
void displayCartesianVelocity(double *cartesian_velocity);
void deviceForceCallback(const geometry_msgs::Vector3::ConstPtr &msg);

int main(int argc, char **argv)
{
  // Declare variables
  double device_cartesian_position[3];
  double device_cartesian_velocity[3];
  
  // Init node
  ros::init(argc, argv, "omega7");
  ros::NodeHandle n;

  // Declare publishers
  ros::Publisher cartesian_position_device_pub;
  cartesian_position_device_pub = n.advertise<geometry_msgs::Vector3Stamped>("/EE_cartesian_position",1);

  ros::Publisher cartesian_velocity_device_pub;
  cartesian_velocity_device_pub = n.advertise<geometry_msgs::Vector3Stamped>("/EE_cartesian_velocity_device",1);

  // Declare subscribers
  ros::Subscriber cartesian_force_device_sub;
  cartesian_force_device_sub = n.subscribe<geometry_msgs::Vector3>("/EE_cartesian_force_device",1, deviceForceCallback);

  // open a connection to the device
  if (dhdOpen() < 0)
  {
    std::cout << "error: cannot open device" << std::endl;
    return -1;
  }

  //dhdWaitForReset(10000); 

  // Disable brakes and set device in force mode
  dhdSetBrakes(DHD_OFF);
  dhdEnableForce(DHD_ON);

  // Display Status
  displayDeviceStatus(dhdGetDeviceID());

  //Set zero force
  dhdSetForce(0, 0, 0);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    //Get device cartesian positions and cartesian velocities
    dhdGetPosition(&device_cartesian_position[0], &device_cartesian_position[1], &device_cartesian_position[2], dhdGetDeviceID());
    dhdGetLinearVelocity(&device_cartesian_velocity[0], &device_cartesian_velocity[1], &device_cartesian_velocity[2], dhdGetDeviceID());

    //Display cartesian positions and cartesian velocities
    //displayCartesianPosition(&device_cartesian_position[0]);
    //displayCartesianVelocity(&device_cartesian_velocity[0]);
    
    //Publishing device cartesian positions and cartesian velocities
    geometry_msgs::Vector3Stamped cart_pos_dev;
    geometry_msgs::Vector3Stamped cart_vel_dev;

    cart_pos_dev.vector.x = device_cartesian_position[0];
    cart_pos_dev.vector.y = device_cartesian_position[1];
    cart_pos_dev.vector.z = device_cartesian_position[2];

    cart_vel_dev.vector.x = device_cartesian_velocity[0];
    cart_vel_dev.vector.y = device_cartesian_velocity[1];
    cart_vel_dev.vector.z = device_cartesian_velocity[2];

    cartesian_position_device_pub.publish(cart_pos_dev);
    cartesian_velocity_device_pub.publish(cart_vel_dev);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

void displayDeviceStatus(char ID)
{
    int status[DHD_MAX_STATUS];
    dhdGetStatus(status, ID);
    ROS_INFO("========================");
    ROS_INFO("Status");
    ROS_INFO("%d : DHD_STATUS_POWER", status[DHD_STATUS_POWER]);
    ROS_INFO("%d : DHD_STATUS_CONNECTED", status[DHD_STATUS_CONNECTED]);
    ROS_INFO("%d : DHD_STATUS_STARTED", status[DHD_STATUS_STARTED]);
    ROS_INFO("%d : DHD_STATUS_RESET", status[DHD_STATUS_RESET]);
    ROS_INFO("%d : DHD_STATUS_IDLE", status[DHD_STATUS_IDLE]);
    ROS_INFO("%d : DHD_STATUS_FORCE", status[DHD_STATUS_FORCE]);
    ROS_INFO("%d : DHD_STATUS_BRAKE", status[DHD_STATUS_BRAKE]);
    ROS_INFO("%d : DHD_STATUS_TORQUE", status[DHD_STATUS_TORQUE]);
    ROS_INFO("%d : DHD_STATUS_WRIST_DETECTED", status[DHD_STATUS_WRIST_DETECTED]);
    ROS_INFO("%d : DHD_STATUS_ERROR", status[DHD_STATUS_ERROR]);
    ROS_INFO("%d : DHD_STATUS_GRAVITY", status[DHD_STATUS_GRAVITY]);
    ROS_INFO("%d : DHD_STATUS_TIMEGUARD", status[DHD_STATUS_TIMEGUARD]);
    ROS_INFO("%d : DHD_STATUS_WRIST_INIT", status[DHD_STATUS_WRIST_INIT]);
    ROS_INFO("%d : DHD_STATUS_REDUNDANCY", status[DHD_STATUS_REDUNDANCY]);
    ROS_INFO("%d : DHD_STATUS_FORCEOFFCAUSE", status[DHD_STATUS_FORCEOFFCAUSE]);
    ROS_INFO("%d : DHD_STATUS_LOCKS", status[DHD_STATUS_LOCKS]);
    ROS_INFO("%d : DHD_STATUS_AXISCHECKED", status[DHD_STATUS_AXISCHECKED]);
    ROS_INFO("========================");
}

void displayCartesianPosition(double *cartesian_position)
{
  ROS_INFO("x = %f", cartesian_position[0]);
  ROS_INFO("y = %f", cartesian_position[1]);
  ROS_INFO("z = %f", cartesian_position[2]);
}

void displayCartesianVelocity(double *cartesian_velocity)
{
  ROS_INFO("vx = %f", cartesian_velocity[0]);
  ROS_INFO("vy = %f", cartesian_velocity[1]);
  ROS_INFO("vz = %f", cartesian_velocity[2]); 
}

void deviceForceCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  dhdSetForce(msg->x, msg->y, msg->z);
}