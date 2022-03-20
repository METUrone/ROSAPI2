#include "ros/ros.h"
#include "control_a/hiz.h"
#include "control_a/takeoff_land.h"
#include "control_a/odom_srv.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>

#define KP_X 3
#define KP_Y 3
#define KP_Z 4
#define KI_X 0.3
#define KI_Y 0.3
#define KI_Z 0.1
#define KD_X 1.5 //1
#define KD_Y 1.5 //1
#define KD_Z 0.8 //0.5

class UAVvel{
    ros::NodeHandle n;
    ros::ServiceClient hiz_client,takeoff_land_client,odom_client;
    public:
    double radius = 0.1;
    double kp_x = KP_X,kp_y = KP_Y,kp_z = KP_Z,ki_x = KI_X,ki_y = KI_Y,ki_z = KI_Z,kd_x = KD_X,kd_y = KD_Y,kd_z = KD_Z;

    UAVvel(ros::NodeHandle& _n);

    nav_msgs::Odometry getPosition();
    void giveVelocity(double x, double y, double z);
    void takeoff(double x, double y, double z, double yaw = 0.0);
    void land();
    void gotoposition(double x, double y, double z, double wait_time = 0.5, double timeout = 5.0);
};