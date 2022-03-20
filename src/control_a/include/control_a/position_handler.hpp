#include "ros/ros.h"
#include "control_a/konum.h"
#include "control_a/takeoff_land.h"
#include "control_a/odom_srv.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>

class UAVpos{
    ros::NodeHandle n;
    ros::ServiceClient konum_client,takeoff_land_client,odom_client;
    public:
    double radius = 1.5;

    UAVpos(ros::NodeHandle& _n);

    nav_msgs::Odometry getPosition();
    void takeoff(double x, double y, double z, double yaw = 0.0, double timeout = 10.0);
    void land();
    void gotoposition(double x, double y, double z, double wait_time = 0.5, double timeout = 5.0);
};