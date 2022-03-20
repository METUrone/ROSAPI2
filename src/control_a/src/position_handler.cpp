#include "control_a/position_handler.hpp"

#include "math.h"
#include "vector"

UAVpos::UAVpos(ros::NodeHandle& _n){
    n = _n;
    konum_client = n.serviceClient<control_a::konum>("/control/konum");
    takeoff_land_client = n.serviceClient<control_a::takeoff_land>("/control/takeoff_land");
    odom_client = n.serviceClient<control_a::odom_srv>("/control/odom");

}

nav_msgs::Odometry UAVpos::getPosition(){
    control_a::odom_srv srv_msg;
    odom_client.call(srv_msg);
    return srv_msg.response.odom;        
}

void UAVpos::takeoff(double x, double y, double z, double yaw, double timeout){
    control_a::takeoff_land srv_msg;
    srv_msg.request.isTakeoff = true;
    srv_msg.request.pose.pose.position.x = x;
    srv_msg.request.pose.pose.position.y = y;
    srv_msg.request.pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    q = q.normalize();
    srv_msg.request.pose.pose.orientation.w = q.w();
    srv_msg.request.pose.pose.orientation.x = q.x();
    srv_msg.request.pose.pose.orientation.y = q.y();
    srv_msg.request.pose.pose.orientation.z = q.z();
    takeoff_land_client.call(srv_msg);

    ros::Rate looprate(40);
    nav_msgs::Odometry odom = getPosition();
    double timestamp = ros::Time::now().toSec();
    while(ros::ok()&&(ros::Time::now().toSec()-timestamp < timeout)&&!
    (z-radius <odom.pose.pose.position.z&&odom.pose.pose.position.z< z+radius))
    {
        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        odom = getPosition();
        looprate.sleep();
    }
}

void UAVpos::land(){
    control_a::takeoff_land srv_msg;
    srv_msg.request.isTakeoff = false;
    takeoff_land_client.call(srv_msg);
}

void UAVpos::gotoposition(double x, double y, double z, double wait_time, double timeout){
    control_a::konum pos_command;

    pos_command.request.pose.pose.position.x = x;
    pos_command.request.pose.pose.position.y = y;
    pos_command.request.pose.pose.position.z = z;
    konum_client.call(pos_command);

    ros::Rate looprate(40);
    nav_msgs::Odometry odom = getPosition();
    double timestamp = ros::Time::now().toSec();

    while(ros::ok()&&(ros::Time::now().toSec()-timestamp < timeout)&&!(
    (x-radius <odom.pose.pose.position.x&&odom.pose.pose.position.x< x+radius)&&
    (y-radius <odom.pose.pose.position.y&&odom.pose.pose.position.y< y+radius)&&
    (z-radius <odom.pose.pose.position.z&&odom.pose.pose.position.z< z+radius)))
    {
        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        odom = getPosition();
        looprate.sleep();
    }
    ros::Duration(wait_time).sleep();
}

struct position{
    double x;
    double y;
    double z;
};

std::vector<position> positions;

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "handler");
    ros::NodeHandle n;

    UAVpos drone(n);

    positions.reserve(10);
    positions.push_back({0.0,0.0,5.0});
    positions.push_back({3.0,4.0,5.0});
    positions.push_back({5.0,6.0,5.0});
    positions.push_back({3.0,4.0,3.0});
    positions.push_back({3.0,3.0,3.0});
    positions.push_back({3.0,3.0,4.0});
    positions.push_back({5.0,4.0,5.0});
    positions.push_back({5.0,10.0,5.0});
    positions.push_back({3.0,3.0,5.0});
    positions.push_back({0.0,0.0,5.0});

    drone.takeoff(0,0,5);
    
    for(position &pos : positions){
        drone.gotoposition(pos.x,pos.y,pos.z);
    }
}