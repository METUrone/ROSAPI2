#include "control_a/velocity_handler.hpp"

#include "math.h"
#include "vector"

struct velocity{
    double x;
    double y;
    double z;
};

std::vector<velocity> velocities;

ros::ServiceClient hiz_client;

UAVvel::UAVvel(ros::NodeHandle& _n){
    n = _n;
    odom_client = n.serviceClient<control_a::odom_srv>("/control/odom");
    takeoff_land_client = n.serviceClient<control_a::takeoff_land>("/control/takeoff_land");
    hiz_client = n.serviceClient<control_a::hiz>("/control/hiz");
}

nav_msgs::Odometry UAVvel::getPosition(){
    control_a::odom_srv srv_msg;
    odom_client.call(srv_msg);
    return srv_msg.response.odom;        
}

void UAVvel::giveVelocity(double x, double y, double z){
    control_a::hiz vel_command;

    vel_command.request.twist.twist.linear.x = x;
    vel_command.request.twist.twist.linear.y = y;
    vel_command.request.twist.twist.linear.z = z;

    hiz_client.call(vel_command);
}

void UAVvel::takeoff(double x, double y, double z, double yaw){
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
}

void UAVvel::land(){
    control_a::takeoff_land srv_msg;
    srv_msg.request.isTakeoff = false;
    takeoff_land_client.call(srv_msg);
}


void UAVvel::gotoposition(double x, double y, double z, double wait_time, double timeout){
    double err_x,err_y,err_z;
    double last_err_x,last_err_y,last_err_z;
    double last_time = ros::Time::now().toSec();
    double dt,i_x,i_y,i_z;

    ros::Rate looprate(50);
    nav_msgs::Odometry odom = getPosition();
    double timestamp = ros::Time::now().toSec();

    while(ros::ok()&&(ros::Time::now().toSec()-timestamp < timeout)&&!(
    (x-radius <odom.pose.pose.position.x&&odom.pose.pose.position.x< x+radius)&&
    (y-radius <odom.pose.pose.position.y&&odom.pose.pose.position.y< y+radius)&&
    (z-radius <odom.pose.pose.position.z&&odom.pose.pose.position.z< z+radius)))
    {
        err_x = x - odom.pose.pose.position.x;
        err_y = y - odom.pose.pose.position.y;
        err_z = z - odom.pose.pose.position.z;

        if((last_err_x<0&&err_x>0)||(last_err_x>0&&err_x<0)){
            i_x = 0;
        }
        if((last_err_y<0&&err_y>0)||(last_err_y>0&&err_y<0)){
            i_y = 0;
        }
        if((last_err_z<0&&err_z>0)||(last_err_z>0&&err_z<0)){
            i_z = 0;
        }
        dt = ros::Time::now().toSec()-last_time;
        i_x += err_x*dt;
        i_y += err_y*dt;
        i_z += err_z*dt;
        
        giveVelocity(kp_x*err_x + kd_x*(err_x - last_err_x)/dt + ki_x*i_x,kp_y*err_y + kd_y*(err_y - last_err_y)/dt + ki_y*i_y,kp_z*err_z + kd_z*(err_z - last_err_z)/dt + ki_z*i_z);

        last_time = ros::Time::now().toSec();
        last_err_x = err_x;
        last_err_y = err_y;
        last_err_z = err_z;


        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        odom = getPosition();
        looprate.sleep();
    }
    ros::Duration(wait_time).sleep();
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "handler");
    ros::NodeHandle n;

    UAVvel drone(n);

    velocities.reserve(10);
    velocities.push_back({0.0,0.0,5.0});
    velocities.push_back({3.0,4.0,5.0});
    velocities.push_back({5.0,6.0,5.0});
    velocities.push_back({3.0,4.0,3.0});
    velocities.push_back({3.0,3.0,3.0});
    velocities.push_back({3.0,3.0,4.0});
    velocities.push_back({5.0,4.0,5.0});
    velocities.push_back({5.0,10.0,5.0});
    velocities.push_back({3.0,3.0,5.0});
    velocities.push_back({0.0,0.0,5.0});

    drone.takeoff(0,0,3);
    ros::Duration(5.0).sleep();

    drone.gotoposition(0,0,5);

    drone.giveVelocity(0,0,0);
/*
    for(velocity &vel : velocities){
        drone.giveVelocity(vel.x,vel.y,vel.z);
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }
*/
}