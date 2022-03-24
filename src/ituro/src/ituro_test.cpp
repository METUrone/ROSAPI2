#include "ros/ros.h"
#include "control_a/hiz.h"
#include "control_a/konum.h"
#include "control_a/takeoff_land.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"
#include "ituro/cv_cizgi.h"
#include "math.h"
#include "vector"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <geographic_msgs/GeoPointStamped.h>

tf2::Quaternion quat; 

#define KP_T 3
#define KP_Y 3
#define KP_Z 4
#define KI_T 0.3
#define KI_Y 0.3
#define KI_Z 0.1
#define KD_T 1.5 //1
#define KD_Y 1.5 //1
#define KD_Z 0.8 //0.5

struct velocity{
    double x;
    double y;
    double z;
};

#define RADIUS 0.01//m

ros::ServiceClient pos_client;
ros::ServiceClient hiz_client;

nav_msgs::Odometry odom;
mavros_msgs::State state;

ituro::cv_cizgi cizgi;


void state_listener(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}


void pos_listener(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
}

void cizgi_listener(const ituro::cv_cizgi::ConstPtr& msg){
    cizgi = *msg;
}

void giveVelocity(double x, double y, double z, double yaw_speed = 0.0){
    control_a::hiz vel_command;

    vel_command.request.twist.twist.linear.x = x;
    vel_command.request.twist.twist.linear.y = y;
    vel_command.request.twist.twist.linear.z = z;
    vel_command.request.twist.twist.angular.z = yaw_speed;

    hiz_client.call(vel_command);
}

void follow_line(){
    double err_t,err_y,err_z;
    double last_err_t,last_err_y,last_err_z;
    double last_time = ros::Time::now().toSec();
    double dt,i_t,i_y,i_z;
    double res_x = 0.1,res_y,res_z,res_t;
    double yaw,p,r;

    ros::Rate looprate(50);

    while(ros::ok())
    {
        tf2::fromMsg(odom.pose.pose.orientation, quat);
        tf2::Matrix3x3 matrix(quat);
        matrix.getEulerYPR(yaw,p,r);

        err_t = cizgi.theta;
        err_y = cizgi.y;
        err_z = 2 - odom.pose.pose.position.z;

        if((last_err_t<0&&err_t>0)||(last_err_t>0&&err_t<0)){
            i_t = 0;
        }
        if((last_err_y<0&&err_y>0)||(last_err_y>0&&err_y<0)){
            i_y = 0;
        }
        if((last_err_z<0&&err_z>0)||(last_err_z>0&&err_z<0)){
            i_z = 0;
        }
        dt = ros::Time::now().toSec()-last_time;
        i_t += err_t*dt;
        i_y += err_y*dt;
        i_z += err_z*dt;

        res_y = KP_Y*err_y + KD_Y*(err_y - last_err_y)/dt + KI_Y*i_y;
        res_z = KP_Z*err_z + KD_Z*(err_z - last_err_z)/dt + KI_Z*i_z;
        res_t = KP_T*err_t + KD_T*(err_t - last_err_t)/dt + KI_T*i_t;
        giveVelocity(cos(yaw)*res_x-sin(yaw)*res_y,cos(yaw)*res_y+sin(yaw)*res_x,res_z,res_t);
        
        last_time = ros::Time::now().toSec();
        last_err_t = err_t;
        last_err_y = err_y;
        last_err_z = err_z;

        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        looprate.sleep();
    }
    ros::Duration(0.5).sleep();
}

void gotoPos(double x, double y, double z){
    control_a::konum pos;
    pos.request.pose.pose.position.x = x;
    pos.request.pose.pose.position.y = y;
    pos.request.pose.pose.position.z = z;
    pos_client.call(pos);
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "handler");
    ros::NodeHandle n;

    ros::Publisher gp_origin_pub = n.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin",10);
    
    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 50,pos_listener);

    ros::Subscriber state_sub = n.subscribe("/mavros/state", 100,state_listener);

    ros::Subscriber cizgi_sub = n.subscribe("/ituro/cizgi", 50,cizgi_listener);

    ros::ServiceClient tkoff_land = n.serviceClient<control_a::takeoff_land>("/control/takeoff_land");

    pos_client = n.serviceClient<control_a::konum>("/control/konum");

    ros::Rate looprate(20);

    // Mavros bağlanana kadar bekle
    while((!state.connected) && ros::ok()){
        ros::spinOnce();
        looprate.sleep();
    }

    ROS_INFO("Connected!");

    geographic_msgs::GeoPointStamped gp_origin;
    gp_origin_pub.publish(gp_origin);

    while ((state.mode.compare("GUIDED")) && ros::ok())
    {
        ros::spinOnce();
        looprate.sleep();
    }
    
    ros::Duration(1.0).sleep();

    control_a::takeoff_land tkoff_msg;
    tkoff_msg.request.isTakeoff = true;
    tkoff_msg.request.pose.pose.position.z = 1.5;
    tkoff_land.call(tkoff_msg);
    
    ros::Duration(10.0).sleep();

    gotoPos(1,0,1.5);

    ros::Duration(10.0).sleep();

    gotoPos(1,1,1.5);
    
    ros::Duration(10.0).sleep();

    gotoPos(0,1,1.5);

    ros::Duration(10.0).sleep();

    gotoPos(0,0,1.5);

    ros::Duration(10.0).sleep();

    tkoff_msg.request.isTakeoff = false;
    tkoff_land.call(tkoff_msg);
}