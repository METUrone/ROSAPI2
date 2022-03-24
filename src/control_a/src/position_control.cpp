#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_a/konum.h"
#include "control_a/takeoff_land.h"
#include "control_a/odom_srv.h"
#include "mavros_msgs/ParamSet.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient param_set_client;

tf2::Quaternion quat; 
nav_msgs::Odometry odom;
geometry_msgs::PoseStamped pose_command;
mavros_msgs::State state;
bool onAir;
double yaw,p,r;

void pos_listener(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
    tf2::fromMsg(odom.pose.pose.orientation, quat);
    tf2::Matrix3x3 matrix(quat);
    matrix.getEulerYPR(yaw,p,r);
    ROS_INFO("%.2f",yaw*180/M_PI);
}

void state_listener(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

bool pos_handler(control_a::konum::Request& req,control_a::konum::Response& res){
    pose_command = req.pose;
    return true;
}

bool odom_handler(control_a::odom_srv::Request& req,control_a::odom_srv::Response& res){
    res.odom = odom;
    return true;
}

bool takeoff_land_handler(control_a::takeoff_land::Request& req,control_a::takeoff_land::Response& res){
    mavros_msgs::SetMode setmode_msg;
    if(req.isTakeoff){
        // Mesajımı hazırlıyorum
        mavros_msgs::CommandBool srv_msg;
        srv_msg.request.value = true;
        // Servisi çağırıp mesajı gönderdim
        arming_client.call(srv_msg);

        mavros_msgs::ParamSet param_cmd;
        param_cmd.request.param_id = "MIS_TAKEOFF_ALT";
        param_cmd.request.value.real = req.pose.pose.position.z;
        param_set_client.call(param_cmd);

        // Mesajımı hazırlıyorum
        setmode_msg;
        setmode_msg.request.custom_mode = "AUTO.TAKEOFF";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);
        pose_command = req.pose;
        onAir = true;
    }else{
        // Mesajımı hazırlıyorum
        setmode_msg;
        setmode_msg.request.custom_mode = "AUTO.LAND";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);
        onAir = false;
    }
    return true;
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    ros::Publisher pose_cmd_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);

    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 100,pos_listener);
    ros::Subscriber state_sub = n.subscribe("/mavros/state", 100,state_listener);

    ros::ServiceServer pos_server = n.advertiseService("/control/konum",pos_handler);
    ros::ServiceServer tkoff_land_server = n.advertiseService("/control/takeoff_land",takeoff_land_handler);
    ros::ServiceServer odom_server = n.advertiseService("/control/odom",odom_handler);

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    param_set_client = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate looprate(20);//Hz

    // Mavros bağlanana kadar bekle
    while((!state.connected) && ros::ok()){
        ros::spinOnce();
        looprate.sleep();
    }

    ROS_INFO("Connected!");

    while(ros::ok()){

        while((!onAir)&&ros::ok()){
            looprate.sleep();
            ros::spinOnce();
        }

        while ((odom.pose.pose.position.z < (pose_command.pose.position.z - 0.5))&&ros::ok())
        {
            looprate.sleep();
            ros::spinOnce();
        }

        for(int i = 0;(i < 20) && ros::ok();i++){
            pose_cmd_pub.publish(pose_command);
            looprate.sleep();
            ros::spinOnce();
        }

        // Mesajımı hazırlıyorum
        mavros_msgs::SetMode setmode_msg;
        setmode_msg.request.custom_mode = "OFFBOARD";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);


        while(ros::ok()){
            ROS_INFO("%.2f %.2f %.2f %d %d %s", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,state.connected,state.armed,state.mode.c_str());
            pose_cmd_pub.publish(pose_command);

            if(!onAir){
                break;
            }

            ros::spinOnce();
            looprate.sleep();
        }
    }
}