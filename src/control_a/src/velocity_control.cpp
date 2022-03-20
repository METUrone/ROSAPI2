#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "geometry_msgs/TwistStamped.h"
#include "control_a/hiz.h"
#include "control_a/takeoff_land.h"
#include "control_a/odom_srv.h"
#include "mavros_msgs/ParamSet.h"

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient param_set_client;

nav_msgs::Odometry odom;
geometry_msgs::TwistStamped vel_command;
mavros_msgs::State state;
bool onAir;
double takeoffHeight;

void pos_listener(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
}

void state_listener(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

bool vel_handler(control_a::hiz::Request& req,control_a::hiz::Response& res){
    vel_command = req.twist;
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
        bool ret = arming_client.call(srv_msg);
        ROS_INFO("%d %d %d",ret,srv_msg.response.result,srv_msg.response.success);

        mavros_msgs::ParamSet param_cmd;
        param_cmd.request.param_id = "MIS_TAKEOFF_ALT";
        param_cmd.request.value.real = req.pose.pose.position.z;
        param_set_client.call(param_cmd);

        // Mesajımı hazırlıyorum
        setmode_msg;
        setmode_msg.request.custom_mode = "AUTO.TAKEOFF";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);
        takeoffHeight = req.pose.pose.position.z;
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

    ros::Publisher vel_cmd_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",100);

    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 100,pos_listener);
    ros::Subscriber state_sub = n.subscribe("/mavros/state", 100,state_listener);

    ros::ServiceServer vel_server = n.advertiseService("/control/hiz",vel_handler);
    ros::ServiceServer tkoff_land_server = n.advertiseService("/control/takeoff_land",takeoff_land_handler);
    ros::ServiceServer odom_server = n.advertiseService("/control/odom",odom_handler);

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    param_set_client = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

    ros::Rate looprate(20);//Hz

    // Mavros bağlanana kadar bekle
    while((!state.connected) && ros::ok()){
        ros::spinOnce();
        looprate.sleep();
    }

    ROS_INFO("Connected!");

    while (ros::ok())
    {
        while((!onAir)&&ros::ok()){
            looprate.sleep();
            ros::spinOnce();
        }
    ROS_INFO("Connected!");

        while ((odom.pose.pose.position.z < (takeoffHeight - 0.5))&&ros::ok())
        {
            looprate.sleep();
            ros::spinOnce();
        }
    ROS_INFO("Connected!");

        for(int i = 0;(i < 20) && ros::ok();i++){
            vel_cmd_pub.publish(vel_command);
            looprate.sleep();
            ros::spinOnce();
        }

        // Mesajımı hazırlıyorum
        mavros_msgs::SetMode setmode_msg;
        setmode_msg.request.custom_mode = "OFFBOARD";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);

    ROS_INFO("Connected!");

        while(ros::ok()){
            ROS_INFO("%.2f %.2f %.2f %d %d %s", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,state.connected,state.armed,state.mode.c_str());
            vel_cmd_pub.publish(vel_command);

            if(!onAir){
                break;
            }

            ros::spinOnce();
            looprate.sleep();
        }
    }
}