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
#include "mavros_msgs/CommandTOL.h"


ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient param_set_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient land_client;
ros::Publisher pose_cmd_pub;

nav_msgs::Odometry odom;
geometry_msgs::PoseStamped pose_command;
mavros_msgs::State state;
bool onAir;

void pos_listener(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
}

void state_listener(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

bool pos_handler(control_a::konum::Request& req,control_a::konum::Response& res){
    pose_command = req.pose;
    pose_cmd_pub.publish(pose_command);
    return true;
}

bool odom_handler(control_a::odom_srv::Request& req,control_a::odom_srv::Response& res){
    res.odom = odom;
    return true;
}

bool takeoff_land_handler(control_a::takeoff_land::Request& req,control_a::takeoff_land::Response& res){
    mavros_msgs::SetMode setmode_msg;
    mavros_msgs::CommandTOL TOL_msg;
    if(req.isTakeoff){//TAKEOFF
        // Mesajımı hazırlıyorum
        setmode_msg.request.custom_mode = "GUIDED";
        // Servisi çağırıp mesajı gönderdim
        set_mode_client.call(setmode_msg);
        
        //ros::Duration(1).sleep();

        // Mesajımı hazırlıyorum
        mavros_msgs::CommandBool srv_msg;
        srv_msg.request.value = true;
        // Servisi çağırıp mesajı gönderdim
        arming_client.call(srv_msg);

        //ros::Duration(1).sleep();

        TOL_msg.request.altitude = req.pose.pose.position.z;
        ROS_INFO("%f",TOL_msg.request.altitude);
        takeoff_client.call(TOL_msg);

        pose_command = req.pose;
        onAir = true;
    }else{//LAND
        TOL_msg.request.altitude = 0.0;
        land_client.call(TOL_msg);
        onAir = false;
    }
    return true;
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    pose_cmd_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);

    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 100,pos_listener);
    ros::Subscriber state_sub = n.subscribe("/mavros/state", 100,state_listener);

    ros::ServiceServer pos_server = n.advertiseService("/control/konum",pos_handler);
    ros::ServiceServer tkoff_land_server = n.advertiseService("/control/takeoff_land",takeoff_land_handler);
    ros::ServiceServer odom_server = n.advertiseService("/control/odom",odom_handler);

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    param_set_client = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    ros::Rate looprate(20);//Hz

    // Mavros bağlanana kadar bekle
    while((!state.connected) && ros::ok()){
        ros::spinOnce();
        looprate.sleep();
    }

    ROS_INFO("Connected!");

    while(ros::ok()){
        ROS_INFO("%.2f %.2f %.2f %d %d %s", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,state.connected,state.armed,state.mode.c_str());
        
        ros::spinOnce();
        looprate.sleep();
    }
}