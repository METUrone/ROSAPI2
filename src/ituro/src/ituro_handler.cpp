#include "ros/ros.h"
#include "control_a/hiz.h"
#include "mavros_msgs/State.h"
#include "control_a/takeoff_land.h"
#include "nav_msgs/Odometry.h"
#include "ituro/cv_cizgi.h"
#include "math.h"
#include "vector"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <geographic_msgs/GeoPointStamped.h>

#ifdef __arm__
#include <wiringPi.h>
#define MZ80_PIN 4
#endif

tf2::Quaternion quat; 
#define MAXV 0.3
#define MAXANG 0.3

#define KP_X 0.5
#define KI_X 0.01
#define KD_X 0.1 //1

#define KP_T 0.5
#define KP_Y 0.0
#define KP_Z 4
#define KI_T 0.3
#define KI_Y 0.3
#define KI_Z 0.1
#define KD_T 0.1 //1
#define KD_Y 0.1 //1
#define KD_Z 0.06 //0.5

struct velocity{
    double x;
    double y;
    double z;
};

#define RADIUS 0.01//m

ros::ServiceClient hiz_client;

nav_msgs::Odometry odom;
mavros_msgs::State state;

ituro::cv_cizgi cizgi;
double yaw,p,r;
double desiredHeight = 1.5;

void state_listener(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

void pos_listener(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
    tf2::fromMsg(odom.pose.pose.orientation, quat);
    tf2::Matrix3x3 matrix(quat);
    matrix.getEulerYPR(yaw,p,r);
}

void cizgi_listener(const ituro::cv_cizgi::ConstPtr& msg){
    cizgi = *msg;
}

void giveVelocity(double x, double y, double z, double yaw_speed = 0.0){
    control_a::hiz vel_command;

    vel_command.request.twist.twist.linear.x = fmax(-MAXV,fmin(x,MAXV));
    vel_command.request.twist.twist.linear.y = fmax(-MAXV,fmin(y,MAXV));
    vel_command.request.twist.twist.linear.z = fmax(-MAXV,fmin(z,MAXV));
    vel_command.request.twist.twist.angular.z = fmax(-MAXANG,fmin(yaw_speed,MAXANG));

    hiz_client.call(vel_command);
}

void gotoposition(double x, double y, double z, double wait_time = 0.1, double timeout=5.0){
    double err_x,err_y,err_z;
    double last_err_x,last_err_y,last_err_z;
    double last_time = ros::Time::now().toSec();
    double dt,i_x,i_y,i_z,vx,vy,vz;
    double radius = 0.3;

    ros::Rate looprate(50);
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

        vx = KP_X*err_x + KD_X*(err_x - last_err_x)/dt + KI_X*i_x;
        vy = KP_Y*err_y + KD_Y*(err_y - last_err_y)/dt + KI_Y*i_y;
        vz = KP_Z*err_z + KD_Z*(err_z - last_err_z)/dt + KI_Z*i_z;

        giveVelocity(vx,vy,vz);
        last_time = ros::Time::now().toSec();
        last_err_x = err_x;
        last_err_y = err_y;
        last_err_z = err_z;


        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        looprate.sleep();
    }
    ros::Duration(wait_time).sleep();
}


void duck(){
    giveVelocity(0,0,0);
    desiredHeight = 0.5;
    gotoposition(odom.pose.pose.position.x,odom.pose.pose.position.y,desiredHeight);
    double gx = 0.5;
    gotoposition(odom.pose.pose.position.x + cos(yaw)*gx,odom.pose.pose.position.y + sin(yaw)*gx,desiredHeight);
    ros::Rate rate(50);
    while(ros::ok()){
        

        rate.sleep();
        ros::spinOnce();
    }

    desiredHeight = 0.5;
}

void follow_line(){
    double err_t,err_y,err_z;
    double last_err_t,last_err_y,last_err_z;
    double last_time = ros::Time::now().toSec();
    double mztime = ros::Time::now().toSec();
    double ascend_time = ros::Time::now().toSec();
    double go_time = ros::Time::now().toSec();
    double dt,i_t,i_y,i_z;
    double res_x = 0.1,res_y,res_z,res_t;
    bool seen,ducked,went;

    ros::Rate rate(50);

    while(ros::ok())
    {   
        #ifdef __arm__
        if(digitalRead(MZ80_PIN)){
            seen = false;
        }else{
            if(!seen){
                seen = true;
                mztime = ros::Time::now().toSec();
            }
        }
        if(ros::Time::now().toSec()-mztime > 0.1 && seen){
            seen = false;
            desiredHeight = 0.6;
            res_x = 0;
            ascend_time = ros::Time::now().toSec();
            ducked = true;
        }
        if(ros::Time::now().toSec()-ascend_time > 3 && ducked){
            ducked = false;
            res_x = 0.1;
            go_time = ros::Time::now().toSec();
            went = true;
        }
        if(ros::Time::now().toSec()-go_time > 15 && went){
            went = false;
            desiredHeight = 1.5;
        }

        #endif

        err_t = -cizgi.theta;
        err_y = -cizgi.y;
        err_z = desiredHeight - odom.pose.pose.position.z;

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

        if(!cizgi.x){
            res_x = -0.1;
            res_y = 0;
            res_z = 0;
            res_t = 0;
        }

        giveVelocity(cos(yaw)*res_x+sin(yaw)*res_y,-cos(yaw)*res_y+sin(yaw)*res_x,res_z,res_t);
        
        last_time = ros::Time::now().toSec();
        last_err_t = err_t;
        last_err_y = err_y;
        last_err_z = err_z;

        ros::spinOnce();
        ROS_INFO("%.2f %.2f %.2f", odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        rate.sleep();
    }
    ros::Duration(0.5).sleep();
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

    hiz_client = n.serviceClient<control_a::hiz>("/control/hiz");

    #ifdef __arm__
    wiringPiSetupGpio();
    pinMode(MZ80_PIN,INPUT);
    #endif

    geographic_msgs::GeoPointStamped gp_origin;
    gp_origin_pub.publish(gp_origin);

    ros::Rate looprate(20);

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

    follow_line();
}