#include "ros/ros.h"
#include "iki/yasuyormu.h"

bool yasama_durumu;
int years_left;

void clbk(const iki::yasuyormu::ConstPtr& msg){
    yasama_durumu = (*msg).truth;
    years_left = msg->kalan_omur; // -> üsteki kullanımla aynı anlama geliyor
    ROS_INFO("adana %d %d", yasama_durumu,years_left);
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/yasiyor_muyus",100,clbk);

    ros::spin();
}