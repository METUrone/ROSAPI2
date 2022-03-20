#include "ros/ros.h"
#include "iki/konum_service.h"

float x,y,z;

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "client");
    ros::NodeHandle n;

    ros::ServiceClient pos_client = n.serviceClient<iki::konum_service>("/konum_alma");

    ros::Rate looprate(2);

    iki::konum_service srv_message;

    srv_message.request.isGlobal = true;

    while(ros::ok()){
        //response kısmı boş
        pos_client.call(srv_message);//servis çağırılıyor, response kısmı dolduruluyor
        //response kısmı dolu
        x = srv_message.response.x;
        y = srv_message.response.y;
        z = srv_message.response.z;

        ROS_INFO("%f %f %f", x,y,z);

        ros::spinOnce();
        looprate.sleep();
    }
}