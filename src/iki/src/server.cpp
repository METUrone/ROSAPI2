#include "ros/ros.h"
#include "iki/konum_service.h"

float local_x= 3.3,local_y=4.5,local_z = 0.3;
float global_x= 1.2,global_y=7.5,global_z = 9.2;

bool clbk(iki::konum_service::Request &request,
            iki::konum_service::Response &response){
    if(request.isGlobal){
        response.x = global_x;
        response.y = global_y;
        response.z = global_z;
    }else{
        response.x = local_x;
        response.y = local_y;
        response.z = local_z;
    }
    return true;
}

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("/konum_alma",clbk);

    ros::spin();
}