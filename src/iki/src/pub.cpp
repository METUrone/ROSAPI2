#include "ros/ros.h"
#include <iki/yasuyormu.h>

int main(int argc, char **argv)
{   // Ros için gerekli olan şeyler
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    // Topic oluştur ve mesaj tipini belirle
    ros::Publisher pub = n.advertise<iki::yasuyormu>("/yasiyor_muyus",100);
    // Aynı tipte bir mesaj oluştur ve içini doldur
    iki::yasuyormu msg;
    msg.truth = true;
    msg.kalan_omur = 45;
    
    ros::Rate looprate(5); // 5 Hz

    while(ros::ok()){
        //Mesajı gönder
        pub.publish(msg);
        //ros için her döngünün sonunda gerekli
        ros::spinOnce();
        //5 Hz için bekle
        looprate.sleep();
    }
}