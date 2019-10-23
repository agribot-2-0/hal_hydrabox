#include <ros/ros.h>
#include <ros/console.h>
#include <hal_gnss/Gnss.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}