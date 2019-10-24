#include <ros/ros.h>
#include <ros/console.h>
#include <hal_gnss/Gnss.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    if(argc < 2)
    {
        std::cout << "Specify can channel [can0 or can1]" << std::endl;
        return -1;
    }

    Gnss HydraBox(argv[1]);

    int iRet = HydraBox.configure();
    if(iRet == 0)
    {
        while(ros::ok())
        {
            //listen for frames, untill all data are fresh and ready:
            if(HydraBox.update())
            {
                //publish data
                ROS_INFO("[Hydrabox node] All data gathered!");
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else
    {
        ROS_ERROR("[Hydrabox] Configuration failed with error code %d", iRet);
    }
    return 0;
}
