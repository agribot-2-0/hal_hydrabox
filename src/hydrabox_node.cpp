#include <ros/ros.h>
#include <ros/console.h>
#include <hal_hydrabox/Hydrabox.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("agribot/imu",1);
    ros::Publisher magnetometer_pub = n.advertise<sensor_msgs::MagneticField>("agribot/magnetic_field",1);
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField magnetometer_msg;

    if(argc < 2)
    {
        std::cout << "Specify can channel [can0 or can1]" << std::endl;
        return -1;
    }

    Hydrabox Hydra(argv[1]);

    int iRet = Hydra.configure();
    if(iRet == 0)
    {
        while(ros::ok())
        {
            //listen for frames, untill all data are fresh and ready:
            if(Hydra.update())
            {
                if((Hydra.u16GetDataReadiness()&FIRST_DATASET_READY) == FIRST_DATASET_READY)
                {
                    ROS_INFO("[Hydrabox node] First dataset ready to publish");
                    Hydra.vClearFirstDataset();
                }
                if((Hydra.u16GetDataReadiness()&SECOND_DATASET_READY) == SECOND_DATASET_READY)
                {
                    ROS_INFO("[Hydrabox node] Second dataset ready to publish");
                    Hydra.vClearSecondDataset();
                }
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
