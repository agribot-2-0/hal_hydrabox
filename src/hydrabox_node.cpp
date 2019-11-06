#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hal_hydrabox/Hydrabox.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <agribot_msgs/AgribotHeading.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("agribot/imu",1);
    ros::Publisher magnetometer_pub = n.advertise<sensor_msgs::MagneticField>("agribot/magnetic_field",1);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("agribot/gps",1);
    ros::Publisher heading_pub = n.advertise<agribot_msgs::AgribotHeading>("agribot/heading",1);
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField magnetometer_msg;
    sensor_msgs::NavSatFix gps_msg;
    agribot_msgs::AgribotHeading heading_msg;

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
                    //Publish:
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = "/agribot/imu";
                    tf2::Quaternion quat;
                    quat.setRPY(Hydra.pfGetRPY()[0], Hydra.pfGetRPY()[1], Hydra.pfGetRPY()[2]);
                    imu_msg.orientation = tf2::toMsg(quat);
                    imu_msg.angular_velocity.x = Hydra.pfGetGyro()[0];
                    imu_msg.angular_velocity.y = Hydra.pfGetGyro()[1];
                    imu_msg.angular_velocity.z = Hydra.pfGetGyro()[2];
                    imu_msg.linear_acceleration.x = Hydra.pfGetAccel()[0];
                    imu_msg.linear_acceleration.y = Hydra.pfGetAccel()[1];
                    imu_msg.linear_acceleration.z = Hydra.pfGetAccel()[2];
                    magnetometer_msg.header.stamp = ros::Time::now();
                    magnetometer_msg.header.frame_id = "/agribot/magnetometer";
                    magnetometer_msg.magnetic_field.x = Hydra.pfGetMagnetometr()[0];
                    magnetometer_msg.magnetic_field.y = Hydra.pfGetMagnetometr()[1];
                    magnetometer_msg.magnetic_field.z = Hydra.pfGetMagnetometr()[2];
                    imu_pub.publish(imu_msg);
                    magnetometer_pub.publish(magnetometer_msg);
                }
                if((Hydra.u16GetDataReadiness()&SECOND_DATASET_READY) == SECOND_DATASET_READY)
                {
                    ROS_INFO("[Hydrabox node] Second dataset ready to publish");
                    Hydra.vClearSecondDataset();
                    gps_msg.header.stamp = ros::Time::now();
                    gps_msg.header.frame_id = "/agribot/gps";
                    gps_msg.latitude = Hydra.dGetLat();
                    gps_msg.longitude = Hydra.dGetLon();
                    gps_msg.altitude = Hydra.dGetHei();
                    gps_msg.status.status = Hydra.u8GetFix();
                    gps_msg.status.service = 1; //SERVICE_GPS
                    gps_msg.position_covariance_type = 2; //DIAGONAL_KNOWN
                    gps_msg.position_covariance[0] = Hydra.fGetHorizontalAcc();
                    gps_msg.position_covariance[3] = Hydra.fGetVerticalAcc();
                    heading_msg.header.stamp = ros::Time::now();
                    heading_msg.header.frame_id = "/agribot/heading";
                    heading_msg.heading = Hydra.fGetHeading();
                    heading_msg.heading_accuracy = Hydra.fGetHeadingAcc();
                    gps_pub.publish(gps_msg);
                    heading_pub.publish(heading_msg);
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
