#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hal_hydrabox/Hydrabox.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include "hal_hydrabox/TriggerOffsetLoad.h"

#define INIT_INTERVAL 30 //3s

#define LINEAR_ACCELERATION_COVARIANCE  0.04
#define ANGULAR_VELOCITY_COVARIANCE     0.02
#define MAGNETOMETER_COVARIANCE         0.001

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hydrabox");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("agribot/imu/raw",1);
    ros::Publisher magnetometer_pub = n.advertise<sensor_msgs::MagneticField>("agribot/magnetic_field",1);
    ros::Publisher gnss_pub = n.advertise<sensor_msgs::NavSatFix>("agribot/gnss",1);
    ros::Publisher heading_pub = n.advertise<sensor_msgs::Imu>("agribot/heading",1);
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField magnetometer_msg;
    sensor_msgs::NavSatFix gnss_msg;
    sensor_msgs::Imu heading_msg;
    uint8_t u8Timer = INIT_INTERVAL;

    if(argc < 2)
    {
        std::cout << "Specify can channel [can0 or can1]" << std::endl;
        return -1;
    }

    Hydrabox Hydra(argv[1]);
    //Hydra.vInitOffsets(argv);
    while(ros::ok())
    {
        u8Timer++;
        if(u8Timer >= INIT_INTERVAL) //every 3s, try to initialize hydrabox
        {
            u8Timer = 0;
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
                            Hydra.vClearFirstDataset();
                            //Publish:
                            imu_msg.header.stamp = ros::Time::now();
                            imu_msg.header.frame_id = "imu_link";
                            tf2::Quaternion quat;
                            quat.setRPY(Hydra.pfGetRPY()[0], Hydra.pfGetRPY()[1], Hydra.pfGetRPY()[2]);
                            imu_msg.orientation = tf2::toMsg(quat);
                            imu_msg.angular_velocity.x = Hydra.pfGetGyro()[0];
                            imu_msg.angular_velocity.y = Hydra.pfGetGyro()[1];
                            imu_msg.angular_velocity.z = Hydra.pfGetGyro()[2];
                            imu_msg.angular_velocity_covariance[0] = ANGULAR_VELOCITY_COVARIANCE;
                            imu_msg.angular_velocity_covariance[4] = ANGULAR_VELOCITY_COVARIANCE;
                            imu_msg.angular_velocity_covariance[8] = ANGULAR_VELOCITY_COVARIANCE;
                            imu_msg.linear_acceleration.x = Hydra.pfGetAccel()[0];
                            imu_msg.linear_acceleration.y = Hydra.pfGetAccel()[1];
                            imu_msg.linear_acceleration.z = Hydra.pfGetAccel()[2];
                            imu_msg.linear_acceleration_covariance[0] = LINEAR_ACCELERATION_COVARIANCE;
                            imu_msg.linear_acceleration_covariance[4] = LINEAR_ACCELERATION_COVARIANCE;
                            imu_msg.linear_acceleration_covariance[8] = LINEAR_ACCELERATION_COVARIANCE;
                            magnetometer_msg.header.stamp = ros::Time::now();
                            magnetometer_msg.header.frame_id = "magnetic_link";
                            magnetometer_msg.magnetic_field.x = Hydra.pfGetMagnetometr()[0];
                            magnetometer_msg.magnetic_field.y = Hydra.pfGetMagnetometr()[1];
                            magnetometer_msg.magnetic_field.z = Hydra.pfGetMagnetometr()[2];
                            magnetometer_msg.magnetic_field_covariance[0] = MAGNETOMETER_COVARIANCE;
                            magnetometer_msg.magnetic_field_covariance[4] = MAGNETOMETER_COVARIANCE;
                            magnetometer_msg.magnetic_field_covariance[8] = MAGNETOMETER_COVARIANCE;

                            imu_pub.publish(imu_msg);
                            magnetometer_pub.publish(magnetometer_msg);
                        }
                        if((Hydra.u16GetDataReadiness()&SECOND_DATASET_READY) == SECOND_DATASET_READY)
                        {
                            Hydra.vClearSecondDataset();
                            gnss_msg.header.stamp = ros::Time::now();
                            gnss_msg.header.frame_id = "gps_link";
                            gnss_msg.latitude = Hydra.dGetLat();
                            gnss_msg.longitude = Hydra.dGetLon();
                            gnss_msg.altitude = Hydra.dGetHei();
                            gnss_msg.status.status = Hydra.u8GetFix();
                            gnss_msg.status.service = Hydra.u8GetGnssFlag(); //SERVICE_gnss
                            float horizontalAcc = Hydra.fGetHorizontalAcc();
                            float verticalAcc = Hydra.fGetVerticalAcc();
                            gnss_msg.position_covariance[0] = horizontalAcc;
                            gnss_msg.position_covariance[4] = horizontalAcc;
                            gnss_msg.position_covariance[8] = verticalAcc;
                            //gnss_msg.rtk = Hydra.u8GetRtk();
                            tf2::Quaternion quat;
                            quat.setRPY(0, 0, Hydra.fGetHeading());
                            heading_msg.header.stamp = ros::Time::now();
                            heading_msg.header.frame_id = "gps_link";
                            heading_msg.orientation = tf2::toMsg(quat);
                            heading_msg.orientation_covariance[8] = Hydra.fGetHeadingAcc();;
                            gnss_pub.publish(gnss_msg);
                            heading_pub.publish(heading_msg);
                        }
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            else
            {
                ROS_ERROR("[Hydrabox] Configuration failed with error code %d. Retrying connection...", iRet);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
