#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import csv
import yaml
import numpy as np

calibraton_time = 10 #sec

def parse_data():
    accel_data = np.genfromtxt('accel_data.csv', delimiter=' ')
    gyro_data = np.genfromtxt('gyro_data.csv', delimiter=' ')
    return accel_data[:,0], accel_data[:,1], accel_data[:,2], gyro_data[:,0], gyro_data[:,1], gyro_data[:,2]

def calc_constants_and_save_to_yaml():
    x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro = parse_data()
    try:
        with open("accel_calib_const.yaml", "w") as file:
            yaml.safe_dump(dict(offsets = [float(np.average(x_acc)), float(np.average(y_acc)), float(np.average(z_acc))]), file)
    except IOError:
        print("Can't open yaml file")
    
    try:
        with open("gyro_calib_const.yaml", "w") as file:
            yaml.safe_dump(dict(offsets = [float(np.average(x_gyro)), float(np.average(y_gyro)), float(np.average(z_gyro))]), file)
    except IOError:
        print("Can't open yaml file")
    
    return

def collect_imu(data):
    with open('accel_data.csv', mode='a') as accel_data:
        accel_writer = csv.writer(accel_data, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        accel_writer.writerow([data.linear_acceleration.x , data.linear_acceleration.y, data.linear_acceleration.z])
    accel_data.close()

    with open('gyro_data.csv', mode='a') as gyro_data:
        gyro_writer = csv.writer(gyro_data, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        gyro_writer.writerow([data.angular_velocity.x , data.angular_velocity.y, data.angular_velocity.z])
    gyro_data.close()

def node():
    rospy.init_node('get_constants_imu')
    sub_imu = rospy.Subscriber("/agribot/imu/raw", Imu, collect_imu)
    # Phase1 - gatering data
    phase1 = True
    # Phase2 - gatering data
    phase2 = False
    
    with open('accel_data.csv', mode='w+') as accel_data:
        accel_data.close()
    
    with open('gyro_data.csv', mode='w+') as gyro_data:
        gyro_data.close()

    now = rospy.get_rostime().secs
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        end = rospy.get_rostime().secs
        if end - now == calibraton_time and phase1 == True:
            sub_imu.unregister()
            phase1 = False
            phase2 = True
        if phase2 == True:
            calc_constants_and_save_to_yaml()
            phase2 = False

        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
