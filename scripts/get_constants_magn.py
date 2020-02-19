#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import yaml

calibraton_time = 10 #sec

X_vec = []
Y_vec = []
Z_vec = []

# Phase3 - drawing data with new constants
phase3 = False

def collect_magn(data):
    global phase3,X,Y,Z

    if phase3 == False:
        with open('magn_data.csv', mode='a') as magn_data:
            magn_writer = csv.writer(magn_data, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            magn_writer.writerow([data.magnetic_field.x , data.magnetic_field.y, data.magnetic_field.z])
        magn_data.close()
    if phase3 == True:
        X_vec.append(data.magnetic_field.x)
        Y_vec.append(data.magnetic_field.y)
        Z_vec.append(data.magnetic_field.z)


def parse_data():
    magn_data = np.genfromtxt('magn_data.csv', delimiter=' ')
    return magn_data[:,0], magn_data[:,1], magn_data[:,2]

def calc_constants_and_save_to_yaml():
    x,y,z = parse_data()
    
    # Raw data
    fig = plt.figure()
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(x, y, 'r.')
    ax1.plot(x, z, 'b.')
    ax1.plot(y, z, 'c.')
    ax1.set(xlabel = '', ylabel = '', title = 'Raw magnetometer data.')
    ax1.grid()

    # Hard iron distortion filtration
    offset_x = (np.amax(x) + np.amin(x)) / 2
    offset_y = (np.amax(y) + np.amin(y)) / 2
    offset_z = (np.amax(z) + np.amin(z)) / 2

    for i in range (0, len(x)):
        x[i] = x[i] - offset_x
        y[i] = y[i] - offset_y
        z[i] = z[i] - offset_z

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(x, y, 'r.')
    ax2.plot(x, z, 'b.')
    ax2.plot(y, z, 'c.')
    ax2.set(xlabel = '', ylabel = '', title = 'After hard iron distortion filtration.')
    ax2.grid()

    # Soft iron distortion filtration
    avg_delta = (offset_x + offset_y + offset_z) / 3
    
    avg_delta_x = (np.amax(x) - np.amin(x)) / 2
    avg_delta_y = (np.amax(y) - np.amin(y)) / 2
    avg_delta_z = (np.amax(z) - np.amin(z)) / 2

    scale_x = avg_delta / avg_delta_x
    scale_y = avg_delta / avg_delta_y
    scale_z = avg_delta / avg_delta_z

    for i in range (0, len(x)):
        x[i] = x[i] * scale_x
        y[i] = y[i] * scale_y
        z[i] = z[i] * scale_z

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(x, y, 'r.')
    ax3.plot(x, z, 'b.')
    ax3.plot(y, z, 'c.')
    ax3.set(xlabel = '', ylabel = '', title = 'After soft and hard iron distortion filtration.')
    ax3.grid()
    
    ax4 = fig.add_subplot(2, 2, 4, projection ='3d')
    ax4.plot(x, y, z, 'r.')
    ax4.set(xlabel = '', ylabel = '', title = '3D after filtration')
    ax4.grid()

    try:
        with open("magn_calib_const.yaml", "w") as file:
            yaml.safe_dump(dict(offsets = [float(offset_x), float(offset_y), float(offset_z)], 
            scales = [float(scale_x), float(scale_y), float(scale_z)]), file, default_flow_style=False)
    except IOError:
        print("Can't open yaml file")

    print("Offsets: " + str(offset_x) + " " + str(offset_y) + " " + str(offset_z))
    print("Scales: " + str(scale_x) + " " + str(scale_y) + " " + str(scale_z))
    
    plt.show()
    return

def plot_data(rat):
    sub_mag = rospy.Subscriber("/agribot/magnetic_field", MagneticField, collect_magn)
    # Raw data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection ='3d')
    ax.plot(X_vec, Y_vec, Z_vec, 'b.')
    ax.set(xlabel = '', ylabel = '', title = '3D after filtration')
    
    while not rospy.is_shutdown():
        ax.plot(X_vec, Y_vec, Z_vec, 'r.')
        plt.draw()
        plt.pause(0.0001)
        ax.cla()

    return

def node():
    rospy.init_node('get_constants_magn')
    sub_mag = rospy.Subscriber("/agribot/magnetic_field", MagneticField, collect_magn)
    # Phase1 - gatering data
    phase1 = True
    # Phase2 - calculating constants
    phase2 = False
    # Phase3 - drawing data with new constants
    global phase3

    with open('magn_data.csv', mode='w+') as magn_data:
        magn_data.close()

    now = rospy.get_rostime().secs
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        end = rospy.get_rostime().secs
        if end - now == calibraton_time and phase1==True:
            sub_mag.unregister()
            phase1 = False
            phase2 = True

        if phase2 == True:
            calc_constants_and_save_to_yaml()
            phase2 = False
            phase3 = True

        if phase3 == True:
            plot_data(rate)
            phase3 = False
            return

        rate.sleep()    
        
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
