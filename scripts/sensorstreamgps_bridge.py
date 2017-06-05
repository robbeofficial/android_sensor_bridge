#!/usr/bin/env python
# Robert Walter
#   https://github.com/robbeofficial


# todo create class

from enum import Enum
import socket, traceback
import math

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

import rospy

rospy.init_node('sensorstreamgps_bridge')

pub_imu = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)

host = rospy.get_param('~host', '')
port = rospy.get_param('~port', 5555)



# TODO check why they are different from https://developer.android.com/reference/android/hardware/Sensor.html
ACCELEROMETER = 3 # [m/s^2]
GRAVITY = 83 # [m/s^2]
LINEAR_ACCELERATION = 82 # ACCELEROMETER - GRAVITY [m/s^2]

GYROSCOPE = 4 # [rad/s]
MAGNETIC_FIELD = 5 # [uT]
ORIENTATION = 81 # legacy - azimuth, pitch, yaw [deg]
ROTATION_VECTOR = 84 # roation angle theta around axis <x,y,z>: <x*sin(theta/2), y*sin(theta/2), z*sin(theta/2)> 

TEMPERATURE = 86 # [celsius]
PRESSURE = 85 # [hPa]

# unknown sensor: 1 - ['  52.517632', '  13.447430', '  83.2', ' 3', '   5.801', '-13.746', '-28.710', ' 4', '  11.797', '  0.481', ' -3.857', ' 6', '  3782828.967', '  904506.791', ' 5038124.782', ' 7', ' -0.253', '-0.131', ' 0.212', ' 8', ' 1496608842000', ' 86', ' 32']

GPS = 1
vals = {}


# TODO publish sensor_msgs/Imu

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

def pop_front(lst, n=1, cast=float):
    if n == 1:
        return cast(lst.pop(0))
    else:
        vals = lst[:n]
        del lst[:n]
        return [cast(s) for s in vals]


while not rospy.is_shutdown():
    try:
        message, address = s.recvfrom(8192)

        #print(message)
        arr = message.split(',')

        time_stamp = pop_front(arr)
        while len(arr) > 0:
            sensor = pop_front(arr, 1, int)
            if sensor in [ACCELEROMETER, GYROSCOPE, MAGNETIC_FIELD, ORIENTATION, LINEAR_ACCELERATION, GRAVITY, ROTATION_VECTOR, GPS]:
                sensor_vals = pop_front(arr, 3)
                vals[sensor] = (time_stamp, sensor_vals)
            elif sensor in [TEMPERATURE, PRESSURE]:
                sensor_vals = pop_front(arr, 1)
                vals[sensor] = (time_stamp, sensor_vals)
            else:
                print "unknown sensor: {} - {}".format(sensor, arr)
                break
        
        # publish IMU
        imu_msg = Imu()
        imu_msg.header.frame_id = 'android'
        rv = vals[ROTATION_VECTOR][1]
        imu_msg.orientation.x = rv[0]
        imu_msg.orientation.y = rv[1]
        imu_msg.orientation.z = rv[2]
        w = 1 - rv[0]*rv[0] - rv[1]*rv[1] - rv[2]*rv[2]
        if w > 0:
            w = math.sqrt(w)
        else:
            w = 0

        imu_msg.orientation.w = w
        

        imu_msg.orientation_covariance[0] = -1 # cov unknown

        imu_msg.angular_velocity.x = vals[GYROSCOPE][1][0]
        imu_msg.angular_velocity.y = vals[GYROSCOPE][1][1]
        imu_msg.angular_velocity.z = vals[GYROSCOPE][1][2]
        imu_msg.angular_velocity_covariance[0] = -1 # cov unknown

        imu_msg.linear_acceleration.x = vals[LINEAR_ACCELERATION][1][0]
        imu_msg.linear_acceleration.y = vals[LINEAR_ACCELERATION][1][1]
        imu_msg.linear_acceleration.z = vals[LINEAR_ACCELERATION][1][2]
        imu_msg.linear_acceleration_covariance[0] = -1 # cov unknown
        pub_imu.publish(imu_msg)

        # publish magfield
        mag_msg = MagneticField()
        mag_msg.header.frame_id = 'android'
        mag_msg.magnetic_field.x = vals[MAGNETIC_FIELD][1][0]
        mag_msg.magnetic_field.x = vals[MAGNETIC_FIELD][1][1]
        mag_msg.magnetic_field.z = vals[MAGNETIC_FIELD][1][2]
        pub_mag.publish(mag_msg)





    except (KeyboardInterrupt, SystemExit):
        raise 
    except:
        traceback.print_exc()