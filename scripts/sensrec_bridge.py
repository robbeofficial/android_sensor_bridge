#!/usr/bin/env python
# Robert Walter
#   https://github.com/robbeofficial

# check this: https://source.android.com/devices/sensors/sensor-types

# TODO create class
# TODO check parse errors
# TODO include system time stamps in header
# TODO add http://docs.ros.org/api/sensor_msgs/html/msg/TimeReference.html message
# TODO use http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html for battery
# TODO check what of message generation stuff is actually important

import socket, traceback

from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from nmea_msgs.msg import Sentence as NmeaSentence

import rospy

rospy.init_node('sensrec_bridge')

# pubs
pub_imu = rospy.Publisher('~imu/data', Imu, queue_size=1)
pub_mag = rospy.Publisher('~imu/mag', MagneticField, queue_size=1)

pub_nmea = rospy.Publisher('~gps/nmea', NmeaSentence, queue_size=1)
#pub_nmea = rospy.Publisher('nmea_sentence', NmeaSentence, queue_size=1)
pub_fix = rospy.Publisher('~gps/fix', NavSatFix, queue_size=1)
pub_vel = rospy.Publisher('~gps/vel', TwistStamped, queue_size=1)

pub_pressure = rospy.Publisher('~pressure', Float64, queue_size=1)
pub_light = rospy.Publisher('~light', Float64, queue_size=1)
pub_bat_level = rospy.Publisher('~battery/level', Float64, queue_size=1)
pub_bat_voltage = rospy.Publisher('~battery/voltage', Float64, queue_size=1)
pub_bat_temp = rospy.Publisher('~battery/temperature', Float64, queue_size=1)

# params
host = rospy.get_param('~host', '')
port = rospy.get_param('~port', 44335)
socket_bufsize = rospy.get_param('~socket_bufsize', 4096)

# UDP socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((host, port))

# holds latest message for each sensor type
vals = {}

def messages(sock, bufsize=4096):
    "reads newline-separated messages from socket"
    buf = sock.recv(bufsize)
    buffering = True
    while buffering:
        if "\n" in buf:
            (line, buf) = buf.split('\n', 1)
            yield line
        else:
            more = sock.recv(bufsize)
            if not more:
                buffering = False
            else:
                buf += more
    if buf:
        yield buf

while not rospy.is_shutdown():
    try:
        for message in messages(sock, socket_bufsize):
            fields = message.strip().split('\t') # message fields are tab-separated

            sensor = fields[0]
            vals[sensor] = fields

            # check https://github.com/mrwojtek/sens-rec/blob/b0d34aad1e827720cb6eb11b512ad9eff021a1e5/lib/src/main/java/pl/mrwojtek/sensrec/SensorRecorder.java#L121
            # For Android Sensors: type_id ms ns length values[]
            if sensor not in ['rotv_0', 'lacc_0', 'gyro_0', 'magn_0', 'press_0', 'light_0', 'gps', 'nmea', 'bat']:
                rospy.logerr("Unknwon message: '{}'".format(message))

            # publish an Imu message for each received rotation vector
            if sensor == 'rotv_0':
                rotv = fields
                imu_msg = Imu()
                imu_msg.header.frame_id = 'android'
                imu_msg.orientation.x = float(rotv[4])
                imu_msg.orientation.y = float(rotv[5])
                imu_msg.orientation.z = float(rotv[6])
                imu_msg.orientation.w = float(rotv[7])
                if 'lacc_0' in vals:
                    lacc = vals['lacc_0']
                    imu_msg.linear_acceleration.x = float(lacc[4])
                    imu_msg.linear_acceleration.y = float(lacc[5])
                    imu_msg.linear_acceleration.z = float(lacc[6])
                if 'gyro_0' in vals:
                    gyro = vals['gyro_0']
                    imu_msg.angular_velocity.x = float(gyro[4])
                    imu_msg.angular_velocity.y = float(gyro[5])
                    imu_msg.angular_velocity.z = float(gyro[6])
                pub_imu.publish(imu_msg)

            # publish magnetic field
            elif sensor == 'magn_0':
                magn = fields
                mag_msg = MagneticField()
                mag_msg.header.frame_id = 'android'
                mag_msg.magnetic_field.x = float(magn[4])
                mag_msg.magnetic_field.y = float(magn[5])
                mag_msg.magnetic_field.z = float(magn[6])
                pub_mag.publish(mag_msg)

            # publish pressure
            elif sensor == 'press_0':
                msg_press = Float64()
                msg_press.data = float(fields[4])
                pub_pressure.publish(msg_press)

            # publish light
            elif sensor == "light_0":
                msg_light = Float64()
                msg_light.data = float(fields[4])
                pub_light.publish(msg_light)

            # publish NMEA setence
            elif sensor == "nmea":
                msg_nmea = NmeaSentence()
                msg_nmea.header.frame_id = 'android'
                msg_nmea.header.stamp = rospy.Time.from_sec(float(fields[2]) / 1000)
                msg_nmea.sentence = fields[3]
                pub_nmea.publish(msg_nmea)

            # publish fix and vel from device location
            # check https://github.com/mrwojtek/sens-rec/blob/b0d34aad1e827720cb6eb11b512ad9eff021a1e5/lib/src/main/java/pl/mrwojtek/sensrec/LocationRecorder.java#L113
            # check https://developer.android.com/reference/android/location/Location.html#getAccuracy()
            # gps	48021735	52.51782937923822	13.447371576740874	82.6810805981322	0.0	9.953303E-4	12.0	1496696180000
            # type ms lat lon alt bearing[deg] speed[m/s] accuracy[m] time[s]
            # horizontal accuracy of this location, radial, in meters
            # bearing is the horizontal direction of travel of this device (0.0, 360.0], 0.0 = no bearing [deg]
            elif sensor == 'gps':
                stamp = rospy.Time.from_sec(float(fields[8]) / 1000)

                # publish fix
                msg_fix = NavSatFix()
                msg_fix.header.frame_id = 'android'
                msg_fix.header.stamp = stamp
                msg_fix.latitude = float(fields[2])
                msg_fix.longitude = float(fields[3])
                msg_fix.altitude = float(fields[4])
                var = float(fields[7])
                msg_fix.position_covariance = [var*var, 0, 0, 0, var*var, 0, 0, 0, var*var]
                msg_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                pub_fix.publish(msg_fix)

                # publish velicity
                msg_vel = TwistStamped()
                msg_vel.header.frame_id = 'android'
                msg_vel.header.stamp = stamp
                msg_vel.twist.linear.x = float(fields[6])
                pub_vel.publish(msg_vel)

            # publish battery info
            # check https://github.com/mrwojtek/sens-rec/blob/b0d34aad1e827720cb6eb11b512ad9eff021a1e5/lib/src/main/java/pl/mrwojtek/sensrec/BatteryRecorder.java#L110
            # bat	49820907	0.51	3826	299
            # type ms percentage voltage[mV] temperature[delsius*10]
            elif sensor == 'bat':
                pub_bat_level.publish(float(fields[2]))
                pub_bat_voltage.publish(float(fields[3]) / 1000)
                pub_bat_temp.publish(float(fields[4]) / 10)

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()