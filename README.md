# Android sensor bridge for ROS

## Overview

[![Android sensor bridge for ROS ](http://img.youtube.com/vi/K4_FIi-hl-w/0.jpg)](http://www.youtube.com/watch?v=K4_FIi-hl-w)

A collection of python scripts that publish sensor data from your phone (connected via WiFi or USB) to ROS. Supports a small variety of data logging apps (no dedicated Android app required):

* `ip_webcam_bridge.py`: Connects to [IP Webcam](https://play.google.com/store/apps/details?id=com.pas.webcam) and publishes camera images as `sensor_msgs/CompressedImage`. Launch `http://<phone_ip>:<port>` in the browser to configure!

* `sensrec_bridge.py`: Connects to [Sensors Record](https://play.google.com/store/apps/details?id=pl.mrwojtek.sensrec.app) and publishes:
    - `sensor_msgs/Imu` inertial measurement based on phone's rotation vector, linear acceleration and gyroscope
    - `sensor_msgs/MagneticField` magnetic field based on phone's magnetometer
    - `nmea_msgs/Sentence` low-level GPS NMEA sentecnes for processing with 3rd party tools (e.g. `nmea_navsat_driver`)
    - `sensor_msgs/NavSatFix` location based on phone's location API
    - `geometry_msgs/TwistStamped` velocity based on phones location API
    - battery info
    - pressure
    - ambient light intensity
    
    Check the [Android Sensor Types](https://source.android.com/devices/sensors/sensor-types) documentation for detais!

## Related projects

* [android_sensors_driver](https://github.com/ros-android/android_sensors_driver) relies on a dedicated Android app
