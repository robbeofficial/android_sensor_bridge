# Android sensor bridge for ROS
Publish sensor data from your phone (connected via WiFi or USB) to ROS.

## Overview

[![Android sensor bridge for ROS ](http://img.youtube.com/vi/K4_FIi-hl-w/0.jpg)](http://www.youtube.com/watch?v=K4_FIi-hl-w)

A collection of python scripts that publish sensor data from your phone (connected via WiFi or USB) to ROS. Supports a small variety of data logging apps (no dedicated Android app required):

* `ip_webcam_bridge.py`: Connects to [IP Webcam](https://play.google.com/store/apps/details?id=com.pas.webcam) and publishes camera images as `sensor_msgs/CompressedImage`. Launch `http://<phone_ip>:<port>` in the browser to configure or use service calls als follows
    - use service `<node>/<setting>/<get>` to retrieve the current value of a setting (e.g. `rosservice call /ip_webcam_bridge/video_size/get` returns `720x480`)
    - use service `<node>/<setting>/<options>` to list all available options for a setting (e.g `rosservice call /ip_webcam_bridge/video_size/options` returns `1920x1080; 1440x1080; 1088x1088; 1280x720; 1056x704; 1024x768; 960x720; 800x450; 720x720; 720x480; 640x480; 352x288; 320x240; 256x144; 176x144`)
    - use service `<node>/<setting>/<set>` to change a setting (e.g. `rosservice call /ip_webcam_bridge/video_size/set "value: {data: '720x480'}"`)

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
