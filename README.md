# youtube video
Click image to link to YouTube video.   
[![Video Label](http://img.youtube.com/vi/zywU8-pVBVQ/0.jpg)](https://youtu.be/zywU8-pVBVQ?t=0s)  

# ros-app-tb3-voiceorder
* To make TurtleBot3 Burger follow what some says
* This is Android application source code 
* This code refers to the below site and use rosjava class 

  Link:[RosJava]: https://github.com/rosjava/android_core

# Requirements

* Android Studio 3.0.1
* Android Phone SDK version 15 (IceCreamSandwitch)  ~ 26 (Oreo)
* ROS Kinetic

# Usage

 * Permission
I run Android Studio with sudo permission.Just be careful.
 * import projet by android studio 
```
If you don't want to build project, you just download and install Apk from google Play Store.
Search key word 'ROS AuTURBO'. 
```
 * android_controllerSample apk<br />
    Link: [android_controllerSample](https://play.google.com/store/apps/details?id=org.ros.android.controllerSample)

 * android_pubsubSample apk<br />
   Link: [android_pubsubSample](https://play.google.com/store/apps/details?id=org.ros.android.pubsubSample)


# Project Description

* android_10
``` 
This module defines an Android class for ROS.
This code has been copied from ROSJAVA and modified below. 
- MessagePub.java was added. The class is a class for ROS Publish.
- MessageSub.java was added. The class is a class for ROS Subcribe.
```
* android_pubsubSample Project
```
This module is Simple Example android application code for ROS Publish and ROS Subscribe. 
This module use class of android_10.
If you write some sentance and push button, This application publish the sentance, 
and this application subscribe the sentence. And this application display subscribed sentence 

publish topic list
1. Topic name is "string_test". Datatype is std_msgs.String.
subscribe topic list 
1. Topic name is "string_test". Datatype is std_msgs.String. 
```
* android_controllerSample Project
```
This module support below funtion. 
funtion list
 - Control turtleBot by voice, Support language is English and Korean
 - Control turtleBot by JoyStick
 - Display TurtleBot Camera View, You can modify Topic name of camera view to fit your turtlebot at application UI.
   Support Datatype is sensor_msgs.CompressedImage. Default topic name is "/camera1/image_raw/compressed"

JoystickOnlyView Class does not contain ros related functions. 
JoystickOnlyView Class only contain Android JoyStick UI.

publish topic list
1. Topic name is "/cmd_vel". Datatype is geometry_msgs.Twist.
2. Topic name is "~recognizer/output". Datatype is std_msgs.String.
subscribe topic list 
1. Topic name is "odom". Datatype is nav_msgs.Odometry.
2. Topic name is "/camera1/image_raw/compressed". Datatype is sensor_msgs.CompressedImage.
```

 When control mode is joystick, Android_controllerSample Project UI.

<img src="/picture/Screenshot_2018-02-04-11-54-11.png" width="70%" height="70%">

 When control mode is voice, Android_controllerSample Project UI.

<img src="/picture/Screenshot_2018-02-04-11-54-22.png" width="70%" height="70%">
