# ros-app-tb3-voiceorder
To make TurtleBot3 Burger follow what some says
This code refers to the site below
https://github.com/rosjava/android_core

## Requirements

* Android Studio 3.0.1

## Support OS

* Android Phone SDK version 15 IceCreamSandwitch  ~ 26 Oreo
* ROS Kinentic

## Usage

1. Permission
You should do
```
chmod +x "file name"
```
2. import projet by studio 
```
self.saver.restore(self.sess, "/home/kihoon/catkin_ws/src/self_driving_turtlebot3/src/signal_sign_detection/model/model.ckpt")
```
3. catkin make
```
cd ~/catkin_ws && catkin_make
```
4. roscore and roslaunch
```
roscore
roslaunch turtlebot3_bringup turtlebot3_rbot.launch
roslaunch self_driving_turtlebot3 self_driving.launch
```

## Some projects which inspired me
* [camera calibration](http://darkpgmr.tistory.com/32)
* [haar cascade](https://www.youtube.com/watch?v=jG3bu0tjFbk&list=PLQVvvaa0QuDdttJXlLtAJxJetJcqmqlQq&index=17)
* [A* Pathfinding Algorithm](https://www.youtube.com/watch?v=aKYlikFAV4k&index=65&list=PLRqwX-V7Uu6ZiZxtDDRCi6uhfTH4FilpH&t=5s)
* [recognizing rectangular box](http://www.pyimagesearch.com/2014/03/10/building-pokedex-python-getting-started-step-1-6/)
* [blob tetection using Opencv](https://www.learnopencv.com/blob-detection-using-opencv-python-c/)
* [tensorflow in ROS](https://github.com/shunchan0677/Tensorflow_in_ROS)

