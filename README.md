# ros-app-tb3-voiceorder
To make TurtleBot3 Burger follow what some says

## Requirements

* Python3.5
* Ubuntu 16.04
* Opencv3.2
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Tensorflow](https://www.tensorflow.org/install/)
* [find_object_2d (ROS package)](http://wiki.ros.org/find_object_2d)
* [usb_cam (ROS package)](http://wiki.ros.org/usb_cam)
* [turtlebot3_bringup (ROS package)](https://github.com/ROBOTIS-GIT/turtlebot3)


## Hardware
 * [Fish eye camera](https://ko.aliexpress.com/item/2mp-hd-1-3-CMOS-AR0330-H-264-mini-cmos-fpv-180-degree-wide-angle-fisheye/32793788459.html?trace=msiteDetail2pcDetail)
 * [Intel Joule](https://software.intel.com/en-us/iot/hardware/joule/dev-kit)
 * [Turtlebot3 burger](http://en.robotis.com/index/product.php?cate_code=111510)
## Usage

1. Permission
You should do
```
chmod +x "file name"
```
for all the files inside of src forder.

2. Path of machine learning data 
In the "signal_sign_detection.py"
```
self.saver.restore(self.sess, "/home/kihoon/catkin_ws/src/self_driving_turtlebot3/src/signal_sign_detection/model/model.ckpt")
```
Change this line for your workspace environment

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

