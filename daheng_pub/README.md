# daheng_pub

## Installation of daheng_pub

This installation guide was tested with Ubuntu 20.04 
- ROS (>= Noetic) (see [installation guide](http://wiki.ros.org/ROS/Installation)).


### Install

Install Galaxy_Linux_Python library:

    git clone git@github.com:Liuxiaoyan-sdu/Galaxy_Linux_Python.git
    cd ./api
    python3 setup.py build
    sudo python3 setup.py install

Finally, build Daheng Pub:

    catkin_make



## Run Examples

First, make sure Get Pose is sourced in your current terminal:

    source /devel/setup.bash

Run:

    roscore 
    rosrun daheng_pub image_pub.py


## Contents
- [Installation of Galaxy_Linux_Python library:](Galaxy_Linux_Python_2.0.2106.9041/README.md)


