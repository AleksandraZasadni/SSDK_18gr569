## kugle_sensor_suite

This package contains launch files collecting the entire kugle_sensor_suite stack.

### Launch Files

+ [`move_base`](/launch/move_base.launch)
	1) Launch [`kobuki`](http://wiki.ros.org/kobuki_node) nodelet
	2) Include [`kobuki_auto_docking`](http://wiki.ros.org/kobuki_auto_docking)

The following launch files are linked together with delays, which is to accommodate for necessity of certain nodes to boot, especially the [`imu`](../imu) package.

+ [`bringup`](/launch/bringup.launch)
+ [`bringup_delayed`](/launch/bringup_delayed.launch)
+ [`bringup_delayed_delayed`](/launch/bringup_delayed_delayed.launch)
	1) [`mobile_base`](/launch/mobile_base.launch)
	2) [`robot_description`](../kugle_sensor_suite_description/launch/display.launch)
	3) [`imu`](../imu/launch/bringup.launch)
	4) [`sick_lidar`](../sick_lidar/launch/bringup.launch)
	5) [`maxbotix_sonar`](../maxbotix_sonar/launch/bringup_single.launch)
	6) [`human_detection`](../human_detection/launch/bringup.launch)
	7) [`realsense_rgbd`](../realsense_rgbd/launch/bringup_single.launch)
	8) [`navigation`](../navigation/launch/bringup.launch)
	9) [`safety`](../safety/launch/bringup.launch)
	10) [`guidance`](../guidance/launch/bringup.launch)
	11) [`rqt_gui`](../kugle_sensor_suite_gui)
	12) [`rviz`](../guidance/rviz.rviz)


### Additional dependencies

##### [ROS (Kinetic-desktop-full)](http://wiki.ros.org/kinetic/Installation/Ubuntu) on [Ubuntu 16.04.5 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)
```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

##### [kobuki](http://wiki.ros.org/kobuki) package
```sh
$ sudo apt-get install ros-kinetic-kobuki ros-kinetic-kobuki-core
$ rosrun kobuki_ftdi create_udev_rules
$ sudo usermod -a -G dialout $USER
```

##### [timed_roslaunch](http://wiki.ros.org/timed_roslaunch) package
```sh
$ sudo apt-get install ros-kinetic-timed-roslaunch
```
