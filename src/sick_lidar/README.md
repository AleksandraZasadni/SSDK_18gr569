## sick_lidar

This package gathers sensor_msgs::LaserScan from two [SICK TiM571-2050101](https://www.sick.com/dk/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444) 2D LIDARs and subsequently merges them together into sensor_msgs::PointCloud2 and sensor_msgs::LaserScan topics via [`ira_laser_tools/laserscan_multi_merger`](https://github.com/iralabdisco/ira_laser_tools/blob/master/src/laserscan_multi_merger.cpp) node.
Furthermore, [`pointcloud_to_laserscan`](http://wiki.ros.org/pointcloud_to_laserscan) package is utilised to disregard measurements that are too distant from the plane at which the LIDARs are mounted and republish them as a LaserScan for subsequent use for [`navigation`](../navigation).
The distance of the measured points from the observed plane is caused due to the tilt of the robot and determined by the use of [`imu`](../imu/) package.

### Launch Files

The package contains two launch files:
+ [`bringup`](launch/bringup.launch)
	1) Initialise nodes that gather the measurements from both LIDARS
+ [`bringup_delayed`](launch/bringup_delayed.launch)
	1) Merge both sensor_msgs::LaserScan topics
	2) Form a new sensor_msgs::LaserScan without points that are too low or too high

The purpose of splitting these launch files is due to necessity of having LIDAR sensor_msgs::LaserScan topics up and running before [`ira_laser_tools/laserscan_multi_merger`](https://github.com/iralabdisco/ira_laser_tools/blob/master/src/laserscan_multi_merger.cpp) node can properly merge them.


### Additional dependencies

##### [sick_tim](http://wiki.ros.org/sick_tim) package

```sh
$ sudo apt-get install ros-kinetic-sick-tim
$ echo "SUBSYSTEM==\"usb\", ACTION==\"add\", ATTR{idVendor}==\"19a2\", ATTR{idProduct}==\"5001\", GROUP=\"plugdev\"" > sick_tim.rules
$ sudo cp sick_tim.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
```

##### [ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools) package
```sh
$ sudo apt-get install ros-kinetic-ira-laser-tools
```

##### [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan) package
```sh
$ sudo apt-get install ros-kinetic-pointcloud-to-laserscan
```
