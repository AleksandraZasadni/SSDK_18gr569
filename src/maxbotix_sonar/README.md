## maxbotix_sonar

This package acquires range measurements from UBS Maxbotix devices (e.g. [HRUSB-MaxSonar](https://www.maxbotix.com/product-category/hrusb-maxsonar-ez-products "HRUSB-MaxSonar-EZ Products") series) and represents the measurements as a sensor_msgs::LaserScan while taking offset from the [beam pattern](https://www.maxbotix.com/pictures/sensor_beam/Beam%20Pattern%20MB1433.gif?TB_iframe=true&width=600&height=550 "MB1433 beam pattern") of the device.


### Nodes

This package contains two nodes:

+ [`publish`](src/maxbotix_publish.cpp)
	1) Obtain range measurements from the device
+ [`scan`](src/maxbotix_scan.cpp)
	1) Subscribe to range measurements
	2) Republish them as sensor_msgs::LaserScan


### Launch Files

The package also contains two launch files:
+ [`bringup_single`](launch/bringup_single.launch) (a single device)
	1) Initialise a single set of maxbotix_sonar nodes
+ [`bringup_dual`](launch/bringup_dual.launch) (two devices) - currently not utilised
	1) Initialise two sets of maxbotix_sonar nodes
	2) Merge individual sensor_msgs::LaserScan topics into a single sensor_msgs::LaserScan and a single sensor_msgs::PointCloud2 by the use of [`ira_laser_tools/laserscan_multi_merger`](https://github.com/iralabdisco/ira_laser_tools/blob/master/src/laserscan_multi_merger.cpp)


> Note: The default device dev paths are `/dev/ttyUSB0` and `/dev/ttyUSB1`. These might need to be changed in the launch files.

### Additional dependencies

##### [serial](http://wiki.ros.org/serial) package
```sh
$ sudo apt-get install ros-kinetic-serial
$ sudo usermod -a -G dialout $USER
```

##### [ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools) package
```sh
$ sudo apt-get install ros-kinetic-ira-laser-tools
```
