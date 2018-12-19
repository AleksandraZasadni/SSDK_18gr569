## human_detection

The [`leg_tracker`](https://github.com/angusleigh/leg_tracker/tree/kinetic) is utilised within this package to detect nearby people by the use of the LIDARs. The primary application of this detection is during the [`guidance`](../guidance).

### Node

+ [`people_inflator`](src/people_inflator.cpp)
	1) Subscribe to array of detected people
	2) Draw sensor_msgs::PointCloud ellipse around each human to inflate the avoidance area during the navigation

### Launch Files

+ [`bringup`](launch/bringup.launch)
	1) Launch [`leg_tracker`](https://github.com/angusleigh/leg_tracker)
	2) Run `human_detection/people_inflator` node

### Additional dependencies

##### [leg_tracker](https://github.com/angusleigh/leg_tracker) package

This package need to be build and sourced (either in the same or a separate catkin workspace)
