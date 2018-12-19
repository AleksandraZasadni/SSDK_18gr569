## navigation

This package provides the mobile robot with the following capabilities:

+ Mapping of the surroundings via [`gmapping`](http://wiki.ros.org/gmapping)
+ Localisation within a known map using [`amcl`](http://wiki.ros.org/amcl)
+ Global path planning by the use of [`global_planner`](http://wiki.ros.org/global_planner) (Dijkstra)
+ Local path planning and obstacle avoidance using [`teb_local_planner`](http://wiki.ros.org/teb_local_planner)


### Node

+ [`costmap_converter_reconfigure`](src/costmap_converter_reconfigure.cpp)
	1) Reconfigures ~/TebLocalPlannerROS/costmap_converter/CostmapToDynamicObstacles/ CostmapToPolygonsDBSMCCH parameters, as it is not possible to change these the regular way

### Launch Files

+ [`mapping`](launch/mapping.launch)
	1) Include move base, imu and lidars
	2) Initialise [`gmapping`](http://wiki.ros.org/gmapping)
+ [`bringup`](launch/bringup.launch)
	1) Loads three (separate) maps for localisation, global planning and safety in dangerous areas
	2) Start [`amcl`](http://wiki.ros.org/amcl)
	3) Initialise move_base with Dijkstra global and Teb local planner
	4) Run `navigation/costmap_converter_reconfigure`
+ [`teb_config`](launch/teb_config.launch) -> utilised during parameter tweaking
	1) Simulate the navigation in [`stage`](http://wiki.ros.org/stage)


### Additional dependencies

##### [navigation](http://wiki.ros.org/navigation) stack
```sh
$ sudo apt-get install ros-kinetic-navigation
```

##### [teb_local_planner](http://wiki.ros.org/teb_local_planner) package
```sh
$ sudo apt-get install ros-kinetic-teb-local-planner
```
