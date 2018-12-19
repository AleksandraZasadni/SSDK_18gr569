## safety

Safety within the current implementation is performed based on dangerous areas marked on a separate map. Examples of such maps can be seen [here](../navigation/maps/dangerous_areas/).

> Note: Due to the limitations of current lazy implementation, the safety feature based on dangerous areas will NOT function properly with maps that have their frames rotated.

### Node

+ [`safety`](src/safety.cpp)
	+ Dangerous areas
		1) Subscribe to the robot's pose estimate obtained from localisation
		2) Determine whether the area is dangerous based on pixel value at robot's position in the map
		3) If dangerous, reduce the velocity and acceleration limits to preset safe values

> Note: The velocity limits set by this package do not overwrite the limits determined by the [`guidance`](../guidance/) package, if these are already lower.
 Otherwise, the safety limits have higher priority.

### Launch File
+ [`bringup`](launch/bringup.launch)
	1) Launch the safety node
