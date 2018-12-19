## guidance

This package implements the guidance capabilities

### Node

+ [`guide`](src/guide.cpp)
	1) Subscribe to [`gui`](../kugle_sensor_suite_gui) button event topics
	2) Initialise navigation to a destination specified by the user on "Navigate" button click event
	3) Track the guidee
		1) Adjust velocity limit based on guidee' velocity and distance
		2) Cancel the guidance if the guidee cannot be detected
	4) Pause and resume the navigation if the "Pause/Resume" button event occurs
	5) Return to the docking station after finishing the guidance, losing the guidee or getting a request from guidee to continue alone in the form of "Continue Alone" click event

> Note: This node will not increase the velocity limit if the [`safety`](../safety) package has already reduced it.

### Launch Files

+ [`bringup`](launch/bringup.launch)
	1) Launch `guidance/guide` node
