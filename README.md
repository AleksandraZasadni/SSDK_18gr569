## SSDK - 18gr569

![ROS](https://img.shields.io/badge/ROS-Kinetic-brightgreen.svg)  ![OS](https://img.shields.io/badge/OS-Ubuntu%2016.04-orange.svg )

This stack of ROS packages proposes a sensor suite for Kugle and is developed by the 5th semester Bsc. of Robotics group `18gr569` at Aalborg University.

> Note: The code comes "as is" and the group is not responsible for any possible harm to the surroundings or the robot itself caused by running this program.


### Packages
The stack includes the following list of packages:
+ [`kugle_sensor_suite_description`](/src/kugle_sensor_suite_description) - Includes transformations, mechanical properties and mesh of the robot
+ [`imu`](/src/imu) - Acquires roll and pitch from IMU to counteract negative effects caused by tilt of the robot
+ [`sick_lidar`](/src/sick_lidar) - Gathers the data from two SICK LIDARs and merges them together
+ [`realsense_rgbd`](/src/realsense_rgbd) - Interfaces RealSense camera to detect objects and cliffs
+ [`maxbotix_sonar`](/src/maxbotix_sonar) - Acquires range measurements from USB Maxbotix SONAR
+ [`human_detection`](/src/human_detection) - Detection of people based on LIDAR readings
+ [`navigation`](/src/navigation) - Mapping (gmapping), localisation (amcl) and path planning (global_planner/teb_local_planner for the robot
+ [`safety`](/src/safety) - A simple implementation of safety features limiting the velocities and accelerations
+ [`guidance`](/src/guidance) - An implementation of Kugle's possible application - guidance of people
+ [`kugle_sensor_suite_gui`](/src/kugle_sensor_suite_gui) - Qt GUI supporting the guidance

Furthermore, the [`kugle_sensor_suite`](/src/kugle_sensor_suite) package collects the other into a single launch file that can be used to initialise guiding.


> Note: More information about the details of each package can be found under the packages themselves.


#### General guideline of steps that must be performed before changing the location where the robot shall guide people:
1) Get a map of the location used for localisation (either through mapping or likewise)
2) Create a map of locations that the robot shall not traverse through
3) Create a map of dangerous areas
4) Mark the docking station of the robot
5) Specify the locations that the users can be guided to

##### Special thanks to:
+ Our supervisor for proposing this project and aiding in the development
+ All participants of the testing
+ Developer of [`leg_tracker`](https://github.com/angusleigh/leg_tracker)
