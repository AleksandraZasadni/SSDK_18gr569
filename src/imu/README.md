## imu

The [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) IMU is utilised to determine the robot's tilt and this package allows subsequent compensation for such tilt.
To obtain data from the IMU, GY-91 module is utilised in connection with Arduino UNO (rev3) via I2C.

### Arduino sketch

+ [`imu_ros`](arduino_sketch/imu_ros.ino)
	1) Obtain raw acceleration and gyroscope measurements from the IMU
	2) Compute roll and pitch
	3) Publish roll and pitch

### Node

+ [`imu_tf`](src/imu_tf.cpp)
	1) Subscribe to roll and pitch angles
	2) Broadcast a new tilted frame, `base_link_tilted`, with appropriate angles and located at the centre of rotation

> Note: In case of the utilised robot setup, the centre of rotation shares the same location as the `base_link` itself.


### Launch File

+ [`bringup`](launch/bringup.launch)
	1) Initialise the communication with the Arduino via [`rosserial`](http://wiki.ros.org/rosserial) package
	2) Run the `imu/imu_tf` node

### Additional dependencies

##### [rosserial](http://wiki.ros.org/rosserial) package
```sh
$ sudo apt-get install ros-kinetic-rosserial ros-kinetic-rosserial-arduino
```
