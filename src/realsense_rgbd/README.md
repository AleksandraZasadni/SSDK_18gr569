## realsense_rgbd

The way in which [Intel® RealSense™ Depth Camera D435](https://click.intel.com/intelr-realsensetm-depth-camera-d435.html) is utilised in this project is handled by this package.
It is both used to acquire the depth map and process it to find obstacles as well as cliffs in front of the robot.

### Node
+ [`pcl2_project`](src/pcl2_project.cpp)
	1) Subscribe to PointCloud2
	2) Project all points to xy-plane by setting the z-value to 0.0
	3) Republish under different topic

### Launch Files

The package contains two launch files:
+ [`bringup_single`](launch/bringup_single.launch) (a single device)
	1) Initialise one RealSense camera by including `d435_bringup`
+ [`bringup_dual`](launch/bringup_dual.launch) (two devices) - currently not utilised
	1) Initialise two RealSense cameras by including `d435_bringup` twice
+ [`d435_bringup`](launch/d435_bringup.launch)
	1) Initialise RealSense D435 with custom settings
	2) Crop the depth image to reduce near-boundary noise with a [`image_proc/crop_decimate`](http://wiki.ros.org/image_proc) nodelet
	3) Convert the cropped depth image into PointCloud2 topic by [`depth_image_proc/point_cloud_xyz`](http://wiki.ros.org/depth_image_proc)
	4) Include `d435_bringup_delayed` with a delay
+ [`d435_bringup_delayed`](launch/d435_bringup_delayed.launch)
	1) Run [`pcl/PassThrough`](http://wiki.ros.org/pcl) nodelet that filters the points based on its distance to xy-plane in `base_link` frame while removing points that are higher than the robot
	2) Run another [`pcl/PassThrough`](http://wiki.ros.org/pcl) to remove points that form the floor
	3) Orthogonally project all points to the xy-plane using [`realsense_rgbd/pcl2_project`](src/pcl2_project.cpp) node
	4) Downsample points by the use of [`pcl/VoxelGrid`](http://wiki.ros.org/pcl) nodelet


### Additional dependencies

##### [realsense2_camera](http://wiki.ros.org/realsense2_camera) package

```sh
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
$ version="2.16.1-0~realsense0.88"
$ sudo apt-get install librealsense2-dkms librealsense2=${version} librealsense2-utils=${version} librealsense2-dev=${version} librealsense2-dbg=${version} ros-kinetic-rgbd-launch
```

> Note: Older version of librealsense2 is utilised as it does not have issues with build errors of the realsense2_camera package

- Download latest realsense2_camera pkg from [here](https://github.com/intel-ros/realsense/releases).
- Create the following folder structure: `.../realsense_ros/src/realsense2_camera/`
- Build the package within realsense_ros folder

```sh
$ catkin_make clean
$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
$ catkin_make install
$ echo "source $PWD/devel/setup.bash" >> ~/.bashrc
```
