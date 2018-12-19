#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

//#define CALIBRATE
#define CALIBRATION_OFFSET_PITCH  -0.0403871667
#define CALIBRATION_OFFSET_ROLL   0.00886605667

#ifdef CALIBRATE
double rpy_x_sum = 0, rpy_y_sum = 0;
unsigned long counter = 0;
#endif

class TiltTF2Broadcaster {
 private:
	ros::Subscriber imu_rpy_sub;

	geometry_msgs::TransformStamped transformStamped;

 public:
	TiltTF2Broadcaster();

 private:
	void imu_callback(const geometry_msgs::Vector3::ConstPtr &rpy);
};

TiltTF2Broadcaster::TiltTF2Broadcaster()
{
	ros::NodeHandle n, n_params("~");
	std::string imu_rpy_topic, imu_frame, base_link_frame;
	n_params.param<std::string>("input", imu_rpy_topic, "tilt_angle");
	n_params.param<std::string>("base_link", base_link_frame, "base_link");
	n_params.param<std::string>("imu_frame", imu_frame, "base_link_tilted");

	imu_rpy_sub = n.subscribe<geometry_msgs::Vector3>(imu_rpy_topic, 1, &TiltTF2Broadcaster::imu_callback, this);

	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;

	transformStamped.header.frame_id = base_link_frame;
	transformStamped.child_frame_id = imu_frame;

	n_params.deleteParam("input");
	n_params.deleteParam("base_link");
	n_params.deleteParam("imu_frame");
	ros::spin();
}

void TiltTF2Broadcaster::imu_callback(const geometry_msgs::Vector3::ConstPtr &rpy)
{
	transformStamped.header.stamp = ros::Time::now();

	tf2::Quaternion quat;
	quat.setRPY(rpy->x - CALIBRATION_OFFSET_ROLL, rpy->y - CALIBRATION_OFFSET_PITCH, 0.0);
	transformStamped.transform.rotation.x = quat.x();
	transformStamped.transform.rotation.y = quat.y();
	transformStamped.transform.rotation.z = quat.z();
	transformStamped.transform.rotation.w = quat.w();

	static tf2_ros::TransformBroadcaster imuBroadcaster;
	imuBroadcaster.sendTransform(transformStamped);

#ifdef CALIBRATE
	rpy_x_sum += rpy->x;
	rpy_y_sum += rpy->y;
	counter++;
	ROS_ERROR_STREAM("Calibration --- roll: " << rpy_x_sum/counter << "\t pitch: " << rpy_y_sum/counter << "\n");
#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tilt_tf2_broadcaster");

	TiltTF2Broadcaster tf2_tilt_broadcast;

	return (0);
}