#include "ros/ros.h"
#include "std_msgs/builtin_float.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#define BEAM_WIDTH 0.354 //m

#define TRANSITION_DISTANCE    0.75      //m
#define TRANSITION_ANGLE       0.231759414 //rad

#define SONAR_RANGE_MIN        0.3  //m
#define SONAR_RANGE_MAX        2.0  //m
#define NUMBER_OF_RANGE_POINTS 8   //#

class MaxbotixScan {
 private:
	ros::Subscriber sonar_sub;
	ros::Publisher scan_pub;
	sensor_msgs::LaserScan sonarScan;

 public:
	MaxbotixScan();

 private:
	void sonar_callback(const std_msgs::Float32::ConstPtr &range);
};

MaxbotixScan::MaxbotixScan()
{
	ros::NodeHandle n, n_params("~");
	std::string range_topic, scan_topic, frame_id;
	n_params.param<std::string>("input", range_topic, "sonarRange");
	n_params.param<std::string>("output", scan_topic, "sonarScan");
	n_params.param<std::string>("frame_id", frame_id, "sonar");

	sonar_sub = n.subscribe<std_msgs::Float32>(range_topic, 1, &MaxbotixScan::sonar_callback, this);
	scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

	sonarScan.header.frame_id = frame_id;
	sonarScan.time_increment = sonarScan.scan_time = 0;
	sonarScan.range_min = SONAR_RANGE_MIN;
	sonarScan.range_max = SONAR_RANGE_MAX;
	sonarScan.ranges.resize(NUMBER_OF_RANGE_POINTS);

	n_params.deleteParam("input");
	n_params.deleteParam("output");
	n_params.deleteParam("frame_id");
	ros::spin();
}

void MaxbotixScan::sonar_callback(const std_msgs::Float32::ConstPtr &range)
{
	sonarScan.header.stamp = ros::Time::now();
	if (range->data > TRANSITION_DISTANCE) {
		sonarScan.angle_max = atanf(static_cast<float>(0.5 * BEAM_WIDTH / range->data));
	} else {
		sonarScan.angle_max = TRANSITION_ANGLE;
	}
	sonarScan.angle_min = -sonarScan.angle_max;

	sonarScan.angle_increment = 2 * sonarScan.angle_max / (NUMBER_OF_RANGE_POINTS - 1);

	for (uint8_t i = 0; i < NUMBER_OF_RANGE_POINTS; ++i) {
		sonarScan.ranges[i] = range->data;
	}

	scan_pub.publish(sonarScan);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "maxbotix_scan");

	MaxbotixScan maxbotix_MB1433;

	return (0);
}
