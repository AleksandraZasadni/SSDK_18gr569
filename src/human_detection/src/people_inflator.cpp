#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "leg_tracker/PersonArray.h"
#include "tf/transform_datatypes.h"

#define NUMBER_OF_POINTS_PER_PERSON 10
#define DISTANCE_FROM_PERSON_X 0.15
#define DISTANCE_FROM_PERSON_Y 0.20
#define DIRECTIONAL_OFFSET 0.05

class PeopleInflator {
 private:
	ros::Subscriber people_sub;
	ros::Publisher scan_pub;
	std::string frame_id;

 public:

	PeopleInflator();

 private:

	void people_callback(const leg_tracker::PersonArray::ConstPtr &people);
};

PeopleInflator::PeopleInflator()
{
	ros::NodeHandle n, n_params("~");
	std::string people_topic, inflating_topic;
	n_params.param<std::string>("input", people_topic, "people_tracked");
	n_params.param<std::string>("output", inflating_topic, "inflatingPointCloud");
	n_params.param<std::string>("frame_id", frame_id, "base_link");

	people_sub = n.subscribe<leg_tracker::PersonArray>(people_topic, 1, &PeopleInflator::people_callback, this);
	scan_pub = n.advertise<sensor_msgs::PointCloud>(inflating_topic, 1);

	n_params.deleteParam("input");
	n_params.deleteParam("output");
	n_params.deleteParam("frame_id");
	ros::spin();
}

void PeopleInflator::people_callback(const leg_tracker::PersonArray::ConstPtr &people)
{
	sensor_msgs::PointCloud inflatingPointCloud;
	inflatingPointCloud.header.frame_id = frame_id;
	inflatingPointCloud.header.stamp = ros::Time::now();

	for (int i = 0; i < people->people.size(); ++i) {
		float angle = 0;
		for (int j = 0; j < NUMBER_OF_POINTS_PER_PERSON; ++j) {
			angle += 2 * M_PI / NUMBER_OF_POINTS_PER_PERSON;

			tf::Quaternion q(
					people->people[i].pose.orientation.x,
					people->people[i].pose.orientation.y,
					people->people[i].pose.orientation.z,
					people->people[i].pose.orientation.w);
			double roll, pitch, yaw;
			tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

			geometry_msgs::Point32 inflatingPoint;
			inflatingPoint.x = static_cast<float>(people->people[i].pose.position.x + cos(yaw) * DIRECTIONAL_OFFSET +
					DISTANCE_FROM_PERSON_X * cosf(angle) * cos(yaw) -
					DISTANCE_FROM_PERSON_Y * sinf(angle) * sin(yaw));
			inflatingPoint.y = static_cast<float>(people->people[i].pose.position.y + sin(yaw) * DIRECTIONAL_OFFSET +
					DISTANCE_FROM_PERSON_X * cosf(angle) * sin(yaw) +
					DISTANCE_FROM_PERSON_Y * sinf(angle) * cos(yaw));
			inflatingPoint.z = 0;
			inflatingPointCloud.points.push_back(inflatingPoint);
		}
	}

	scan_pub.publish(inflatingPointCloud);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "people_inflator");

	PeopleInflator peopleInflator3000;

	return (0);
}
