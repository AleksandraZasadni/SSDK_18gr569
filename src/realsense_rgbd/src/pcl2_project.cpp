#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

class PointCloud2Project {
 private:
	ros::Subscriber pcl2_sub;
	ros::Publisher pcl2_projected_pub;

 public:
	PointCloud2Project();

 private:
	void pcl2_callback(const sensor_msgs::PointCloud2::ConstPtr &pcl2_in);
};

PointCloud2Project::PointCloud2Project()
{
	ros::NodeHandle n, n_params("~");
	std::string input_topic, output_topic, cliffs_topic;
	n_params.param<std::string>("input", input_topic, "depth/pointCloud2_passthrough");
	n_params.param<std::string>("output", output_topic, "depth/pointCloud2_projected");

	pcl2_sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, &PointCloud2Project::pcl2_callback, this);
	pcl2_projected_pub = n.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

	n_params.deleteParam("input");
	n_params.deleteParam("output");
	ros::spin();
}

void PointCloud2Project::pcl2_callback(const sensor_msgs::PointCloud2::ConstPtr &pcl2_in)
{
	sensor_msgs::PointCloud2Ptr pcl2_projected = boost::make_shared<sensor_msgs::PointCloud2>();
	*pcl2_projected = *pcl2_in;

	for (sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl2_projected, "z"); iter_z != iter_z.end(); ++iter_z) {
		*iter_z = 0.0;
	}

	pcl2_projected_pub.publish(*pcl2_projected);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl2_project");

	PointCloud2Project obj;

	return (0);
}