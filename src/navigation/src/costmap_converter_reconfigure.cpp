#include "ros/ros.h"
#include "dynamic_reconfigure/DoubleParameter.h"
#include "dynamic_reconfigure/Reconfigure.h"

#define RECONFIGURE_SERVICE_CALL "move_base/TebLocalPlannerROS/costmap_converter/CostmapToDynamicObstacles/CostmapToPolygonsDBSMCCH/set_parameters"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "costmap_converter_reconfigure");

	ros::NodeHandle n;

	sleep(5);

	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::IntParameter int_param;
	dynamic_reconfigure::Config config;
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;

	double_param.name = "cluster_max_distance";
	double_param.value = 0.125;
	config.doubles.push_back(double_param);
	int_param.name = "cluster_min_pts";
	int_param.value = 4;
	config.ints.push_back(int_param);
	int_param.name = "cluster_max_pts";
	int_param.value = 200;
	config.ints.push_back(int_param);
	double_param.name = "convex_hull_min_pt_separation";
	double_param.value = 0.1;
	config.doubles.push_back(double_param);
	srv_req.config = config;
	ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req, srv_resp);

	return (0);
}