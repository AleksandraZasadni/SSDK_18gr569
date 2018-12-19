#include "ros/ros.h"
#include "std_msgs/builtin_float.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "dynamic_reconfigure/DoubleParameter.h"
#include "dynamic_reconfigure/Reconfigure.h"

#define UPDATE_RATE                 10   // Hz

#define REGULAR_MAX_VEL_X           0.392  // m/s
#define REGULAR_MAX_VEL_X_BACKWARDS 0.196  // m/s
#define REGULAR_ACC_LIM_X           0.294  // m/s^2
#define REGULAR_MAX_VEL_THETA       1.0472 // rad/s
#define REGULAR_ACC_LIM_THETA       0.78539 // rad/s^2

#define SAFE_SCALE_FACTOR           0.5
#define SAFE_MAX_VEL_X              (SAFE_SCALE_FACTOR * REGULAR_MAX_VEL_X)
#define SAFE_MAX_VEL_X_BACKWARDS    (SAFE_SCALE_FACTOR * REGULAR_MAX_VEL_X_BACKWARDS)
#define SAFE_ACC_LIM_X              (SAFE_SCALE_FACTOR * REGULAR_ACC_LIM_X)
#define SAFE_MAX_VEL_THETA          (SAFE_SCALE_FACTOR * REGULAR_MAX_VEL_THETA)
#define SAFE_ACC_LIM_THETA          (SAFE_SCALE_FACTOR * REGULAR_ACC_LIM_THETA)

#define RECONFIGURE_SERVICE_CALL    "/move_base/TebLocalPlannerROS/set_parameters"

class KugleSafety {
 private:
	ros::Subscriber pose_sub, dangerous_areas_map_sub, max_velocity_sub;
	ros::Publisher max_vel_pub;

	dynamic_reconfigure::ReconfigureRequest srv_req_regular, srv_req_safe;
	dynamic_reconfigure::ReconfigureResponse srv_resp;

	nav_msgs::OccupancyGrid dangerousAreasMap;
	bool isCurrentAreaDangerous = false;
	double guideeVelocity = REGULAR_MAX_VEL_X;

 public:
	KugleSafety();

 private:
	void current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);

	void dangerous_areas_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map);

	void velocity_callback(const std_msgs::Float32::ConstPtr &velo);

	void reconfigureSetup();
};

KugleSafety::KugleSafety()
{
	ros::NodeHandle n, n_params("~");
	std::string current_pose_topic, local_costmap_topic, dangerous_areas_map_topic, max_velocity_topic;
	n_params.param<std::string>("input_current_pose", current_pose_topic, "amcl_pose");
	n_params.param<std::string>("input_dangerous_areas_map", dangerous_areas_map_topic, "map_dangerous_areas");
	n_params.param<std::string>("output_max_velocity", max_velocity_topic, "current_max_velocity_safety");

	reconfigureSetup();

	pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(current_pose_topic, 1,
	                                                                 &KugleSafety::current_pose_callback, this);
	dangerous_areas_map_sub = n.subscribe<nav_msgs::OccupancyGrid>(dangerous_areas_map_topic, 1,
	                                                               &KugleSafety::dangerous_areas_map_callback, this);
	max_velocity_sub = n.subscribe<std_msgs::Float32>("current_max_velocity_guidee", 1, &KugleSafety::velocity_callback,
	                                                  this);

	max_vel_pub = n.advertise<std_msgs::Float32>(max_velocity_topic, 1);

	n_params.deleteParam("input_current_pose");
	n_params.deleteParam("input_dangerous_areas_map");
	n_params.deleteParam("output_max_velocity");

	ros::Rate rosRate(UPDATE_RATE);
	while (ros::ok()) {
		ros::spinOnce();
		rosRate.sleep();
	}
}

void KugleSafety::current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
	uint16_t element = static_cast<uint16_t>(floor(
			(pose->pose.pose.position.y + dangerousAreasMap.info.origin.position.y) / dangerousAreasMap.info.resolution) *
			dangerousAreasMap.info.width + floor(
			(pose->pose.pose.position.x + dangerousAreasMap.info.origin.position.x) / dangerousAreasMap.info.resolution));
	if (element >= dangerousAreasMap.info.width * dangerousAreasMap.info.height) {
		return;
	}
	
	if (dangerousAreasMap.data[element] == 100) {
		isCurrentAreaDangerous = true;
		std_msgs::Float32 maxVelocity;
		maxVelocity.data = SAFE_MAX_VEL_X;
		max_vel_pub.publish(maxVelocity);
		if (guideeVelocity < SAFE_MAX_VEL_X) {
			return;
		}
		ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_safe, srv_resp);
		ROS_INFO_STREAM("Safety: Dangerous area entered");
	} else  {
		isCurrentAreaDangerous = false;
		std_msgs::Float32 maxVelocity;
		maxVelocity.data = REGULAR_MAX_VEL_X;
		max_vel_pub.publish(maxVelocity);
		ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_regular, srv_resp);
		ROS_INFO_STREAM("Safety: Dangerous area exited");
	}
	
}

void KugleSafety::velocity_callback(const std_msgs::Float32::ConstPtr &velo)
{
	guideeVelocity = velo->data;
}

void KugleSafety::dangerous_areas_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	dangerousAreasMap = *map;
}

void KugleSafety::reconfigureSetup()
{
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config config_regular, config_safe;

//REGULAR
	double_param.name = "max_vel_x";
	double_param.value = REGULAR_MAX_VEL_X;
	config_regular.doubles.push_back(double_param);
	double_param.name = "max_vel_x_backwards";
	double_param.value = REGULAR_MAX_VEL_X_BACKWARDS;
	config_regular.doubles.push_back(double_param);
	double_param.name = "acc_lim_x";
	double_param.value = REGULAR_ACC_LIM_X;
	config_regular.doubles.push_back(double_param);
	double_param.name = "max_vel_theta";
	double_param.value = REGULAR_MAX_VEL_THETA;
	config_regular.doubles.push_back(double_param);
	double_param.name = "acc_lim_theta";
	double_param.value = REGULAR_ACC_LIM_THETA;
	config_regular.doubles.push_back(double_param);

	srv_req_regular.config = config_regular;

//SAFE
	double_param.name = "max_vel_x";
	double_param.value = SAFE_MAX_VEL_X;
	config_safe.doubles.push_back(double_param);
	double_param.name = "max_vel_x_backwards";
	double_param.value = SAFE_MAX_VEL_X_BACKWARDS;
	config_safe.doubles.push_back(double_param);
	double_param.name = "acc_lim_x";
	double_param.value = SAFE_ACC_LIM_X;
	config_safe.doubles.push_back(double_param);
	double_param.name = "max_vel_theta";
	double_param.value = SAFE_MAX_VEL_THETA;
	config_safe.doubles.push_back(double_param);
	double_param.name = "acc_lim_theta";
	double_param.value = SAFE_ACC_LIM_THETA;
	config_safe.doubles.push_back(double_param);

	srv_req_safe.config = config_safe;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety");

	KugleSafety velocityLimiter;

	return (0);
}
