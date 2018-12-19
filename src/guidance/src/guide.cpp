#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/builtin_bool.h"
#include "std_msgs/builtin_uint8.h"
#include "leg_tracker/PersonArray.h"
#include "dynamic_reconfigure/DoubleParameter.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"


//#define NJV14
#define FBV7

#define ACTIVATE_KOBUKI_AUTO_DOCKING

#ifdef NJV14
const double goalOptions[][2] = {{79.125, 32.425},  // Auditorium
																 {72.5,   41.625},  // Elevator
																 {65.5,   23.3},  // Laboratory
																 {20.225, 39.475}  //Library
};
#endif

#ifdef FBV7
const double goalOptions[][2] = {{36.36, 17.92},  // Auditorium 1
                                 {25.18, 45.90},  // Elevator
                                 {46.60, 40.05},  // Auditorium 2
                                 {8.45,  44.84}    // Lounge
};
#endif
#ifdef ACTIVATE_KOBUKI_AUTO_DOCKING
#ifdef NJV14
const double dockingStation[] = {76.496864, 12.033934, 0, 0, 0, -0.204231772348, 0.978922562394}; // NJV14 with auto
#endif
#ifdef FBV7
const double dockingStation[] = {10.75, 7.27, 0, 0, 0, -0.72964, 0.68382}; // FBV_7C_1 with auto
#endif
#else
#ifdef NJV14
const double dockingStation[] = {76.8, 12.8, 0, 0, 0, 0.0357128861437, 0.999362091418}; // NJV14 direct
#endif
#ifdef FBV7
const double dockingStation[] = {0, 0, 0, 0, 0, 0, 1}; // FBV_7C_1 direct
#endif
#endif

//const double dockingStation[] = {72.5, 41.625, 0, 0, 0, 0, 1}; // one of the tests



#define DELAY_BEFORE_RETURNING_TO_DOCKING_STATION 3.0 //s

#define UPDATE_RATE                       7.5 // Hz

#define TIMEOUT_LIMIT                     5.0

#define AVERAGE_SAMPLE_COUNT              3
#define RECONFIGURE_SERVICE_CALL          "/move_base/TebLocalPlannerROS/set_parameters"

#define REGULAR_MAX_VEL_X                 0.392   // m/s
#define MAX_GUIDING_VELOCITY              0.60 // m/s
#define MIN_GUIDING_VELOCITY              0.05 // m/s

#define OPTIMAL_GUIDEE_DISTANCE           0.687   // m (0.46 + 0.177 + 0.05) [req + kobuki radius + leg radius]
#define MAX_TRACKING_DISTANCE             2.5 * OPTIMAL_GUIDEE_DISTANCE
#define MAX_TRACKING_WIDTH                1.0 // m
#define OPTIMAL_DISTANCE_VELOCITY_FACTOR  0.75
#define SPEEDING_UP_FAVOUR_FACTOR         1.35
#define GUIDEE_VELOCITY_FILTER_INCREASE_FACTOR     0.75
#define GUIDEE_VELOCITY_FILTER_DECREASE_FACTOR     0.25

class KugleGuidance {
 private:
	ros::Subscriber gui_navigate_sub, gui_pause_sub, gui_stop_sub, people_sub, max_velocity_sub, status_sub, odom_sub;
	ros::Publisher max_vel_pub, lost_guidee_pub, guidee_marker_pub, twist_pub;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	move_base_msgs::MoveBaseGoal goal;
	visualization_msgs::Marker guidee_marker;

	geometry_msgs::Pose currentOdom;
	geometry_msgs::Twist backingTwist;

	bool isGuidingActive = false, switchedID = false, timeoutBegan = false, returningToChargingStation = false, isHome = true;
	double timeoutTimer = 0, guideeVelocityTimer = 0, previousPositionX = 0, previousPositionY = 0, guideeVelocity = 0, safetyVelocity = REGULAR_MAX_VEL_X, guideeDistance = 0, previousGuideeVelocity = -1;
	unsigned int trackedID = 0;
	uint8_t counter = 0;

 public:
	KugleGuidance();

 private:
	void people_callback(const leg_tracker::PersonArray::ConstPtr &people);

	void gui_navigate_callback(const std_msgs::UInt8::ConstPtr &goalIndex);

	void gui_pause_callback(const std_msgs::Bool::ConstPtr &click);

	void gui_stop_callback(const std_msgs::Bool::ConstPtr &click);

	void velocity_callback(const std_msgs::Float32::ConstPtr &velo);

	void status_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &status);

	void odom_callback(const nav_msgs::Odometry::ConstPtr &od);

	void returnToChargingStation();

	void markerSetup();
};

KugleGuidance::KugleGuidance() : client("move_base", true)
{
	ros::NodeHandle n;

	people_sub = n.subscribe<leg_tracker::PersonArray>("people_tracked", 1, &KugleGuidance::people_callback, this);
	gui_navigate_sub = n.subscribe<std_msgs::UInt8>("kugle_sensor_suite_gui/gui_navigate", 1,
	                                                &KugleGuidance::gui_navigate_callback, this);
	gui_pause_sub = n.subscribe<std_msgs::Bool>("kugle_sensor_suite_gui/gui_pause", 1, &KugleGuidance::gui_pause_callback,
	                                            this);
	gui_stop_sub = n.subscribe<std_msgs::Bool>("kugle_sensor_suite_gui/gui_stop", 1, &KugleGuidance::gui_stop_callback,
	                                           this);
	max_velocity_sub = n.subscribe<std_msgs::Float32>("current_max_velocity_safety", 1, &KugleGuidance::velocity_callback,
	                                                  this);
	status_sub = n.subscribe("/move_base/result", 1, &KugleGuidance::status_callback, this);
	odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, &KugleGuidance::odom_callback, this);

	max_vel_pub = n.advertise<std_msgs::Float32>("current_max_velocity_guidee", 1);
	twist_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);
	guidee_marker_pub = n.advertise<visualization_msgs::Marker>("guidee_marker", 1);
	lost_guidee_pub = n.advertise<std_msgs::Bool>("/lost_guidee", 1);

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.z = 0;

	isHome = true;

	backingTwist.linear.x = -0.15;
	backingTwist.linear.y = 0;
	backingTwist.linear.z = 0;
	backingTwist.angular.x = 0;
	backingTwist.angular.y = 0;
	backingTwist.angular.z = 0;

	markerSetup();

	while (!client.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ros::Rate rosRate(UPDATE_RATE);
	while (ros::ok()) {
		ros::spinOnce();
		rosRate.sleep();
	}
}

void KugleGuidance::people_callback(const leg_tracker::PersonArray::ConstPtr &people)
{
	if (!isGuidingActive) {
		return;
	}

	geometry_msgs::Point personInOdomFrame;

	if (trackedID != 0) {
		bool trackedPersonFoundAgain = false;
		for (const auto &person : people->people) {
			if (trackedID == person.id) {
				personInOdomFrame = person.pose.position;

				guideeDistance = sqrt(pow(person.pose.position.x - currentOdom.position.x, 2) +
						                      pow(person.pose.position.y - currentOdom.position.y, 2));

				if (guideeDistance > MAX_TRACKING_DISTANCE) {
					trackedID = 0;
					break;
				}

				trackedPersonFoundAgain = true;

				guidee_marker.pose = person.pose;
				guidee_marker.header.stamp = ros::Time();
				guidee_marker_pub.publish(guidee_marker);

				break;
			}
		}
		if (!trackedPersonFoundAgain) {
			trackedID = 0;
		}
	}

	if (trackedID == 0) {
		double shortestDistance = MAX_TRACKING_DISTANCE;
		for (const auto &person : people->people) {
			double distance = sqrt(pow(person.pose.position.x - currentOdom.position.x, 2) +
					                       pow(person.pose.position.y - currentOdom.position.y, 2));

			if (distance < MAX_TRACKING_DISTANCE) {
				tf::Quaternion q(
						person.pose.orientation.x,
						person.pose.orientation.y,
						person.pose.orientation.z,
						person.pose.orientation.w);
				tf::Matrix3x3 m(q);
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);

				if ((cos(yaw) * (person.pose.position.x - currentOdom.position.x) < 0) &&
						(fabs(sin(yaw) * (person.pose.position.y - currentOdom.position.y)) < (0.5 * MAX_TRACKING_WIDTH))) {
					if (distance < shortestDistance) {
						trackedID = person.id;
						timeoutBegan = false;
						switchedID = true;
					}
				}
			}
		}
	}

	if (trackedID == 0) {
		if (!timeoutBegan) {
			timeoutTimer = ros::Time::now().toSec();

			dynamic_reconfigure::ReconfigureRequest srv_req_lost_guidee_velocity;
			dynamic_reconfigure::DoubleParameter double_param;
			dynamic_reconfigure::Config config;

			double_param.name = "max_vel_x";
			double_param.value = MIN_GUIDING_VELOCITY;
			std_msgs::Float32 maxVelocity;
			maxVelocity.data = MIN_GUIDING_VELOCITY;
			max_vel_pub.publish(maxVelocity);
			config.doubles.push_back(double_param);
			double_param.value /= 2;
			double_param.name = "max_vel_x_backwards";
			config.doubles.push_back(double_param);

			srv_req_lost_guidee_velocity.config = config;
			ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_lost_guidee_velocity, srv_resp);

			timeoutBegan = true;
		}

		if (ros::Time::now().toSec() - timeoutTimer > TIMEOUT_LIMIT) {
			std_msgs::Bool msg;
			lost_guidee_pub.publish(msg);
			returnToChargingStation();
		}

		return;
	}

	if (!switchedID) {
		counter++;
		if (counter >= AVERAGE_SAMPLE_COUNT) {

			guideeVelocity = sqrt(pow(personInOdomFrame.x - previousPositionX, 2) +
					                      pow(personInOdomFrame.y - previousPositionY, 2)) /
					(ros::Time::now().toSec() - guideeVelocityTimer);
			guideeVelocityTimer = ros::Time::now().toSec();

			guideeVelocity +=
					(SPEEDING_UP_FAVOUR_FACTOR * OPTIMAL_GUIDEE_DISTANCE - guideeDistance) * OPTIMAL_DISTANCE_VELOCITY_FACTOR;

			double newVelocity;
			if (guideeVelocity >= MAX_GUIDING_VELOCITY) {
				newVelocity = MAX_GUIDING_VELOCITY;
			} else if (guideeVelocity <= MIN_GUIDING_VELOCITY) {
				newVelocity = MIN_GUIDING_VELOCITY;
			} else {
				newVelocity = guideeVelocity;
			}

			dynamic_reconfigure::ReconfigureRequest srv_req_adjust_guidee_velocity;
			dynamic_reconfigure::DoubleParameter double_param;
			dynamic_reconfigure::Config config;

			if (previousGuideeVelocity != -1) {
				if (double_param.value > previousGuideeVelocity) {
					double_param.value = GUIDEE_VELOCITY_FILTER_INCREASE_FACTOR * newVelocity +
							(1 - GUIDEE_VELOCITY_FILTER_INCREASE_FACTOR) * previousGuideeVelocity;
				} else {
					double_param.value = GUIDEE_VELOCITY_FILTER_DECREASE_FACTOR * newVelocity +
							(1 - GUIDEE_VELOCITY_FILTER_DECREASE_FACTOR) * previousGuideeVelocity;
				}
			} else {
				double_param.value = newVelocity;
			}

			double_param.name = "max_vel_x";
			config.doubles.push_back(double_param);
			
			counter = 0;
			previousPositionX = personInOdomFrame.x;
			previousPositionY = personInOdomFrame.y;
			
			std_msgs::Float32 maxVelocity;
			if ((safetyVelocity < REGULAR_MAX_VEL_X) && (double_param.value >= safetyVelocity)) {
				guideeVelocityTimer = ros::Time::now().toSec();
				previousGuideeVelocity = -1;
				maxVelocity.data = REGULAR_MAX_VEL_X;
				max_vel_pub.publish(maxVelocity);
				return;
			}
			previousGuideeVelocity = double_param.value;
			maxVelocity.data = double_param.value;
			max_vel_pub.publish(maxVelocity);
			
			double_param.value /= 2;
			double_param.name = "max_vel_x_backwards";
			config.doubles.push_back(double_param);
			
			srv_req_adjust_guidee_velocity.config = config;
			ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_adjust_guidee_velocity, srv_resp);
		}
	} else {
		guideeVelocityTimer = ros::Time::now().toSec();
		previousPositionX = personInOdomFrame.x;
		previousPositionY = personInOdomFrame.y;
		previousGuideeVelocity = -1;
		counter = 0;
		switchedID = false;
	}
}

void KugleGuidance::status_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &status)
{

	if (status->status.status == status->status.SUCCEEDED) {

		if (isGuidingActive) {
			returnToChargingStation();
		} else {
			dynamic_reconfigure::DoubleParameter double_param;
			dynamic_reconfigure::ReconfigureRequest srv_req_docking_finished;
			dynamic_reconfigure::Config config_guiding;

			double_param.name = "xy_goal_tolerance";
			double_param.value = 0.1;
			config_guiding.doubles.push_back(double_param);
			double_param.name = "yaw_goal_tolerance";
			double_param.value = 6.28318531;
			config_guiding.doubles.push_back(double_param);

			srv_req_docking_finished.config = config_guiding;

			ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_docking_finished, srv_resp);

			returningToChargingStation = false;

#ifdef ACTIVATE_KOBUKI_AUTO_DOCKING
			system("roslaunch kobuki_auto_docking activate.launch");
#endif

			isHome = true;
		}
	}
}

void KugleGuidance::gui_navigate_callback(const std_msgs::UInt8::ConstPtr &goalIndex)
{
	if (isHome) {
		for (int i = 0; i < 8; ++i) {
			twist_pub.publish(backingTwist);
			ros::Duration(0.3).sleep();
		}
		isHome = false;
	}

	isGuidingActive = true;
	returningToChargingStation = false;

	goal.target_pose.pose.position.x = goalOptions[goalIndex->data][0];
	goal.target_pose.pose.position.y = goalOptions[goalIndex->data][1];
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1;
	goal.target_pose.header.stamp = ros::Time::now();

	client.sendGoal(goal);
}

void KugleGuidance::gui_pause_callback(const std_msgs::Bool::ConstPtr &click)
{
	if (click->data) {
		isGuidingActive = false;
		client.cancelAllGoals();

		if (returningToChargingStation) {
			dynamic_reconfigure::DoubleParameter double_param;
			dynamic_reconfigure::ReconfigureRequest srv_req_docking_finished;
			dynamic_reconfigure::Config config_guiding;

			double_param.value = REGULAR_MAX_VEL_X;
			double_param.name = "max_vel_x";
			std_msgs::Float32 maxVelocity;
			maxVelocity.data = REGULAR_MAX_VEL_X;
			max_vel_pub.publish(maxVelocity);
			config_guiding.doubles.push_back(double_param);
			double_param.value /= 2;
			double_param.name = "max_vel_x_backwards";
			config_guiding.doubles.push_back(double_param);
			double_param.name = "xy_goal_tolerance";
			double_param.value = 0.1;
			config_guiding.doubles.push_back(double_param);
			double_param.name = "yaw_goal_tolerance";
			double_param.value = 6.28318531;
			config_guiding.doubles.push_back(double_param);

			srv_req_docking_finished.config = config_guiding;

			ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_docking_finished, srv_resp);
		}

	} else {
		if (returningToChargingStation) {
			returnToChargingStation();
		} else {
			isGuidingActive = true;
			goal.target_pose.header.stamp = ros::Time::now();
			client.sendGoal(goal);
		}
	}
}

void KugleGuidance::gui_stop_callback(const std_msgs::Bool::ConstPtr &click)
{
	returnToChargingStation();
}

void KugleGuidance::velocity_callback(const std_msgs::Float32::ConstPtr &velo)
{
	safetyVelocity = velo->data;
}

void KugleGuidance::returnToChargingStation()
{
	isGuidingActive = false;
	timeoutBegan = false;
	returningToChargingStation = true;

	guidee_marker.action = visualization_msgs::Marker::DELETEALL;
	guidee_marker_pub.publish(guidee_marker);
	guidee_marker.action = visualization_msgs::Marker::ADD;

	client.cancelAllGoals();
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::ReconfigureRequest srv_req_docking;
	dynamic_reconfigure::Config config;

	double_param.value = REGULAR_MAX_VEL_X;
	double_param.name = "max_vel_x";
	std_msgs::Float32 maxVelocity;
	maxVelocity.data = REGULAR_MAX_VEL_X;
	max_vel_pub.publish(maxVelocity);
	config.doubles.push_back(double_param);

	double_param.name = "xy_goal_tolerance";
	double_param.value = 0.05;
	config.doubles.push_back(double_param);
	double_param.name = "yaw_goal_tolerance";
	double_param.value = 0.174533;
	config.doubles.push_back(double_param);

	srv_req_docking.config = config;

	ros::service::call(RECONFIGURE_SERVICE_CALL, srv_req_docking, srv_resp);

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x = dockingStation[0];
	goal.target_pose.pose.position.y = dockingStation[1];
	goal.target_pose.pose.orientation.x = dockingStation[3];
	goal.target_pose.pose.orientation.y = dockingStation[4];
	goal.target_pose.pose.orientation.z = dockingStation[5];
	goal.target_pose.pose.orientation.w = dockingStation[6];

	ros::Duration(DELAY_BEFORE_RETURNING_TO_DOCKING_STATION).sleep();
	goal.target_pose.header.stamp = ros::Time::now();
	client.sendGoal(goal);
}

void KugleGuidance::markerSetup()
{
	guidee_marker.header.frame_id = "odom";
	guidee_marker.ns = "guidee_marker";
	guidee_marker.id = 0;
	guidee_marker.type = visualization_msgs::Marker::CYLINDER;
	guidee_marker.action = visualization_msgs::Marker::ADD;
	guidee_marker.scale.x = 0.2;
	guidee_marker.scale.y = 0.3;
	guidee_marker.scale.z = 1.63;
	guidee_marker.pose.position.z += guidee_marker.scale.z / 2;
	guidee_marker.color.a = 1.0;
	guidee_marker.color.r = 0.5568627;
	guidee_marker.color.g = 0.6745098;
	guidee_marker.color.b = 0.7137254;
}

void KugleGuidance::odom_callback(const nav_msgs::Odometry::ConstPtr &od)
{
	currentOdom = od->pose.pose;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "guide");

	KugleGuidance digitalGuide;

	return (0);
}
