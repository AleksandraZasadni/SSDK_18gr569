#include "ros/ros.h"
#include "std_msgs/builtin_float.h"
#include "serial/serial.h"

#define SAMPLING_FREQUENCY 6 //Hz

int main(int argc, char **argv)
{
	ros::init(argc, argv, "maxbotix_publish");
	ros::NodeHandle n;
	ros::Rate loop_rate(SAMPLING_FREQUENCY);

	ros::NodeHandle n_params("~");
	std::string device_port, range_topic;
	n_params.param<std::string>("device_port", device_port, "ttyUSB0");
	n_params.param<std::string>("output", range_topic, "sonarRange");
	device_port = "/dev/" + device_port;

	serial::Serial sonar_serial(device_port, 57600, serial::Timeout::simpleTimeout(1000));
	if (!sonar_serial.isOpen()) {
		ROS_ERROR_STREAM("SONAR device \"" << device_port << "\" cannot be opened!");
		return (-1);
	}
	ROS_INFO_STREAM("Reading from SONAR device \"" << device_port << "\" and publishing to topic /" << range_topic << "");

	n_params.deleteParam("device_port");
	n_params.deleteParam("output");

	ros::Publisher sonar_pub = n.advertise<std_msgs::Float32>(range_topic, 1);
	std_msgs::Float32 range_msg;
	while (ros::ok()) {
		//SONAR outputs "R1234\r" each reading (in mm)
		if (sonar_serial.available() > 5) {
			if (sonar_serial.available() > 6) {
				sonar_serial.read(sonar_serial.available() - 6); //Remove stale readings
			}
			if (sonar_serial.read() != "R") {
				sonar_serial.flushInput();
				continue;
			}
			range_msg.data =
					strtof(sonar_serial.read(5).c_str(), (char **) NULL)
							/ 1000; //Reading 5 bytes -> 4 digits + '\r' character
			sonar_pub.publish(range_msg);
		}
		loop_rate.sleep();
	}

	sonar_serial.close();
	return (0);
}