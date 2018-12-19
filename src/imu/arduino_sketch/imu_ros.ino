#include <ros.h>
#include <ArduinoHardware.h>
#include <Wire.h>
#include <MPU9250.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>

MPU9250 IMU(Wire, 0x68);

ros::NodeHandle_<ArduinoHardware, 1, 1, 256, 256> n; // < Number of publishers, Number of subscribers, Output buffer size, Input buffer size>  default: <25, 25, 512, 512>
geometry_msgs::Vector3 RPY;
ros::Publisher pub("tilt_angle", &RPY);

unsigned long timer;
double deltaTime;
float accAngleX, accAngleY, tiltAngleX, tiltAngleY, tempTermX, tempTermY;

void setup()
{
	delay(5000);
	IMU.begin();

	n.initNode();
	n.advertise(pub);

	timer = micros();
}

void loop()
{
	while (micros() - timer < 20000) {} //50 Hz
	deltaTime = (micros() - timer) / 1000000.0;
	timer = micros();
	IMU.readSensor();


//  accAngleX  = atan2(IMU.getAccelY_mss(),sqrt((sq(IMU.getAccelX_mss())+sq(IMU.getAccelZ_mss())))); // Extrinsic
	accAngleX = atan2(IMU.getAccelY_mss(), IMU.getAccelZ_mss()); // Intrinsic
	accAngleY = -atan2(IMU.getAccelX_mss(), sqrt(sq(IMU.getAccelY_mss()) + sq(IMU.getAccelZ_mss())));

	tempTermX += (accAngleX - tiltAngleX) * deltaTime;
	tiltAngleX = (
			((tempTermX + ((accAngleX - tiltAngleX) * 2) + (IMU.getGyroX_rads() - IMU.getGyroBiasX_rads())) * deltaTime) +
					tiltAngleX);
	tempTermY += (accAngleY - tiltAngleY) * deltaTime;
	tiltAngleY = (
			((tempTermY + ((accAngleY - tiltAngleY) * 2) + (IMU.getGyroY_rads() - IMU.getGyroBiasY_rads())) * deltaTime) +
					tiltAngleY);

	RPY.x = tiltAngleX;
	RPY.y = tiltAngleY;
	pub.publish(&RPY);

	n.spinOnce();
}
