#ifndef KUGLE_SENSOR_SUITE_GUI_H
#define KUGLE_SENSOR_SUITE_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <kugle_sensor_suite_gui/ui_gui.h>
#include <QWidget>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "ros/ros.h"
#include "std_msgs/builtin_bool.h"
#include "std_msgs/builtin_uint8.h"

#define UPDATE_RATE_TIMER      0.2   // s
#define DELAY_BEFORE_RETURNING_TO_DOCKING_STATION 3.0 //s

namespace kugle_sensor_suite_gui
{

	class gui569 : public rqt_gui_cpp::Plugin {
	 Q_OBJECT
	 public:
		gui569();

		virtual void initPlugin(qt_gui_cpp::PluginContext &context);

		virtual void shutdownPlugin();

		virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;

		virtual void
		restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

	 private:
		Ui::gui569Widget ui_;
		QWidget *widget_;

		ros::NodeHandle n;
		ros::Publisher navigate_pub, pause_pub, stop_pub;
		ros::Subscriber goal_sub, lost_guidee_pub;
		ros::Timer timer;

		bool isGuiding, isReturningHome;

	 public
		Q_SLOTS:

		virtual void navigateClick();

		virtual void pauseClick(bool checked);

		virtual void stopClick();

		void statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &status);

		void lostGuideeCallback(const std_msgs::Bool::ConstPtr &msg);

		void timerCallback(const ros::TimerEvent &);
	};

}
#endif  // KUGLE_SENSOR_SUITE_GUI_H
