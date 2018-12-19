#include "kugle_sensor_suite_gui/gui.h"
#include <pluginlib/class_list_macros.h>

namespace kugle_sensor_suite_gui
{

	gui569::gui569() : rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("gui569");
	}

	void gui569::initPlugin(qt_gui_cpp::PluginContext &context)
	{
		QStringList argv = context.argv();
		widget_ = new QWidget();
		ui_.setupUi(widget_);
		context.addWidget(widget_);

		connect(ui_.pushButton_navigate, SIGNAL(clicked()), this, SLOT(navigateClick()));
		connect(ui_.pushButton_pause, SIGNAL(toggled(bool)), this, SLOT(pauseClick(bool)));
		connect(ui_.pushButton_stop, SIGNAL(clicked()), this, SLOT(stopClick()));
		goal_sub = getNodeHandle().subscribe("/move_base/result", 1, &gui569::statusCallback, this);
		lost_guidee_pub = getNodeHandle().subscribe<std_msgs::Bool>("/lost_guidee", 1, &gui569::lostGuideeCallback, this);
		timer = getNodeHandle().createTimer(ros::Duration(UPDATE_RATE_TIMER), &gui569::timerCallback, this);

		isGuiding = false;
		isReturningHome = false;
		ui_.pushButton_pause->setEnabled(false);
		ui_.pushButton_stop->setEnabled(false);
	}

	void gui569::shutdownPlugin()
	{
		navigate_pub.shutdown();
		pause_pub.shutdown();
		stop_pub.shutdown();
	}

	void gui569::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
	{
	}

	void
	gui569::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
	{
	}

	void gui569::navigateClick()
	{
		navigate_pub = getNodeHandle().advertise<std_msgs::UInt8>("gui_navigate", 1, true);
		while (navigate_pub.getNumSubscribers() == 0) {
			ros::Duration(0.2).sleep();
		}
		if (ros::ok()) {
			std_msgs::UInt8 goal;
			goal.data = ui_.comboBox->currentIndex();
			navigate_pub.publish(goal);

			ui_.pushButton_pause->setEnabled(true);
			ui_.pushButton_pause->setChecked(false);
			ui_.pushButton_stop->setEnabled(true);

			isGuiding = true;
			isReturningHome = false;

			ui_.info_label->setText("Navigating...");
		}
	}

	void gui569::pauseClick(bool checked)
	{
		pause_pub = getNodeHandle().advertise<std_msgs::Bool>("gui_pause", 1, true);
		while (pause_pub.getNumSubscribers() == 0) {
			ros::Duration(0.2).sleep();
		}
		if (ros::ok()) {
			std_msgs::Bool click;
			click.data = checked;
			pause_pub.publish(click);
			ui_.comboBox->setEnabled(checked);
			ui_.pushButton_navigate->setEnabled(checked);

			if (checked) {
				ui_.info_label->setText("Paused!");
			} else {
				ui_.info_label->setText("Resuming...");
				ros::Duration(0.25).sleep();
				if (isReturningHome) {
					ui_.info_label->setText("Returning home...");
				} else {
					ui_.info_label->setText("Navigating...");
				}
			}
		}
	}

	void gui569::stopClick()
	{
		stop_pub = getNodeHandle().advertise<std_msgs::Bool>("gui_stop", 1, true);
		while (stop_pub.getNumSubscribers() == 0) {
			ros::Duration(0.2).sleep();
		}
		if (ros::ok()) {
			std_msgs::Bool click;
			click.data = true;
			stop_pub.publish(click);

			ui_.comboBox->setCurrentIndex(0);
			ui_.comboBox->setEnabled(false);
			ui_.pushButton_navigate->setEnabled(false);
			ui_.pushButton_pause->setEnabled(false);
			ui_.pushButton_pause->setChecked(false);
			ui_.pushButton_stop->setEnabled(false);

			isGuiding = false;

			ui_.info_label->setText("Stopping...");

			ros::Duration(DELAY_BEFORE_RETURNING_TO_DOCKING_STATION).sleep();
			isReturningHome = true;
			ui_.info_label->setText("Returning home...");
			ui_.pushButton_pause->setEnabled(true);
		}
	}

	void gui569::statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &status)
	{
		if (status->status.status == 3) {
			ui_.comboBox->setCurrentIndex(0);
			ui_.pushButton_pause->setChecked(false);
			ui_.pushButton_stop->setEnabled(false);

			if (!isGuiding) {
				ui_.pushButton_pause->setEnabled(false);
				ui_.comboBox->setEnabled(true);
				ui_.pushButton_navigate->setEnabled(true);
				ui_.info_label->setText("Select your goal:");
				isReturningHome = false;
			} else {
				ui_.comboBox->setEnabled(false);
				ui_.pushButton_navigate->setEnabled(false);
				ui_.pushButton_pause->setEnabled(false);
				ui_.info_label->setText("Goal reached!");
				ros::Duration(DELAY_BEFORE_RETURNING_TO_DOCKING_STATION).sleep();
				ui_.pushButton_pause->setEnabled(true);
				ui_.info_label->setText("Returning home...");
				isReturningHome = true;
			}

			isGuiding = false;
		}
	}

	void gui569::lostGuideeCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		ui_.comboBox->setCurrentIndex(0);
		ui_.pushButton_pause->setChecked(false);
		ui_.pushButton_stop->setEnabled(false);
		ui_.comboBox->setEnabled(false);
		ui_.pushButton_navigate->setEnabled(false);
		ui_.pushButton_pause->setEnabled(false);
		ui_.info_label->setText("Cancelling...");
		ros::Duration(DELAY_BEFORE_RETURNING_TO_DOCKING_STATION).sleep();
		ui_.info_label->setText("Returning home...");
		ui_.pushButton_pause->setEnabled(true);
		isReturningHome = true;
		isGuiding = false;
	}

	void gui569::timerCallback(const ros::TimerEvent &t)
	{
		double velocity;
		getNodeHandle().getParam("/move_base/TebLocalPlannerROS/max_vel_x", velocity);
		ui_.velocity_label->setText(QString::number(velocity, 'f', 3));
		ros::spinOnce();
	}

}

PLUGINLIB_DECLARE_CLASS(kugle_sensor_suite_gui, gui569, kugle_sensor_suite_gui::gui569, rqt_gui_cpp::Plugin
)