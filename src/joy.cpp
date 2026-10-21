#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
// #include "mobile_robot_platform_v3/EMS.h"
#include "robot/GlobalEnable.h"
// #include <mobile_robot_platform_v3/joyControll.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <list>
#include <string>
#include <sstream>
#include <string>
#include <ros/callback_queue.h>
// #include "button.h"

using namespace std;

#define sign(a) (((a) < 0) ? -1 : 1)

// button bDeadMan1("DeadMan1");

// -1 a jyosticken levo indexbol.
int button_DeadMan_1 = 6;
int button_DeadMan_2 = 4;
int button_ControlMode_1 = 3;
int button_DeadMan_1_Value;
int button_DeadMan_2_Value;

int button_ControlMode_2_Value;
int button_ControlMode_2_Value_Before;

float aix0_Value = 0;
float aix1_Value = 0;

float lefterror;
float righterror;

// mobile_robot_platform_v3::SetEmergencyStopValue srv;
ros::ServiceClient client;

float actualAngularPositionArm = 0;

ros::Publisher ems_pub;
robot::GlobalEnable emsMsg;
pthread_t DeadManSwitchStateWrite;
pthread_t JoyStateStateWrite;

ros::Publisher joy_controll_pub;
geometry_msgs::Twist joyControll_msg;

geometry_msgs::Vector3 V3_linear;
geometry_msgs::Vector3 V3_angular;

ros::Publisher leftCmd_pub;
ros::Publisher rightCmd_pub;
std_msgs::Float64 leftcmd_msg;
std_msgs::Float64 rightcmd_msg;

int cntDeadMan = 0;

int ControlMode = 1;

ros::CallbackQueue ros_queue;

void LeftErrorCallback(const control_msgs::JointControllerState &msg)
{
	lefterror = msg.set_point;
}

void RightErrorCallback(const control_msgs::JointControllerState &msg)
{
	righterror = msg.set_point;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
	ROS_INFO("joyCallback start.");
	// buttomint::update(joy);
	float r = joy->axes[1];
	button_DeadMan_1_Value = joy->buttons[button_DeadMan_1];
	button_DeadMan_2_Value = joy->buttons[button_DeadMan_2];
	cntDeadMan = 0;
	button_ControlMode_2_Value = joy->buttons[button_ControlMode_1];

	if (button_ControlMode_2_Value == 1 && button_ControlMode_2_Value_Before == 0)
	{
		if (ControlMode == 1)
		{
			ControlMode = 0;
		}
		else
		{
			ControlMode = 1;
		}
	}

	button_ControlMode_2_Value_Before = button_ControlMode_2_Value;

	if (joy->buttons[3] == 1 && actualAngularPositionArm < 360)
	{
		actualAngularPositionArm += 1;
	}

	if (joy->buttons[0] == 1 && actualAngularPositionArm > -360)
	{
		actualAngularPositionArm -= 1;
	}

	if (joy->buttons[0] == 1 && actualAngularPositionArm > -360)
	{
		actualAngularPositionArm -= 1;
	}

	aix0_Value = joy->axes[0] * 0.7;
	aix1_Value = joy->axes[1];

	V3_linear.x = aix1_Value;  //
	V3_angular.z = aix0_Value; //

	float vx = ((1 - abs(aix0_Value / 2.0)) * aix1_Value);
	float vz = aix0_Value / 2.0;

	leftcmd_msg.data = vx == 0 && vz == 0 ? 0.0 : -(vz + vx) * 20;
	rightcmd_msg.data = vx == 0 && vz == 0 ? 0.0 : (-vz + vx) * 20;

	// joyControll_msg.header.stamp = ros::Time::now();
	// joyControll_msg.vx = aix1_Value;
	// joyControll_msg.omega = aix0_Value;
	// joyControll_msg.ControlMode = ControlMode;
	joyControll_msg.linear = V3_linear;
	joyControll_msg.angular = V3_angular;

	ROS_INFO("joyCallback exit.");
	// srv.request.EMSvalue = button_DeadMan_1_Value && button_DeadMan_2_Value;
	// ROS_DEBUG("Set EMS value : %d", srv.request.EMSvalue);

	// twist.linear.x = l_scale_*joy->axes[linear_];
	// vel_pub_.publish(twist);
}

void *DeadMan(void *thisPtr)
{
	ROS_INFO("DeadMan start.");
	while (true)
	{
		try
		{
			emsMsg.header.stamp = ros::Time::now();
			emsMsg.systemIsOk = (button_DeadMan_1_Value && button_DeadMan_2_Value) ? 1 : 0;
			// if(emsMsg.systemIsOk && cntDeadMan < 6)

			ems_pub.publish(emsMsg);

			cntDeadMan = cntDeadMan + 1;
		}
		catch (std::exception &ex)
		{
			ROS_ERROR("Exception on thread: %s", ex.what());
		}

		usleep(300000);
	}
	ROS_INFO("DeadMan exit.");
}

void *JoyCommand(void *thisPtr)
{
	ROS_INFO("DeadMan start.");
	while (true)
	{
		try
		{
			if (emsMsg.systemIsOk == 1)
			{
				joy_controll_pub.publish(joyControll_msg);
				leftCmd_pub.publish(leftcmd_msg);
				rightCmd_pub.publish(rightcmd_msg);
			}
		}
		catch (std::exception &ex)
		{
			ROS_ERROR("Exception on %s", ex.what());
		}

		usleep(300000);
	}
	ROS_INFO("DeadMan exit.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TeleopJoy");
	ros::AsyncSpinner spinner(4);
	ros::NodeHandle nh;

	ROS_INFO("Start TeleopJoy");

	ros::Subscriber joy_sub;
	ros::Subscriber leftError_sub;
	ros::Subscriber rightError_sub;

	sleep(1);

	joy_sub = nh.subscribe("joy", 10, &joyCallback);

	leftError_sub = nh.subscribe("/GYOM/controller/velocity/LeftWCont/state", 100, &LeftErrorCallback);
	rightError_sub = nh.subscribe("/GYOM/controller/velocity/LeftWCont/state", 100, &RightErrorCallback);

	ems_pub = nh.advertise<robot::GlobalEnable>("DeadMan", 10);

	joy_controll_pub = nh.advertise<geometry_msgs::Twist>("TeleopSig", 10);

	joy_controll_pub = nh.advertise<geometry_msgs::Twist>("TeleopSig", 10);
	joy_controll_pub = nh.advertise<geometry_msgs::Twist>("TeleopSig", 10);

	leftCmd_pub = nh.advertise<std_msgs::Float64>("LeftCmd", 10);
	rightCmd_pub = nh.advertise<std_msgs::Float64>("RightCmd", 10);

	ROS_INFO("Start thread");
	pthread_create(&DeadManSwitchStateWrite, NULL, &DeadMan, NULL);
	pthread_create(&JoyStateStateWrite, NULL, &JoyCommand, NULL);

	// client = nh.serviceClient<mobile_robot_platform_v3::SetEmergencyStopValue>("SetEMS");
	// mobile_robot_platform_v3::SetEmergencyStopValue srv;

	spinner.start();
	// ros::MultiThreadedSpinner spinner(4);
	// spinner.spin(&ros_queue);

	ros::waitForShutdown();

	while (true)
	{
		try
		{
			ros_queue.callAvailable(ros::WallDuration());
			// loop_rate.sleep();
			usleep(10000);
			// ros::spinOnce();
		}
		catch (...)
		{
			ROS_ERROR("Main error");
		}
	}
	ROS_INFO("END main.");
	return 0;
}
