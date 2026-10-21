#include <unistd.h>
#include <ros/callback_queue.h>
#include <robot/RobotConfig.h>
#include "robot/GlobalEnable.h"
#include <dynamic_reconfigure/server.h>
#include <ROBOT_hardware_interface/ROBOT_hardware_interface.h>

using namespace ROBOT_hardware_interface;

ros::Subscriber EMS_sub;

ROBOT_hardware_interface::ROBOTHardwareInterface *rhi;

pthread_t Thread;

ros::CallbackQueue ros_queue;

void DynamicReconfigureCallBack(robot::RobotConfig &config, uint32_t level)
{
    ROS_INFO("DynamicReconfigureCallBack");
    rhi->dynamicReconfigureCallback(config, level);
}

void GlobalEnableCallBack(const robot::GlobalEnable &msg)
{
    rhi->globalEnableCallBack(msg);
}

void *StartSpin(void *param)
{
    while (true)
    {
        try
        {
            // loop_rate.sleep();
            usleep(10000);
            // ros::spinOnce();
        }
        catch (...)
        {
            ROS_ERROR("Main error");
        }
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Begin main.");
    ros::init(argc, argv, "Zybo");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);

    rhi = new ROBOT_hardware_interface::ROBOTHardwareInterface(nh);

    EMS_sub = nh.subscribe("GlobalEnable", 500, &GlobalEnableCallBack);

    dynamic_reconfigure::Server<robot::RobotConfig> DinReconfig;
    dynamic_reconfigure::Server<robot::RobotConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfigureCallBack, _1, _2);
    DinReconfig.setCallback(f);

    pthread_create(&Thread, NULL, &StartSpin, NULL);

    spinner.start();

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
