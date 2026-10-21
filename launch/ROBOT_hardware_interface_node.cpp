#include <ROBOT_hardware_interface/ROBOT_hardware_interface.h>
#include <ros/callback_queue.h>
#include <robot/RobotConfig.h>
#include "robot/GlobalEnable.h"
#include <dynamic_reconfigure/server.h>
#include <unistd.h>

using namespace ROBOT_hardware_interface;

ros::Subscriber EMS_sub;

ROBOT_hardware_interface::ROBOTHardwareInterface *rhi;

pthread_t Thread;
ros::CallbackQueue ros_queue;

void DynamicReconfigureCallBack(robot::RobotConfig &config, uint32_t level)
{
    ROS_ERROR("DynamicReconfigureCallBack");
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
            usleep(100000);
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
    ros::init(argc, argv, "Zybo");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    // spinner.start();
    nh.setCallbackQueue(&ros_queue);
    rhi = new ROBOT_hardware_interface::ROBOTHardwareInterface(nh);

    EMS_sub = nh.subscribe("/globalEnable", 500, &GlobalEnableCallBack);

    dynamic_reconfigure::Server<robot::RobotConfig> DinReconfig;
    dynamic_reconfigure::Server<robot::RobotConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfigureCallBack, _1, _2);
    DinReconfig.setCallback(f);

    pthread_create(&Thread, NULL, &StartSpin, NULL);
    ros::Rate loop_rate(200);
    ros::Time prev_time = ros::Time::now();
    // ros::Rate loop_rate(200);
    // ros::spin();

    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin(&ros_queue);
    spinner.start();

    // ros::waitForShutdown();

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
    ROS_ERROR("END main.");
    return 0;
}
