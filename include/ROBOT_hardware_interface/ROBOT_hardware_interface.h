#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ROBOT_hardware_interface/CircularBuffer.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
// #include <ROBOTcpp/ROBOT.h>
#include <ROBOT_hardware_interface/ROBOT_hardware.h>

#include "robot/GlobalEnable.h"
#include <robot/RobotConfig.h>
#include "robot/TestWheel.h"
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Float64.h"
#include <angles/angles.h>
#include <string>

using namespace std;
using namespace FastFilters;

namespace ROBOT_hardware_interface
{
    static const double POSITION_STEP_FACTOR = 10;
    static const double VELOCITY_STEP_FACTOR = 10;

    class ROBOTHardwareInterface : public ROBOT_hardware_interface::ROBOTHardware
    {
    public:
        ROBOTHardwareInterface(ros::NodeHandle &nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent &e);
        void read();
        void write(ros::Duration elapsed_time);
        void dynamicReconfigureCallback(robot::RobotConfig &config, uint32_t level);
        void globalEnableCallBack(const robot::GlobalEnable &msg);
        void TestWhellJoint(float startcmd, float stopcmd, float stepcmd, void *whellHw, int sleepMs, FastFilters::AVGFilter *filter, string name, int coefMaxDuty);
        bool TestWheel_Handler(robot::TestWheel::Request &req, robot::TestWheel::Response &res);

    private:
    protected:
        // ROBOTcpp::ROBOT ROBOT;
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;

        std::shared_ptr<urdf::Model> _urdf_model;

        ros::ServiceServer TestWheelservice;

        hardware_interface::PositionJointInterface positionJointInterface;
        hardware_interface::VelocityJointInterface velocityJointInterface;
        hardware_interface::EffortJointInterface effortJointInterface;

        joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
        joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimitsInterface;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        int loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        double p_error_, v_error_, e_error_;
        void *ptrR;
        void *ptrL;
        void *bridgeR;
        void *bridgeL;
        void *incSenzR;
        void *incSenzL;
        int mem;
        double maxControlSig, coefMaxDutyR, coefMaxDutyL;
        bool globalEnable;
        int globalEnableTimeOutCnt;
        double CPUclkPeriod = 10; //[ns]

        signed long leftIncCnt, rightIncCnt, leftIncCntBefore, rightIncCntBefore;

        double leftIncCoef, rightIncCoef;

        time_t now, before;

        ros::Publisher LeftSetPoint_pub, RightSetPoint_pub, Left_state_pub, Right_state_pub;
        std_msgs::Float64 msgR, msgL;

        ros::Subscriber extControlLeft, extControlRight;
        double extControlLeftVal, extControlRightVal;

        FastFilters::AVGFilter *leftWhellSpeed;
        FastFilters::AVGFilter *rightWhellSpeed;

        float actualPeriodTime;

        double GetMaxPWM(double pwm, double coefMaxDuty);
        void Left_ExternalController_Callback(const std_msgs::Float64::ConstPtr &msg);
        void Right_ExternalController_Callback(const std_msgs::Float64::ConstPtr &msg);

        bool testMode = false;
    };

}

#endif
