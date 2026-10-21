#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string>
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <urdf/model.h>
#include <iostream>
#include <fstream>
#include <robot/RobotConfig.h>
#include "robot/TestWheel.h"
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ROBOT_hardware_interface/ROBOT_hardware_interface.h>
#include <ROBOT_hardware_interface/CircularBuffer.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace std;
using namespace FastFilters;

#define PI 3.141592653589793238463

#define pwmR 0x43C40000
#define pwmL 0x43C50000

#define HbridgeR 0x43C90000
#define HbridgeL 0x43C80000

#define IncSensR 0x43C60000
#define IncSensL 0x43C70000

namespace ROBOT_hardware_interface
{
    ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
    {
        init();

        ROS_INFO("Init Contreoller Manager");
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

        nh_.getParam(ros::this_node::getName() + "/loop_hz", loop_hz_);
        ROS_INFO("Loot hz %i", loop_hz_);

        ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

        ROS_INFO("Init Timer Update");
        non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);

        ROS_INFO("Creat topic publishers");
        LeftSetPoint_pub = nh_.advertise<std_msgs::Float64>("LeftSetPoint", 1000);
        RightSetPoint_pub = nh_.advertise<std_msgs::Float64>("RightSetPoint", 1000);
        Left_state_pub = nh_.advertise<std_msgs::Float64>("LeftState", 1000);
        Right_state_pub = nh_.advertise<std_msgs::Float64>("RightState", 1000);

        ROS_INFO("Creat topic subscribers");
        extControlLeft = nh_.subscribe("LeftControlSig", 1000, &ROBOTHardwareInterface::Left_ExternalController_Callback, this);
        extControlRight = nh_.subscribe("RightControlSig", 1000, &ROBOTHardwareInterface::Right_ExternalController_Callback, this);

        ROS_INFO("Creat service");
        TestWheelservice = nh_.advertiseService("TestWheel", &ROBOTHardwareInterface::TestWheel_Handler, this);

        ROS_INFO("Creat filters for incremental sensors.");
        leftWhellSpeed = new AVGFilter(10);
        rightWhellSpeed = new AVGFilter(10);
        testMode = false;
    }

    bool ROBOTHardwareInterface::TestWheel_Handler(robot::TestWheel::Request &req, robot::TestWheel::Response &res)
    {
        ROS_INFO("Begin: TestWheel : %s", req.nameWheel);
        testMode = true;
        if (req.nameWheel == "left")
        {
            TestWhellJoint(req.startCmd, req.stopCmd, req.stepCmd, ptrL, req.waitMs, leftWhellSpeed, req.nameWheel, coefMaxDutyL);
        }

        if (req.nameWheel == "right")
        {
            TestWhellJoint(req.startCmd, req.stopCmd, req.stepCmd, ptrR, req.waitMs, rightWhellSpeed, req.nameWheel, coefMaxDutyR);
        }

        res.status = "OK";
        testMode = false;

        ROS_INFO("End: TestWheel : %s", req.nameWheel);

        return true;
    }

    ROBOTHardwareInterface::~ROBOTHardwareInterface()
    {
    }

    void ROBOTHardwareInterface::Left_ExternalController_Callback(const std_msgs::Float64::ConstPtr &msg)
    {
        ROS_DEBUG("Left: [%f]", msg->data);
        extControlLeftVal = msg->data;
    }

    void ROBOTHardwareInterface::Right_ExternalController_Callback(const std_msgs::Float64::ConstPtr &msg)
    {
        ROS_DEBUG("Right: [%f]", msg->data);
        extControlRightVal = msg->data;
    }

    void ROBOTHardwareInterface::init()
    {

        ROS_INFO("Init");

        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them

        // Direct memory access pointers.
        {
            ROS_INFO("Init direct mem access pointes.");
            mem = open("/dev/mem", O_RDWR | O_SYNC);
            if (mem < 0)
            {
                ROS_ERROR("Error open /dev/mmem");
            }

            ptrR = mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, mem, pwmR);
            if (ptrR == MAP_FAILED)
            {
                ROS_ERROR("Error open ptrR");
            }

            ptrL = mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, mem, pwmL);
            if (ptrR == MAP_FAILED)
            {
                ROS_ERROR("Error open ptrL");
            }

            incSenzR = mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, mem, IncSensR);
            if (incSenzR == MAP_FAILED)
            {
                ROS_ERROR("Error open incSenzR");
            }

            incSenzL = mmap(0, 0x2000, PROT_READ | PROT_WRITE, MAP_SHARED, mem, IncSensL);
            if (incSenzL == MAP_FAILED)
            {
                ROS_ERROR("Error open incSenzL");
            }
        }

        rightIncCoef = 1;
        leftIncCoef = 1;
        maxControlSig = 30000;

        // FPGA IP block init.
        {
            ROS_INFO("Init FPGA IPs");
            // Pwm Right
            *(volatile int *)(ptrR + PWM_HARDVER_ENABLE) = 0xFFFFFFFF;
            *(volatile int *)(ptrR + PWM_HARDVER_ENABLE) = 0x00000000;
            *(volatile unsigned long *)(ptrR + PWM_HARDVER_FREQ) = 10000;
            *(volatile int *)(ptrR + PWM_HARDVER_DIV) = 0x01;

            // Pwm Left
            *(volatile int *)(ptrL + PWM_HARDVER_ENABLE) = 0xFFFFFFFF;
            *(volatile int *)(ptrL + PWM_HARDVER_ENABLE) = 0x00000000;
            *(volatile unsigned long *)(ptrL + PWM_HARDVER_FREQ) = 10000;
            *(volatile int *)(ptrL + PWM_HARDVER_DIV) = 0x01;

            // Right encoder
            *(volatile signed long *)(incSenzR + POSITIONCOUNTERSIMPLE_SETRESETVALUE) = 0;     // reset value
            *(volatile signed long *)(incSenzR + POSITIONCOUNTERSIMPLE_SETCOUNTER) = 0;        // load value
            *(volatile unsigned long *)(incSenzR + POSITIONCOUNTERSIMPLE_LOADVALUE) = 0;       // start load rising edge
            *(volatile unsigned long *)(incSenzR + POSITIONCOUNTERSIMPLE_COUNT_DIRECTION) = 0; // count direction

            // Left encoder
            *(volatile signed long *)(incSenzL + POSITIONCOUNTERSIMPLE_SETRESETVALUE) = 0;     // reset value
            *(volatile signed long *)(incSenzL + POSITIONCOUNTERSIMPLE_SETCOUNTER) = 0;        // load value
            *(volatile unsigned long *)(incSenzL + POSITIONCOUNTERSIMPLE_LOADVALUE) = 0;       // start load rising edge
            *(volatile unsigned long *)(incSenzL + POSITIONCOUNTERSIMPLE_COUNT_DIRECTION) = 0; // count direction
        }

        // Hardver joints
        {
            ROS_INFO("Init robot joints");

            // Get joint names
            nh_.getParam(ros::this_node::getNamespace() + "/hardware_interface/joints", joint_names_);
            num_joints_ = joint_names_.size();
            ROS_INFO("Number of jonts %i: ", num_joints_);

            // Resize vectors
            joint_position_.resize(num_joints_);
            joint_velocity_.resize(num_joints_);
            joint_effort_.resize(num_joints_);
            joint_position_command_.resize(num_joints_);
            joint_velocity_command_.resize(num_joints_);
            joint_effort_command_.resize(num_joints_);

            // Initialize Controller
            for (int i = 0; i < num_joints_; ++i)
            {
                // ROBOTcpp::Joint joint = ROBOT.getJoint(joint_names_[i]);

                ROS_INFO("register join name: %s", joint_names_[i].data());

                // Create joint state interface
                hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
                joint_state_interface_.registerHandle(jointStateHandle);

                hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
                effort_joint_interface_.registerHandle(jointEffortHandle);

                joint_limits_interface::JointLimits limits;
                joint_limits_interface::SoftJointLimits softLimits;

                joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);

                try
                {
                    std::string urdf_string;
                    if (_urdf_model == nullptr)
                        _urdf_model = std::make_shared<urdf::Model>();

                    // get the urdf param on param server
                    std::string urdf_param_name("robot_description");
                    nh_.getParam(urdf_param_name, urdf_string);

                    _urdf_model->initString(urdf_string);

                    urdf::JointConstSharedPtr urdf_joint = _urdf_model->getJoint(joint_names_[i]);

                    joint_limits_interface::getSoftJointLimits(urdf_joint, softLimits);

                    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandlev(jointEffortHandle, limits); // itt all fejre
                    velocityJointSaturationInterface.registerHandle(jointLimitsHandlev);

                    joint_limits_interface::VelocityJointSoftLimitsHandle jointSoftLimitsHandlev(jointEffortHandle, limits, softLimits); // itt all fejre
                    velocityJointSoftLimitsInterface.registerHandle(jointSoftLimitsHandlev);

                    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits); // itt all fejre
                    effortJointSaturationInterface.registerHandle(jointLimitsHandle);

                    joint_limits_interface::EffortJointSoftLimitsHandle jointSoftLimitsHandle(jointEffortHandle, limits, softLimits); // itt all fejre
                    effortJointSoftLimitsInterface.registerHandle(jointSoftLimitsHandle);
                }
                catch (std::exception &e)
                {
                    std::exception_ptr p = std::current_exception();
                    ROS_ERROR("Error in catch %s", e.what());
                }
            }

            registerInterface(&joint_state_interface_);
            registerInterface(&velocity_joint_interface_);
            registerInterface(&effort_joint_interface_);
            registerInterface(&effortJointSaturationInterface);
            registerInterface(&effortJointSoftLimitsInterface);
            registerInterface(&velocityJointSaturationInterface);
        }

        ROS_INFO("Init end");
    }

    void ROBOTHardwareInterface::update(const ros::TimerEvent &e)
    {
        ROS_DEBUG("Begin update callback");

        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
        ROS_DEBUG("End update callback");
    }

    void ROBOTHardwareInterface::read()
    {
        ROS_DEBUG("Begin read registers, sensors ... callback");

        rightIncCntBefore = rightIncCnt;
        leftIncCntBefore = leftIncCnt;
        leftIncCnt = *(volatile signed long *)(incSenzL + POSITIONCOUNTERSIMPLE_ANGULARPOSITIONAXI);
        rightIncCnt = *(volatile signed long *)(incSenzR + POSITIONCOUNTERSIMPLE_ANGULARPOSITIONAXI);

        double sumleft = 0;
        double sumright = 0;
        actualPeriodTime = elapsed_time_.nsec / 1000000000.0;

        ROS_DEBUG("Period [sec] = %f", actualPeriodTime);

        leftWhellSpeed->AddElement(((leftIncCnt - leftIncCntBefore) * leftIncCoef) / (actualPeriodTime * 2 * PI));
        rightWhellSpeed->AddElement(((rightIncCnt - rightIncCntBefore) * rightIncCoef) / (actualPeriodTime * 2 * PI));

        double leftRoad = 4.0;         //((double)(leftIncCnt*leftIncCoef))/((double)(2*PI));
        double rightRoad = 4.0;        //((double)(rightIncCnt*rightIncCoef))/((double)(2*PI));
                                       // velocity [rad/s]
        joint_position_[1] = leftRoad; // leftWhellSpeed->GetAvg();
        joint_velocity_[1] = leftWhellSpeed->GetAvg();
        joint_effort_[1] = 0.0;         // leftWhellSpeed->GetAvg();
        joint_position_[0] = rightRoad; // rightWhellSpeed->GetAvg();
        joint_velocity_[0] = rightWhellSpeed->GetAvg();
        joint_effort_[0] = 0.0; // rightWhellSpeed->GetAvg();

        ROS_DEBUG("End read registers, sensors ... callback");
    }

    double ROBOTHardwareInterface::GetMaxPWM(double pwm, double coefMaxDuty)
    {
        ROS_DEBUG("Begin GetMaxPWM %d", pwm);
        /*if(pwm > 0.05)
        {
            pwm = pwm + 0.1* pwm;
        }
        if(pwm < 0.05)
        {
            pwm = pwm + (-0.1)* pwm;
        }*/

        if (pwm > 0.99)
        {
            return 0.99 * coefMaxDuty;
        }

        if (pwm < -0.99)
        {
            return -0.99 * coefMaxDuty;
        }

        double result = pwm * coefMaxDuty;
        ROS_DEBUG("End GetMaxPWM %d", result);
        return result;
    }

    void ROBOTHardwareInterface::dynamicReconfigureCallback(robot::RobotConfig &config, uint32_t level)
    {
        ROS_INFO("Begin, dynamicReconfigureCallback");

        *(volatile unsigned long *)(ptrR + PWM_HARDVER_FREQ) = (int)((1.0 / config.PwmFreqventi_R) * 1000000000.0 / CPUclkPeriod);
        *(volatile unsigned long *)(ptrL + PWM_HARDVER_FREQ) = (int)((1.0 / config.PwmFreqventi_L) * 1000000000.0 / CPUclkPeriod);

        int R = *(volatile unsigned long *)(ptrR + PWM_HARDVER_FREQ);
        int L = *(volatile unsigned long *)(ptrL + PWM_HARDVER_FREQ);

        ROS_INFO("RpwmFreqReq %d, LpwmFreqReq %d", R, L);

        coefMaxDutyR = abs(R);
        coefMaxDutyL = abs(L);
        ROS_INFO("Max PWM duty coef: PWMR=%d, PWML=%d", coefMaxDutyR, coefMaxDutyL);

        ROS_INFO("PWM div:");

        if (config.HbridgeCharge_R >= 0 && config.HbridgeCharge_R < 16)
        {
            *(volatile unsigned long *)(ptrR + PWM_HARDVER_DIV) = (int)config.HbridgeCharge_R;
            ROS_INFO("Left pwm div %d", *(volatile unsigned long *)(ptrR + PWM_HARDVER_DIV));
        }
        if (config.HbridgeCharge_L >= 0 && config.HbridgeCharge_L < 16)
        {
            *(volatile unsigned long *)(ptrL + PWM_HARDVER_DIV) = (int)config.HbridgeCharge_L;
            ROS_INFO("Left pwm div %d", *(volatile unsigned long *)(ptrL + PWM_HARDVER_DIV));
        }

        *(volatile unsigned long *)(incSenzL + POSITIONCOUNTERSIMPLE_COUNT_DIRECTION) = (int)config.IncrementalSensorDirection_L;
        *(volatile unsigned long *)(incSenzR + POSITIONCOUNTERSIMPLE_COUNT_DIRECTION) = (int)config.IncrementalSensorDirection_R;
        ROS_INFO("Left sensor direction: %d", config.IncrementalSensorDirection_L);
        ROS_INFO("Right sensor direction: %d", config.IncrementalSensorDirection_R);

        rightIncCoef = 2 * PI / config.IncrementalSensorResolution_R;
        leftIncCoef = 2 * PI / config.IncrementalSensorResolution_L;
        ROS_INFO("Inc sensor resolution sensor right: %d, sensor left: %d", config.IncrementalSensorResolution_R, config.IncrementalSensorResolution_L);

        ROS_INFO("Left AVG flter cnt: %d", config.IncSenVelocityAvgFilterSize_L);
        leftWhellSpeed = new AVGFilter(config.IncSenVelocityAvgFilterSize_L);

        ROS_INFO("Right AVG flter cnt: %d", config.IncSenVelocityAvgFilterSize_R);
        rightWhellSpeed = new AVGFilter(config.IncSenVelocityAvgFilterSize_R);

        ROS_INFO("End dynamicReconfigureCallback");
    }

    void ROBOTHardwareInterface::globalEnableCallBack(const robot::GlobalEnable &msg)
    {
        ROS_DEBUG("Begin, globalEnableCallBack value : [%d]", msg.systemIsOk);

        globalEnable = msg.systemIsOk;
        globalEnableTimeOutCnt = 0;
        if (globalEnable == false)
        {
            *(volatile unsigned long *)(ptrL + PWM_HARDVER_ENABLE) = 0x00000000;
            *(volatile unsigned long *)(ptrR + PWM_HARDVER_ENABLE) = 0x00000000;
        }

        ROS_DEBUG("End globalEnableCallBack");
    }

    void ROBOTHardwareInterface::TestWhellJoint(float startcmd, float stopcmd, float stepcmd, void *whellHw, int sleepMs, FastFilters::AVGFilter *filter, string name, int coefMaxDuty)
    {
        ROS_INFO("Begin TestWhellJoint");
        if (!testMode)
        {
            ROS_WARN("The test mode is not enabled");
            return;
        }

        std::string path = ros::package::getPath("robot");
        ofstream myfile;
        myfile.open(path + "/" + name + ".out");
        ROS_INFO("Path: %s/%s.out", path.data(), name.data());

        int cntr = 0, cmd = 0;
        for (float i = startcmd; i <= stopcmd; i = i + stepcmd)
        {
            cmd = GetMaxPWM(i, coefMaxDuty);
            *(volatile unsigned long *)(whellHw + PWM_HARDVER_ENABLE) = 0xFFFFFFFF;
            *(volatile int *)(whellHw + PWM_HARDVER_DUTY_SW) = cmd;

            usleep(sleepMs * 1000);

            myfile << cntr << ";" << i << ";" << filter->GetAvg() << "\n";

            cntr++;
        }
        *(volatile int *)(whellHw + PWM_HARDVER_DUTY_SW) = GetMaxPWM(0.0, coefMaxDuty);
        ;
        myfile.close();
        ROS_INFO("End TestWhellJoint");
    }

    void ROBOTHardwareInterface::write(ros::Duration elapsed_time)
    {

        ROS_DEBUG("Begin write registes, actuators ....");
        if (testMode)
        {
            ROS_WARN("In test mode, will return!");
            return;
        }

        if (!globalEnable)
        {
            *(volatile unsigned long *)(ptrR + PWM_HARDVER_ENABLE) = 0x00000000;
            *(volatile unsigned long *)(ptrL + PWM_HARDVER_ENABLE) = 0x00000000;
            return;
        }

        for (int i = 0; i < num_joints_; i++)
        {
            // ROBOT.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
        }

        joint_effort_command_[1] = GetMaxPWM(joint_effort_command_[1], coefMaxDutyL);
        joint_effort_command_[0] = GetMaxPWM(joint_effort_command_[0], coefMaxDutyR);
        ROS_INFO("Lcmd0 %f, Rcmd1 %f", joint_effort_command_[1], joint_effort_command_[0]);

        *(volatile unsigned long *)(ptrR + PWM_HARDVER_ENABLE) = 0xFFFFFFFF;
        *(volatile unsigned long *)(ptrL + PWM_HARDVER_ENABLE) = 0xFFFFFFFF;
        *(volatile int *)(ptrL + PWM_HARDVER_DUTY_SW) = joint_effort_command_[1];
        *(volatile int *)(ptrR + PWM_HARDVER_DUTY_SW) = joint_effort_command_[0];

        globalEnableTimeOutCnt += globalEnableTimeOutCnt + 1;

        ROS_DEBUG("End write registes, actuators ....");
    }
}
