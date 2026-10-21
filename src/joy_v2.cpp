#include <ros/ros.h>
#include <pthread.h>
#include <functional>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "robot/GlobalEnable.h"
#include <robot/RobotJoyConfig.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <control_msgs/JointControllerState.h>

#define sign(a) ((a) < 0 ? -1 : 1)
#define cntOfButons  10

using namespace std;

int button_DeadMan_1_Value;
int button_DeadMan_2_Value;

float aix0_Value = 0;
float aix1_Value = 0;

float lefterror;
float righterror;


float maxVx = 0.0;
float maxDelta = 0.0;

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
robot::RobotJoyConfig configJoy;


std::function<void(bool)> functionPointers[cntOfButons];
int buttonsCopy[cntOfButons];

void deadManA(int actValue) {
    button_DeadMan_1_Value = actValue;
}

void deadManB(int actValue) {
    button_DeadMan_2_Value = actValue;
}

void controlMode(int actValue)
{
    int ControlMode = (actValue == 1) ? 0 : 1;
}

float changeValue(float actual,float inc, float config)
{
    float temp = 0.0;
    float out = actual;
    if(config > 0)
    {
        temp = actual + inc;
        if(temp > 0 && temp <= config)
        {
            out = temp;
        }
    }
    else
    {
        temp = actual - inc;
        if(temp < 0 && temp >= config)
        {
            out = temp;
        }
    }
    return out;
}

void vxUp(int actValue)
{
    if(actValue == 1){
        maxVx = changeValue(maxVx,1,configJoy.maxOutputValue);
        ROS_INFO("Max vx speed changed to: %f", maxVx);
    }
}

void vxDown(int actValue)
{
    if(actValue == 1){
        maxVx = changeValue(maxVx,-1,configJoy.maxOutputValue);
        ROS_INFO("Max vx speed changed to: %f", maxVx);
    }
}

void deltaUp(int actValue)
{
    if(actValue == 1){
        maxDelta = changeValue(maxDelta,0.1,configJoy.angularZ_scale);
        ROS_INFO("Max dela speed changed to: %f", maxDelta);
    }
}

void deltaDown(int actValue)
{   
    if(actValue == 1){
        maxDelta = changeValue(maxDelta,-0.1,configJoy.angularZ_scale);
        ROS_INFO("Max dela speed changed to: %f", maxDelta);
    }
}

void updateButtonCallBackFunctions()
{
    ROS_INFO("Updating button handler functons.");
    functionPointers[configJoy.controlMode_button-1] = {controlMode};
    functionPointers[configJoy.deadManA_button-1] = {deadManA};
    functionPointers[configJoy.deadManB_button-1] = {deadManB};
    
    functionPointers[configJoy.VdeltaUp_button-1] = {deltaUp};
    functionPointers[configJoy.VdeltaDown_button-1] = {deltaDown};
    
    functionPointers[configJoy.VxUp_button-1] = {vxUp};
    functionPointers[configJoy.VxDown_button-1] = {vxDown};
}

void DynamicReconfigureCallBack(robot::RobotJoyConfig &config,uint32_t level) {
    ROS_INFO("DynamicReconfigureCallBack");
	configJoy = config;
    updateButtonCallBackFunctions();
    maxVx = configJoy.maxOutputValue;
    maxDelta = configJoy.angularZ_scale;
    
}

void LeftErrorCallback(const control_msgs::JointControllerState& msg)
{
    lefterror = msg.set_point;
}

void RightErrorCallback(const control_msgs::JointControllerState& msg)
{
    righterror = msg.set_point;
}

std::pair<double, double> calcSpeed(double vx, double vz) {
    // Számoljuk ki a és b értékeket
    double a = vx + vz;
    double b = vx - vz;

    // Számoljuk ki az abszolút értékeket
    double absA = std::abs(a);
    double absB = std::abs(b);

    // Ellenőrizzük, hogy a vagy b nagyobb-e mint 1
    if (absA > 1.0 || absB > 1.0) {
        // Számoljuk ki a skálázási faktort
        double scale = 1.0 / std::max(absA, absB);

        // Skálázzuk az értékeket, hogy abszolút értékük ne legyen nagyobb mint 1
        a *= scale;
        b *= scale;
    }

    // Visszaadjuk az eredményt
    return std::make_pair(a, b);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ROS_INFO("BeginjoyCallback.");
    float r = joy->axes[1];

    ROS_INFO("Checking the buttons value.");
    for (int i = 0; i < cntOfButons; ++i) {
        if (joy->buttons[i] != buttonsCopy[i]) {
            // Azonos indexű elemek különböznek, hívjuk meg a függvényt
            if(functionPointers[i] !=nullptr)
            {
                ROS_INFO("Button comparison was changed for button no:%d.", i);
                functionPointers[i](joy->buttons[i]);
            }
            else
            {
                ROS_INFO("Button comparison was not changed for button no:%d.", i);
            }
        }
    }
    
    ROS_INFO("Get axis value.");
    aix0_Value = joy->axes[configJoy.vz_Axis];
    aix1_Value = joy->axes[configJoy.vx_Axis];

    V3_linear.x = aix1_Value;
    V3_angular.z = aix0_Value;

    float vx = aix1_Value;
    float vz = aix0_Value * maxDelta;

    
    std::pair<double, double>speeds = calcSpeed(vx,vz);
    rightcmd_msg.data = (speeds.first == 0.0) ? 0.0 : -speeds.first * maxVx;
    leftcmd_msg.data = (speeds.second == 0.0) ? 0.0 : speeds.second * maxVx;
    ROS_INFO("Joy cmd, right=%d, left=%d", rightcmd_msg.data, leftcmd_msg.data);

    joyControll_msg.linear = V3_linear;
    joyControll_msg.angular = V3_angular;

    std::copy(std::begin(joy->buttons),std::end(joy->buttons),std::begin(buttonsCopy));
    ROS_INFO("End joyCallback.");
}

void* DeadMan(void* thisPtr)
{
    ROS_INFO("Begin DeadMan.");
    while (true)
    {
        try
        {
            emsMsg.header.stamp = ros::Time::now();
            emsMsg.systemIsOk = (button_DeadMan_1_Value && button_DeadMan_2_Value) ? 1 : 0;
            ems_pub.publish(emsMsg);            
        }
        catch (std::exception& ex)
        {
            ROS_ERROR("Exception on thread: %s", ex.what());
        }

        usleep(300000);
    }
    ROS_INFO("End DeadMan.");
}

void* JoyCommand(void* thisPtr)
{
    ROS_INFO("Begin JoyCommand.");
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
        catch (std::exception& ex)
        {
            ROS_ERROR("Exception on %s", ex.what());
        }

        usleep(300000);
    }
    ROS_INFO("End JoyCommand.");
}

int main(int argc, char** argv)
{
    ROS_INFO("Start joy");

    ros::init(argc, argv, "TeleopJoy");
    ros::NodeHandle nh;

    ROS_INFO("Init dinamic reconfiguraton.");
    dynamic_reconfigure::Server<robot::RobotJoyConfig> DinReconfig;
    dynamic_reconfigure::Server<robot::RobotJoyConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfigureCallBack, _1, _2);
    DinReconfig.setCallback(f);    

    ROS_INFO("Init subscribers.");
    ros::Subscriber joy_sub;
    ros::Subscriber leftError_sub;
    ros::Subscriber rightError_sub;
    sleep(1);
    joy_sub = nh.subscribe("joy", 10, &joyCallback);
    leftError_sub = nh.subscribe("/GYOM/controller/velocity/LeftWCont/state", 100, &LeftErrorCallback);
    rightError_sub = nh.subscribe("/GYOM/controller/velocity/LeftWCont/state", 100, &RightErrorCallback);

    ROS_INFO("Init publishers.");
    ems_pub = nh.advertise<robot::GlobalEnable>("DeadMan", 10);
    joy_controll_pub = nh.advertise<geometry_msgs::Twist>("TeleopSig", 10);
    leftCmd_pub = nh.advertise<std_msgs::Float64>("LeftCmd", 10);
    rightCmd_pub = nh.advertise<std_msgs::Float64>("RightCmd", 10);

    ROS_INFO("Start threads");
    pthread_create(&DeadManSwitchStateWrite, NULL, &DeadMan, NULL);
    pthread_create(&JoyStateStateWrite, NULL, &JoyCommand, NULL);

    ros::spin();

    ROS_INFO("END main.");
    return 0;
}
