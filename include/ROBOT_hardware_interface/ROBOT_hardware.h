#ifndef ROS_CONTROL__ROBOT_HARDWARE_H
#define ROS_CONTROL__ROBOT_HARDWARE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace ROBOT_hardware_interface
{
    /// \brief Hardware interface for a robot
    class ROBOTHardware : public hardware_interface::RobotHW
    {
    protected:
// pwm
#define PWM_HARDVER_ENABLE 0x0     /**< enable */
#define PWM_HARDVER_BREAKE_SW 0x4  /**< breake_sw */
#define PWM_HARDVER_HWORSW 0x8     /**< hworsw */
#define PWM_HARDVER_FREQ 0xc       /**< freq */
#define PWM_HARDVER_DUTY_SW 0x10   /**< duty_sw */
#define PWM_HARDVER_DIV 0x14       /**< div */
#define PWM_HARDVER_INPUTDUTY 0x18 /**< inputduty */

#define POSITIONCOUNTERSIMPLE_SETRESETVALUE 0x0       /**< setresetvalue */
#define POSITIONCOUNTERSIMPLE_SETCOUNTER 0x4          /**< setcounter */
#define POSITIONCOUNTERSIMPLE_LOADVALUE 0x8           /**< loadvalue */
#define POSITIONCOUNTERSIMPLE_COUNT_DIRECTION 0xc     /**< count_direction */
#define POSITIONCOUNTERSIMPLE_ANGULARPOSITIONAXI 0x10 /**< angularpositionaxi */

        // Interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;

        joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
        joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
        joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
        joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
        joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

        // Custom or available transmissions
        // transmission_interface::RRBOTTransmission rrbot_trans_;
        // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

        // Shared memory
        int num_joints_;
        int joint_mode_; // position, velocity, or effort
        std::vector<std::string> joint_names_;
        std::vector<int> joint_types_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        std::vector<double> joint_effort_limits_;

    }; // class

} // namespace

#endif
