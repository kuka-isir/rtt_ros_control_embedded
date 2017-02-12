///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Antoine Hoarau, Jimmy Da Silva, ISIR-UPMC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of TU Darmstadt nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////


#ifndef __RTT_HW_INTERFACE__
#define __RTT_HW_INTERFACE__

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <hardware_interface/robot_hw.h>
#include <rtt/Port.hpp>
#include <Eigen/Core>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <urdf/model.h>

// New interface for cart_opt controller
namespace hardware_interface
{
class CartOptEffortJointInterface : public JointCommandInterface {};
}

class RttHwInterface : public hardware_interface::RobotHW
{
public:
    RttHwInterface(RTT::TaskContext* owner);

    void read();
    void write();

private:
    RTT::TaskContext* owner_;
    // Provide state, velocity and position interfaces in this example
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;
    hardware_interface::CartOptEffortJointInterface cart_opt_joint_interface_;

    joint_limits_interface::EffortJointSaturationInterface jnt_eff_limit_interface_;

    std::vector<joint_limits_interface::JointLimits> limits_;
    std::vector<joint_limits_interface::SoftJointLimits> soft_limits_;

    RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                     port_joint_velocity_in,
                                     port_joint_torque_in;

    Eigen::VectorXd jnt_pos_in,
                    jnt_vel_in,
                    jnt_trq_in;

    RTT::OutputPort<Eigen::VectorXd> port_joint_position_cmd_out,
                                     port_joint_velocity_cmd_out,
                                     port_joint_torque_cmd_out;
    Eigen::VectorXd jnt_pos_cmd_out,
                    jnt_vel_cmd_out,
                    jnt_trq_cmd_out;

    RTT::FlowStatus jnt_pos_fs,jnt_trq_fs,jnt_vel_fs;
};


#endif
