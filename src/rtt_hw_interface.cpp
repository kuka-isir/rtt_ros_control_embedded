
#include <rtt_hw_interface/rtt_hw_interface.h>
#include <controller_manager/controller_manager.h>

//using namespace rtt_ros_kdl_tools;

RttHwInterface::RttHwInterface(RTT::TaskContext* owner):
owner_(owner)
{
    owner_->ports()->addPort("JointPosition", port_joint_position_in).doc("");
    owner_->ports()->addPort("JointVelocity", port_joint_velocity_in).doc("");
    owner_->ports()->addPort("JointTorque", port_joint_torque_in).doc("");

    owner_->ports()->addPort("JointPositionCommand", port_joint_position_cmd_out).doc("");
    owner_->ports()->addPort("JointVelocityCommand", port_joint_velocity_cmd_out).doc("");
    owner_->ports()->addPort("JointTorqueCommand", port_joint_torque_cmd_out).doc("");

    std::string robot_description_param_name("robot_description");

    std::string robot_description, root_link, tip_link;

    if(!ros::param::get(robot_description_param_name, robot_description))
        ROS_ERROR( "Could not get %s param",robot_description_param_name.c_str());

    urdf::Model urdf_model;
    std::vector<std::string> joint_names;

    if(!urdf_model.initString(robot_description)) {
        ROS_ERROR("Could not init URDF");
    }

    for (auto& joint : urdf_model.joints_)
    {
      if(joint.second->limits)
      {
        if(joint.second->limits->lower != joint.second->limits->upper)
        {
            ROS_INFO("Adding joint [%s]",joint.second->name.c_str());
            joint_names.push_back(joint.second->name);
        }
      }
    }

    jnt_pos_in.setZero(joint_names.size());
    jnt_vel_in.setZero(joint_names.size());
    jnt_trq_in.setZero(joint_names.size());

    jnt_pos_cmd_out.setZero(joint_names.size());
    jnt_vel_cmd_out.setZero(joint_names.size());
    jnt_trq_cmd_out.setZero(joint_names.size());

    for(size_t i=0; i<joint_names.size(); ++i)
    {

      ROS_INFO("Registering joint [%s]",joint_names[i].c_str());
      hardware_interface::JointStateHandle state_handle(joint_names[i], &jnt_pos_in[i], &jnt_vel_in[i], &jnt_trq_in[i]);
      joint_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names[i]), &jnt_pos_cmd_out[i]);
      position_joint_interface_.registerHandle(pos_handle);

      hardware_interface::JointHandle vel_handle(joint_state_interface_.getHandle(joint_names[i]), &jnt_vel_cmd_out[i]);
      velocity_joint_interface_.registerHandle(vel_handle);

      hardware_interface::JointHandle eff_handle(joint_state_interface_.getHandle(joint_names[i]), &jnt_trq_cmd_out[i]);
      effort_joint_interface_.registerHandle(eff_handle);

    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
}
/*
void RttHwInterface::cleanup()
{

}
*/
void RttHwInterface::read()
{
    port_joint_position_in.readNewest(jnt_pos_in);
    port_joint_velocity_in.readNewest(jnt_vel_in);
    port_joint_torque_in.readNewest(jnt_trq_in);
}

void RttHwInterface::write()
{
    port_joint_position_cmd_out.write(jnt_pos_cmd_out);
    port_joint_velocity_cmd_out.write(jnt_vel_cmd_out);
    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
}
