# ros_control in rtt

Adaptation of https://github.com/skohlbr/rtt_ros_control_example
Reads the URDF, creates the orocos ports for reading/writing robot state.

It assumes that the robot has Eigen::VectorXd ports for:
- Sending state (q,qdot,(Torque))
- Reading commands (Torque, (q,qdot))
