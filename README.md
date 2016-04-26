# ROS Control in Orocos

This package is an daptation of https://github.com/skohlbr/rtt_ros_control_example

### Design 
The original design was working perfectly on Gnulinux, but would cause Xenomai to crash on ROS Services calls. This is probably because the services are supposed to be handled by [rtt_roscomm](https://github.com/orocos/rtt_ros_integration/tree/indigo-devel/rtt_roscomm) and they are declared [in the constructor of the controller manager](https://github.com/ros-controls/ros_control/blob/jade-devel/controller_manager/src/controller_manager.cpp#L57:L63).
So I basically copy the [controller manager's code](https://github.com/ros-controls/ros_control/blob/jade-devel/controller_manager/src/controller_manager.cpp) to enable rtt_roscomm [to handle the rosservices correctly](https://github.com/kuka-isir/rtt_ros_control_embedded/blob/master/src/rtt_ros_control_embedded.cpp#L170:L195).

The hardware_interface provided has been tested successfully on the Kuka LWR 4+ on a Xenomai PC (running rtt-2.8 and ros-indigo on 14.04) at 1Khz with no crash/issues whatsoever.

It's the **hardware interface that [creates the orocos ports on the controller_manager](https://github.com/kuka-isir/rtt_ros_control_embedded/blob/master/src/rtt_hw_interface.cpp#L8:L16 )** (only the controller manager is an rtt component, the hardware interface is a passive class that has read() and write() only), so that the controller manager can stay as generic as possible ! 

