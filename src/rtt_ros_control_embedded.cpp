#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/service.h>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rosservice.h>

#include <boost/thread.hpp>

#include <rtt_hw_interface/rtt_hw_interface.h>

#include <controller_manager/controller_manager.h>

using namespace RTT;
using namespace controller_manager;

class RttRosControl : public RTT::TaskContext {
private:

    // Necessary components to run thread for serving ROS callbacks
    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_,cm_node_;
    ros::CallbackQueue non_rt_ros_queue_;

    std::string cm_ns_;

    // The hardware interface
    RttHwInterface hw_interface_;

    // The controller manager
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // For saving last update time, so period can be handed to controller manager
    ros::Time last_update_time_;

public:
    RttRosControl(const std::string& name):
        TaskContext(name),
        non_rt_ros_nh_(new ros::NodeHandle("")),
        hw_interface_(this)
    {
        this->addProperty("controller_manager_ns",cm_ns_);
        non_rt_ros_nh_->setCallbackQueue(&non_rt_ros_queue_);
        non_rt_ros_queue_thread_ = boost::thread( boost::bind( &RttRosControl::serviceNonRtRosQueue,this ) );
    }

    bool configureHook()
    {
        cm_node_.reset(new ros::NodeHandle(*non_rt_ros_nh_,cm_ns_));
        controller_manager_.reset(new controller_manager::ControllerManager(&hw_interface_, *cm_node_));
        return true;
    }
    bool startHook(){
        last_update_time_ = rtt_rosclock::rtt_now();
        return true;
    }

    void updateHook() {

        // Get current system time (for timestamps of ROS messages)
        ros::Time now (rtt_rosclock::host_now());

        // Get guaranteed monotonic time for period computation
        ros::Time now_monotonic(rtt_rosclock::rtt_now());

        ros::Duration period (now_monotonic - last_update_time_);
        last_update_time_ = now_monotonic;

        hw_interface_.read();
        controller_manager_->update(now, period);
        hw_interface_.write();
    }

    void cleanupHook() {
        non_rt_ros_nh_->shutdown();
        non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
        static const double timeout = 0.001;

        while (non_rt_ros_nh_->ok()) {
            non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }
};

ORO_CREATE_COMPONENT(RttRosControl)
