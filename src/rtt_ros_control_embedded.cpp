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
    ros::NodeHandle non_rt_ros_nh_;
    ros::CallbackQueue non_rt_ros_queue_;

    // The hardware interface
    RttHwInterface hw_interface_;

    // The controller manager
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // For saving last update time, so period can be handed to controller manager
    ros::Time last_update_time_;

private:
    ros::NodeHandle cm_node_;

public:
    RttRosControl(const std::string& name):
        TaskContext(name),
        non_rt_ros_nh_(""),
        hw_interface_(this),
        cm_node_(non_rt_ros_nh_, "")
    {
        non_rt_ros_nh_.setCallbackQueue(&non_rt_ros_queue_);
        non_rt_ros_queue_thread_ = boost::thread( boost::bind( &RttRosControl::serviceNonRtRosQueue,this ) );

	controller_manager_.reset(new controller_manager::ControllerManager(&hw_interface_, cm_node_));

//         this->addOperation("list_controllers",&RttRosControl::listControllersSrv,this,RTT::ClientThread);
//         this->addOperation("list_controller_types",&RttRosControl::listControllerTypesSrv,this,RTT::ClientThread);
//         this->addOperation("load_controller",&RttRosControl::loadControllerSrv,this,RTT::ClientThread);
//         this->addOperation("unload_controller",&RttRosControl::unloadControllerSrv,this,RTT::ClientThread);
//         this->addOperation("reload_controller_libraries",&RttRosControl::reloadControllerLibrariesSrv,this,RTT::ClientThread);
//         this->addOperation("switch_controller",&RttRosControl::switchControllerSrv,this,RTT::ClientThread);

    }

    virtual ~RttRosControl()
    {

    }

//     bool listControllerTypesSrv(controller_manager_msgs::ListControllerTypes::Request &req,
// 				controller_manager_msgs::ListControllerTypes::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/list_controller_types",req,resp);
//     }
//     bool listControllersSrv(controller_manager_msgs::ListControllers::Request &req,
// 			    controller_manager_msgs::ListControllers::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/list_controllers",req,resp);
//     }
//     bool switchControllerSrv(controller_manager_msgs::SwitchController::Request &req,
// 			    controller_manager_msgs::SwitchController::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/switch_controller",req,resp);
//     }
//     bool loadControllerSrv(controller_manager_msgs::LoadController::Request &req,
// 			    controller_manager_msgs::LoadController::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/load_controller",req,resp);
//     }
//     bool unloadControllerSrv(controller_manager_msgs::UnloadController::Request &req,
// 			  controller_manager_msgs::UnloadController::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/unload_controller",req,resp);
//     }
//     bool reloadControllerLibrariesSrv(controller_manager_msgs::ReloadControllerLibraries::Request &req,
// 				      controller_manager_msgs::ReloadControllerLibraries::Response &resp)
//     {
//       return ros::service::call(cm_node_.getNamespace()+"/controller_manager/reload_controller_libraries",req,resp);
//     }
// 
//     bool configureHook() {
//         boost::shared_ptr<rtt_rosservice::ROSService> rosservice =
//             this->getProvider<rtt_rosservice::ROSService>("rosservice");
// 
//         if(!rosservice)
//         {
//           log(Error) << "rosservice not available, please check if rtt_ros is imported" << endlog();
// 	  return false;
//         }
// 
//         rosservice->connect("list_controller_types",
//                             "controller_manager/list_controller_types",
//                             "controller_manager_msgs/ListControllerTypes");
// 
//         rosservice->connect("list_controllers",
//                             "controller_manager/list_controllers",
//                             "controller_manager_msgs/ListControllers");
// 
//         rosservice->connect("load_controller",
//                             "controller_manager/load_controller",
//                             "controller_manager_msgs/LoadController");
// 
//         rosservice->connect("reload_controller_libraries",
//                             "controller_manager/reload_controller_libraries",
//                             "controller_manager_msgs/ReloadControllerLibraries");
// 
//         rosservice->connect("switch_controller",
//                             "controller_manager/switch_controller",
//                             "controller_manager_msgs/SwitchController");
// 
//         rosservice->connect("unload_controller",
//                             "controller_manager/unload_controller",
//                             "controller_manager_msgs/UnloadController");
// 	return true;
// 
//     }

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
        non_rt_ros_nh_.shutdown();
        non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
        static const double timeout = 0.001;

        while (non_rt_ros_nh_.ok()) {
            non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }
};

ORO_CREATE_COMPONENT(RttRosControl)
