///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, Stefan Kohlbrecher, TU Darmstadt
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

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <boost/thread.hpp>

#include <rtt_hw_interface/rtt_hw_interface.h>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>
#include "controller_manager/controller_spec.h"
#include <pthread.h>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tinyxml.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <controller_manager/controller_loader_interface.h>
#include <rtt_roscomm/rosservice.h>

using namespace RTT;
using namespace controller_manager;
//using namespace controller_interface;

class RttRosControl : public RTT::TaskContext {
private:

    // Necessary components to run thread for serving ROS callbacks
    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_;
    ros::CallbackQueue non_rt_ros_queue_;

    // The (example) hardware interface
    boost::shared_ptr<RttHwInterface> hw_interface_;

    // The controller manager
    //boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // For saving last update time, so period can be handed to controller manager
    ros::Time last_update_time_;
    // Controllers to load before the realtime loop !
    std::vector<std::string> controllers_list_,stop_controllers_;
private:

    hardware_interface::RobotHW* robot_hw_;

    ros::NodeHandle root_nh_, cm_node_;

    typedef boost::shared_ptr<ControllerLoaderInterface> LoaderPtr;
    std::list<LoaderPtr> controller_loaders_;

    /** \name Controller Switching
    *\{*/
    std::vector<controller_interface::ControllerBase*> start_request_, stop_request_;
    std::list<hardware_interface::ControllerInfo> switch_start_list_, switch_stop_list_;
    bool please_switch_;
    int switch_strictness_;
    /*\}*/

    /** \name Controllers List
    * The controllers list is double-buffered to avoid needing to lock the
    * real-time thread when switching controllers in the non-real-time thread.
    *\{*/
    /// Mutex protecting the current controllers list
    boost::recursive_mutex controllers_lock_;
    /// Double-buffered controllers list
    std::vector<ControllerSpec> controllers_lists_[2];
    /// The index of the current controllers list
    int current_controllers_list_;
    /// The index of the controllers list being used in the real-time thread.
    int used_by_realtime_;
    /*\}*/


    boost::mutex services_lock_;
    ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_load_controller_;
    ros::ServiceServer srv_unload_controller_, srv_switch_controller_, srv_reload_libraries_;
public:
    RttRosControl(const std::string& name):
        TaskContext(name),
        //robot_hw_(robot_hw),
        //root_nh_(nh),
        //cm_node_(nh, "controller_manager"),
        start_request_(0),
        stop_request_(0),
        please_switch_(false),
        current_controllers_list_(0),
        used_by_realtime_(-1)
    {
        this->addProperty("controllers_list",controllers_list_);
        this->addOperation("preloadController",&RttRosControl::preloadController,this,RTT::OwnThread);
        this->addOperation("startControllers",&RttRosControl::startControllers,this,RTT::ClientThread);

        non_rt_ros_nh_.reset(new ros::NodeHandle(""));
        non_rt_ros_nh_->setCallbackQueue(&non_rt_ros_queue_);
        this->non_rt_ros_queue_thread_ = boost::thread( boost::bind( &RttRosControl::serviceNonRtRosQueue,this ) );

        hw_interface_.reset(new RttHwInterface(this));

        robot_hw_ = hw_interface_.get();
        root_nh_= ros::NodeHandle(*non_rt_ros_nh_);
        cm_node_ = ros::NodeHandle(*non_rt_ros_nh_,"controller_manager");
        // create controller loader
        controller_loaders_.push_back( LoaderPtr(new ControllerLoader<controller_interface::ControllerBase>("controller_interface",
                                       "controller_interface::ControllerBase") ) );
        // Advertise services (this should be the last thing we do in init)
        /*srv_list_controllers_ = cm_node_.advertiseService("list_controllers", &ControllerManager::listControllersSrv, this);
        srv_list_controller_types_ = cm_node_.advertiseService("list_controller_types", &ControllerManager::listControllerTypesSrv, this);
        srv_load_controller_ = cm_node_.advertiseService("load_controller", &ControllerManager::loadControllerSrv, this);
        srv_unload_controller_ = cm_node_.advertiseService("unload_controller", &ControllerManager::unloadControllerSrv, this);
        srv_switch_controller_ = cm_node_.advertiseService("switch_controller", &ControllerManager::switchControllerSrv, this);
        srv_reload_libraries_ = cm_node_.advertiseService("reload_controller_libraries", &ControllerManager::reloadControllerLibrariesSrv, this);*/

        this->addOperation("list_controllers",&RttRosControl::listControllersSrv,this,RTT::ClientThread);
        this->addOperation("list_controller_types",&RttRosControl::listControllerTypesSrv,this,RTT::ClientThread);
        this->addOperation("load_controller",&RttRosControl::loadControllerSrv,this,RTT::ClientThread);
        this->addOperation("unload_controller",&RttRosControl::unloadControllerSrv,this,RTT::ClientThread);
        this->addOperation("reload_controller_libraries",&RttRosControl::reloadControllerLibrariesSrv,this,RTT::ClientThread);
        this->addOperation("switch_controller",&RttRosControl::switchControllerSrv,this,RTT::ClientThread);

        boost::shared_ptr<rtt_rosservice::ROSService> rosservice =
            this->getProvider<rtt_rosservice::ROSService>("rosservice");

        rosservice->connect("list_controller_types",
                            "controller_manager/list_controller_types",
                            "controller_manager_msgs/ListControllerTypes");

        rosservice->connect("list_controllers",
                            "controller_manager/list_controllers",
                            "controller_manager_msgs/ListControllers");

        rosservice->connect("load_controller",
                            "controller_manager/load_controller",
                            "controller_manager_msgs/LoadController");

        rosservice->connect("reload_controller_libraries",
                            "controller_manager/reload_controller_libraries",
                            "controller_manager_msgs/ReloadControllerLibraries");

        rosservice->connect("switch_controller",
                            "controller_manager/switch_controller",
                            "controller_manager_msgs/SwitchController");

        rosservice->connect("unload_controller",
                            "controller_manager/unload_controller",
                            "controller_manager_msgs/UnloadController");
    }

    ~RttRosControl()
    {
        for(const auto& c : controllers_list_)
        {
            RTT::log(RTT::Info) << "Unloading controller "<<c<<RTT::endlog();
            this->unloadController(c);
        }
    }

private:
    bool preloadController(const std::string& controller_name)
    {
        ROS_INFO("Loading controller [%s]",controller_name.c_str());
        controllers_list_.push_back(controller_name);
        return this->loadController(controller_name);
    }
    bool startControllers()
    {
        return switchController(controllers_list_,stop_controllers_,controller_manager_msgs::SwitchController::Request::BEST_EFFORT);
    }
    bool configureHook() {
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

        hw_interface_->read();
        this->update(now, period);
        hw_interface_->write();
    }

    void cleanupHook() {
        non_rt_ros_nh_->shutdown();
        non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
        static const double timeout = 0.001;

        while (this->non_rt_ros_nh_->ok()) {
            this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

// Must be realtime safe.
    void update(const ros::Time& time, const ros::Duration& period, bool reset_controllers = false)
    {
        used_by_realtime_ = current_controllers_list_;
        std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];

        // Restart all running controllers if motors are re-enabled
        if (reset_controllers) {
            for (size_t i=0; i<controllers.size(); i++) {
                if (controllers[i].c->isRunning()) {
                    controllers[i].c->stopRequest(time);
                    controllers[i].c->startRequest(time);
                }
            }
        }


        // Update all controllers
        for (size_t i=0; i<controllers.size(); i++)
            controllers[i].c->updateRequest(time, period);

        // there are controllers to start/stop
        if (please_switch_)
        {
            // switch hardware interfaces (if any)
            robot_hw_->doSwitch(switch_start_list_, switch_stop_list_);

            // stop controllers
            for (unsigned int i=0; i<stop_request_.size(); i++)
                if (!stop_request_[i]->stopRequest(time))
                    ROS_FATAL("Failed to stop controller in realtime loop. This should never happen.");

            // start controllers
            for (unsigned int i=0; i<start_request_.size(); i++)
                if (!start_request_[i]->startRequest(time))
                    ROS_FATAL("Failed to start controller in realtime loop. This should never happen.");

            please_switch_ = false;
        }
    }

    controller_interface::ControllerBase* getControllerByName(const std::string& name)
    {
        // Lock recursive mutex in this context
        boost::recursive_mutex::scoped_lock guard(controllers_lock_);

        std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
        for (size_t i = 0; i < controllers.size(); ++i)
        {
            if (controllers[i].info.name == name)
                return controllers[i].c.get();
        }
        return NULL;
    }

    void getControllerNames(std::vector<std::string> &names)
    {
        boost::recursive_mutex::scoped_lock guard(controllers_lock_);
        names.clear();
        std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
        for (size_t i = 0; i < controllers.size(); ++i)
        {
            names.push_back(controllers[i].info.name);
        }
    }


    bool loadController(const std::string& name)
    {
        ROS_DEBUG("Will load controller '%s'", name.c_str());

        // lock controllers
        boost::recursive_mutex::scoped_lock guard(controllers_lock_);

        // get reference to controller list
        int free_controllers_list = (current_controllers_list_ + 1) % 2;
        while (ros::ok() && free_controllers_list == used_by_realtime_) {
            if (!ros::ok())
                return false;
            usleep(200);
        }
        std::vector<ControllerSpec>
        &from = controllers_lists_[current_controllers_list_],
         &to = controllers_lists_[free_controllers_list];
        to.clear();

        // Copy all controllers from the 'from' list to the 'to' list
        for (size_t i = 0; i < from.size(); ++i)
            to.push_back(from[i]);

        // Checks that we're not duplicating controllers
        for (size_t j = 0; j < to.size(); ++j)
        {
            if (to[j].info.name == name)
            {
                to.clear();
                ROS_ERROR("A controller named '%s' was already loaded inside the controller manager", name.c_str());
                return false;
            }
        }

        ros::NodeHandle c_nh;
        // Constructs the controller
        try {
            c_nh = ros::NodeHandle(root_nh_, name);
        }
        catch(std::exception &e) {
            ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name.c_str(), e.what());
            return false;
        }
        catch(...) {
            ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name.c_str());
            return false;
        }
        boost::shared_ptr<controller_interface::ControllerBase> c;
        std::string type;
        if (c_nh.getParam("type", type))
        {
            ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());
            try
            {
                // Trying loading the controller using all of our controller loaders. Exit once we've found the first valid loaded controller
                std::list<LoaderPtr>::iterator it = controller_loaders_.begin();
                while (!c && it != controller_loaders_.end())
                {
                    std::vector<std::string> cur_types = (*it)->getDeclaredClasses();
                    for(size_t i=0; i < cur_types.size(); i++) {
                        if (type == cur_types[i]) {
                            c = (*it)->createInstance(type);
                        }
                    }
                    ++it;
                }
            }
            catch (const std::runtime_error &ex)
            {
                ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
            }
        }
        else
        {
            ROS_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server (namespace: '%s')?", name.c_str(), c_nh.getNamespace().c_str());
            to.clear();
            return false;
        }

        // checks if controller was constructed
        if (!c)
        {
            ROS_ERROR("Could not load controller '%s' because controller type '%s' does not exist.",  name.c_str(), type.c_str());
            ROS_ERROR("Use 'rosservice call controller_manager/list_controller_types' to get the available types");
            to.clear();
            return false;
        }

        // Initializes the controller
        ROS_DEBUG("Initializing controller '%s'", name.c_str());
        bool initialized;
        std::set<std::string> claimed_resources; // Gets populated during initRequest call
        try {
            initialized = c->initRequest(robot_hw_, root_nh_, c_nh, claimed_resources);
        }
        catch(std::exception &e) {
            ROS_ERROR("Exception thrown while initializing controller %s.\n%s", name.c_str(), e.what());
            initialized = false;
        }
        catch(...) {
            ROS_ERROR("Exception thrown while initializing controller %s", name.c_str());
            initialized = false;
        }
        if (!initialized)
        {
            to.clear();
            ROS_ERROR("Initializing controller '%s' failed", name.c_str());
            return false;
        }
        ROS_DEBUG("Initialized controller '%s' successful", name.c_str());

        // Adds the controller to the new list
        to.resize(to.size() + 1);
        to[to.size()-1].info.type = type;
        to[to.size()-1].info.hardware_interface = c->getHardwareInterfaceType();
        to[to.size()-1].info.name = name;
        to[to.size()-1].info.resources = claimed_resources;
        to[to.size()-1].c = c;

        // Destroys the old controllers list when the realtime thread is finished with it.
        int former_current_controllers_list_ = current_controllers_list_;
        current_controllers_list_ = free_controllers_list;
        while (ros::ok() && used_by_realtime_ == former_current_controllers_list_) {
            if (!ros::ok())
                return false;
            usleep(200);
        }
        from.clear();

        ROS_DEBUG("Successfully load controller '%s'", name.c_str());
        return true;
    }




    bool unloadController(const std::string &name)
    {
        ROS_DEBUG("Will unload controller '%s'", name.c_str());

        // lock the controllers
        boost::recursive_mutex::scoped_lock guard(controllers_lock_);

        // get reference to controller list
        int free_controllers_list = (current_controllers_list_ + 1) % 2;
        while (ros::ok() && free_controllers_list == used_by_realtime_) {
            if (!ros::ok())
                return false;
            usleep(200);
        }
        std::vector<ControllerSpec>
        &from = controllers_lists_[current_controllers_list_],
         &to = controllers_lists_[free_controllers_list];
        to.clear();

        // Transfers the running controllers over, skipping the one to be removed and the running ones.
        bool removed = false;
        for (size_t i = 0; i < from.size(); ++i)
        {
            if (from[i].info.name == name) {
                if (from[i].c->isRunning()) {
                    to.clear();
                    ROS_ERROR("Could not unload controller with name %s because it is still running",
                              name.c_str());
                    return false;
                }
                removed = true;
            }
            else
                to.push_back(from[i]);
        }

        // Fails if we could not remove the controllers
        if (!removed)
        {
            to.clear();
            ROS_ERROR("Could not unload controller with name %s because no controller with this name exists",
                      name.c_str());
            return false;
        }

        // Destroys the old controllers list when the realtime thread is finished with it.
        ROS_DEBUG("Realtime switches over to new controller list");
        int former_current_controllers_list_ = current_controllers_list_;
        current_controllers_list_ = free_controllers_list;
        while (ros::ok() && used_by_realtime_ == former_current_controllers_list_) {
            if (!ros::ok())
                return false;
            usleep(200);
        }
        ROS_DEBUG("Destruct controller");
        from.clear();
        ROS_DEBUG("Destruct controller finished");

        ROS_DEBUG("Successfully unloaded controller '%s'", name.c_str());
        return true;
    }



    bool switchController(const std::vector<std::string>& start_controllers,
                          const std::vector<std::string>& stop_controllers,
                          int strictness)
    {
        if (!stop_request_.empty() || !start_request_.empty())
            ROS_FATAL("The switch controller stop and start list are not empty that the beginning of the swithcontroller call. This should not happen.");

        if (strictness == 0) {
            ROS_WARN("Controller Manager: To switch controllers you need to specify a strictness level of controller_manager_msgs::SwitchController::STRICT (%d) or ::BEST_EFFORT (%d). Defaulting to ::BEST_EFFORT.",
                     controller_manager_msgs::SwitchController::Request::STRICT,
                     controller_manager_msgs::SwitchController::Request::BEST_EFFORT);
            strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
        }

        ROS_DEBUG("switching controllers:");
        for (unsigned int i=0; i<start_controllers.size(); i++)
            ROS_DEBUG(" - starting controller %s", start_controllers[i].c_str());
        for (unsigned int i=0; i<stop_controllers.size(); i++)
            ROS_DEBUG(" - stopping controller %s", stop_controllers[i].c_str());

        // lock controllers
        boost::recursive_mutex::scoped_lock guard(controllers_lock_);

        controller_interface::ControllerBase* ct;
        // list all controllers to stop
        for (unsigned int i=0; i<stop_controllers.size(); i++)
        {
            ct = getControllerByName(stop_controllers[i]);
            if (ct == NULL) {
                if (strictness ==  controller_manager_msgs::SwitchController::Request::STRICT) {
                    ROS_ERROR("Could not stop controller with name %s because no controller with this name exists",
                              stop_controllers[i].c_str());
                    stop_request_.clear();
                    return false;
                }
                else {
                    ROS_DEBUG("Could not stop controller with name %s because no controller with this name exists",
                              stop_controllers[i].c_str());
                }
            }
            else {
                ROS_DEBUG("Found controller %s that needs to be stopped in list of controllers",
                          stop_controllers[i].c_str());
                stop_request_.push_back(ct);
            }
        }
        ROS_DEBUG("Stop request vector has size %i", (int)stop_request_.size());

        // list all controllers to start
        for (unsigned int i=0; i<start_controllers.size(); i++)
        {
            ct = getControllerByName(start_controllers[i]);
            if (ct == NULL) {
                if (strictness ==  controller_manager_msgs::SwitchController::Request::STRICT) {
                    ROS_ERROR("Could not start controller with name %s because no controller with this name exists",
                              start_controllers[i].c_str());
                    stop_request_.clear();
                    start_request_.clear();
                    return false;
                }
                else {
                    ROS_DEBUG("Could not start controller with name %s because no controller with this name exists",
                              start_controllers[i].c_str());
                }
            }
            else {
                ROS_DEBUG("Found controller %s that needs to be started in list of controllers",
                          start_controllers[i].c_str());
                start_request_.push_back(ct);
            }
        }
        ROS_DEBUG("Start request vector has size %i", (int)start_request_.size());

        // Do the resource management checking
        std::list<hardware_interface::ControllerInfo> info_list;
        switch_start_list_.clear();
        switch_stop_list_.clear();

        std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
        for (size_t i = 0; i < controllers.size(); ++i)
        {
            bool in_stop_list  = false;
            for(size_t j = 0; j < stop_request_.size(); j++)
            {
                if (stop_request_[j] == controllers[i].c.get())
                {
                    in_stop_list = true;
                    break;
                }
            }

            bool in_start_list = false;
            for(size_t j = 0; j < start_request_.size(); j++)
            {
                if (start_request_[j] == controllers[i].c.get())
                {
                    in_start_list = true;
                    break;
                }
            }

            const bool is_running = controllers[i].c->isRunning();
            hardware_interface::ControllerInfo &info = controllers[i].info;

            if(!is_running && in_stop_list) { // check for double stop
                if(strictness ==  controller_manager_msgs::SwitchController::Request::STRICT) {
                    ROS_ERROR_STREAM("Could not stop controller '" << info.name << "' since it is not running");
                    stop_request_.clear();
                    start_request_.clear();
                    return false;
                } else {
                    in_stop_list = false;
                }
            }

            if(is_running && !in_stop_list && in_start_list) { // check for doubled start
                if(strictness ==  controller_manager_msgs::SwitchController::Request::STRICT) {
                    ROS_ERROR_STREAM("Controller '" << info.name << "' is already running");
                    stop_request_.clear();
                    start_request_.clear();
                    return false;
                } else {
                    in_start_list = false;
                }
            }

            if(is_running && in_stop_list && !in_start_list) { // running and real stop
                switch_stop_list_.push_back(info);
            }
            else if(!is_running && !in_stop_list && in_start_list) { // start, but no restart
                switch_start_list_.push_back(info);
            }

            bool add_to_list = is_running;
            if (in_stop_list)
                add_to_list = false;
            if (in_start_list)
                add_to_list = true;

            if (add_to_list)
                info_list.push_back(info);
        }

        bool in_conflict = robot_hw_->checkForConflict(info_list);
        if (in_conflict)
        {
            ROS_ERROR("Could not switch controllers, due to resource conflict");
            stop_request_.clear();
            start_request_.clear();
            return false;
        }

        if (!robot_hw_->canSwitch(switch_start_list_, switch_stop_list_))
        {
            ROS_ERROR("Could not switch controllers. The hardware interface combination for the requested controllers is unfeasible.");
            stop_request_.clear();
            start_request_.clear();
            return false;
        }

        // start the atomic controller switching
        switch_strictness_ = strictness;
        please_switch_ = true;

        // wait until switch is finished
        ROS_DEBUG("Request atomic controller switch from realtime loop");
        while (ros::ok() && please_switch_) {
            if (!ros::ok())
                return false;
            usleep(100);
        }
        start_request_.clear();
        stop_request_.clear();

        ROS_DEBUG("Successfully switched controllers");
        return true;
    }





    bool reloadControllerLibrariesSrv(
        controller_manager_msgs::ReloadControllerLibraries::Request &req,
        controller_manager_msgs::ReloadControllerLibraries::Response &resp)
    {
        // lock services
        ROS_DEBUG("reload libraries service called");
        boost::mutex::scoped_lock guard(services_lock_);
        ROS_DEBUG("reload libraries service locked");

        // only reload libraries if no controllers are running
        std::vector<std::string> controllers;
        getControllerNames(controllers);
        if (!controllers.empty() && !req.force_kill) {
            ROS_ERROR("Controller manager: Cannot reload controller libraries because there are still %i controllers running", (int)controllers.size());
            resp.ok = false;
            return true;
        }

        // kill running controllers if requested
        if (!controllers.empty()) {
            ROS_INFO("Controller manager: Killing all running controllers");
            std::vector<std::string> empty;
            if (!switchController(empty,controllers, controller_manager_msgs::SwitchController::Request::BEST_EFFORT)) {
                ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to stop running controllers");
                resp.ok = false;
                return true;
            }
            for (unsigned int i=0; i<controllers.size(); i++) {
                if (!unloadController(controllers[i])) {
                    ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to unload controller %s",
                              controllers[i].c_str());
                    resp.ok = false;
                    return true;
                }
            }
            getControllerNames(controllers);
        }
        assert(controllers.empty());

        // Force a reload on all the PluginLoaders (internally, this recreates the plugin loaders)
        for(std::list<LoaderPtr>::iterator it = controller_loaders_.begin(); it != controller_loaders_.end(); ++it)
        {
            (*it)->reload();
            ROS_INFO("Controller manager: reloaded controller libraries for %s", (*it)->getName().c_str());
        }

        resp.ok = true;

        ROS_DEBUG("reload libraries service finished");
        return true;
    }


    bool listControllerTypesSrv(
        controller_manager_msgs::ListControllerTypes::Request &req,
        controller_manager_msgs::ListControllerTypes::Response &resp)
    {
        // pretend to use the request
        (void) req;

        // lock services
        ROS_DEBUG("list types service called");
        boost::mutex::scoped_lock guard(services_lock_);
        ROS_DEBUG("list types service locked");

        for(std::list<LoaderPtr>::iterator it = controller_loaders_.begin(); it != controller_loaders_.end(); ++it)
        {
            std::vector<std::string> cur_types = (*it)->getDeclaredClasses();
            for(size_t i=0; i < cur_types.size(); i++)
            {
                resp.types.push_back(cur_types[i]);
                resp.base_classes.push_back((*it)->getName());
            }
        }

        ROS_DEBUG("list types service finished");
        return true;
    }


    bool listControllersSrv(
        controller_manager_msgs::ListControllers::Request &req,
        controller_manager_msgs::ListControllers::Response &resp)
    {
        // pretend to use the request
        (void) req;

        // lock services
        ROS_DEBUG("list controller service called");
        boost::mutex::scoped_lock services_guard(services_lock_);
        ROS_DEBUG("list controller service locked");

        // lock controllers to get all names/types/states
        boost::recursive_mutex::scoped_lock controller_guard(controllers_lock_);
        std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
        resp.controller.resize(controllers.size());

        for (size_t i = 0; i < controllers.size(); ++i)
        {
            controller_manager_msgs::ControllerState& cs = resp.controller[i];
            cs.name               = controllers[i].info.name;
            cs.type               = controllers[i].info.type;
            cs.hardware_interface = controllers[i].info.hardware_interface;
            cs.resources.clear();
            cs.resources.reserve(controllers[i].info.resources.size());
            for (std::set<std::string>::iterator it = controllers[i].info.resources.begin(); it != controllers[i].info.resources.end(); ++it)
                cs.resources.push_back(*it);

            if (controllers[i].c->isRunning())
                cs.state = "running";
            else
                cs.state = "stopped";
        }

        ROS_DEBUG("list controller service finished");
        return true;
    }


    bool loadControllerSrv(
        controller_manager_msgs::LoadController::Request &req,
        controller_manager_msgs::LoadController::Response &resp)
    {
        // lock services
        ROS_DEBUG("loading service called for controller %s ",req.name.c_str());
        boost::mutex::scoped_lock guard(services_lock_);
        ROS_DEBUG("loading service locked");

        resp.ok = loadController(req.name);

        ROS_DEBUG("loading service finished for controller %s ",req.name.c_str());
        return true;
    }


    bool unloadControllerSrv(
        controller_manager_msgs::UnloadController::Request &req,
        controller_manager_msgs::UnloadController::Response &resp)
    {
        // lock services
        ROS_DEBUG("unloading service called for controller %s ",req.name.c_str());
        boost::mutex::scoped_lock guard(services_lock_);
        ROS_DEBUG("unloading service locked");

        resp.ok = unloadController(req.name);

        ROS_DEBUG("unloading service finished for controller %s ",req.name.c_str());
        return true;
    }


    bool switchControllerSrv(
        controller_manager_msgs::SwitchController::Request &req,
        controller_manager_msgs::SwitchController::Response &resp)
    {
        // lock services
        ROS_DEBUG("switching service called");
        boost::mutex::scoped_lock guard(services_lock_);
        ROS_DEBUG("switching service locked");

        resp.ok = switchController(req.start_controllers, req.stop_controllers, req.strictness);

        ROS_DEBUG("switching service finished");
        return true;
    }

    void registerControllerLoader(boost::shared_ptr<ControllerLoaderInterface> controller_loader)
    {
        controller_loaders_.push_back(controller_loader);
    }



};


ORO_CREATE_COMPONENT(RttRosControl)
