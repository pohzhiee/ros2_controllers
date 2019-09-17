// Copyright 2019 SIW Manufacturing Sdn. Bhd.
// Poh Zhi-Ee <zhiee.poh@httechnology.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros_controllers/joint_velocity_controller.hpp"

#include <string>
#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "rcutils/logging_macros.h"

namespace ros_controllers
{

JointVelocityController::JointVelocityController()
    : controller_interface::ControllerInterface()
{
}

controller_interface::controller_interface_ret_t
JointVelocityController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
                              const std::string &controller_name)
{
    // initialize lifecycle node
    auto ret = ControllerInterface::init(robot_hardware, controller_name);
    if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCLCPP_WARN(this->get_lifecycle_node()->get_logger(), "JointVelocityController init unsuccesful");
        return ret;
    }
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController init successful");

    return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}
controller_interface::controller_interface_ret_t
JointVelocityController::update()
{
    // RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController update called");
    auto timeNow = this->get_lifecycle_node()->get_clock()->now();
    auto timeElapsed = timeNow - previous_update_time_;
    previous_update_time_ = timeNow;
    for (size_t i = 0; i < registered_joint_state_handles_.size(); i++)
    {
        auto state_handle = registered_joint_state_handles_[i];
        auto error = desired_vels_[i] - state_handle->get_velocity();
        auto cmd = pid_controllers_[i]->compute_command(error, timeElapsed);
        auto cmd_handle = registered_joint_cmd_handles_[i];
        cmd_handle->set_cmd(cmd);
    }
    return hardware_interface::HW_RET_OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    if (auto sptr = robot_hardware_.lock())
    {
        registered_joint_state_handles_ = sptr->get_registered_joint_state_handles();
        registered_joint_cmd_handles_ = sptr->get_registered_joint_command_handles();

        if (registered_joint_cmd_handles_.size() != registered_joint_state_handles_.size())
        {
            RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "Number of state and command handles must be the same. Exiting.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        if (registered_joint_state_handles_.size() == 0)
        {
            RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "0 joint state handles registered. Exiting.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }
    else
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    previous_update_time_ = this->get_lifecycle_node()->get_clock()->now();    
    desired_vels_ = std::vector<double>();
    desired_vels_.resize(registered_joint_state_handles_.size());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController on_activate called");
    pid_controllers_ = std::vector<std::shared_ptr<control_helpers::Pid>>();
    std::string robotName;
    if (auto robot_hardware = robot_hardware_.lock())
    {
        robotName = robot_hardware->get_robot_name();
    }
    else
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    auto topicName = robotName + "/control";
    auto cb = [this](ros2_control_interfaces::msg::JointControl::UniquePtr msg) {
        RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Message received, length: %u", msg->desired_positions.size());
    };
    auto sub = this->get_lifecycle_node()->create_subscription<ros2_control_interfaces::msg::JointControl>(topicName, rclcpp::SensorDataQoS(), cb);

    for (auto &j : registered_joint_cmd_handles_)
    {
        auto controllerName = j->get_name();
        //TODO: get Pid gains from controller parameter server
        pid_controllers_.push_back(std::make_unique<control_helpers::Pid>(control_helpers::Pid::Gains(1, 0, 0, 0, 0, false)));
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController on_deactivate called");
    pid_controllers_.clear();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_deactivate()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController on_cleanup called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_cleanup()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_error(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController on_error called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_error()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointVelocityController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointVelocityController on_shutdown called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_shutdown()

} // namespace ros_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    ros_controllers::JointVelocityController, controller_interface::ControllerInterface)
