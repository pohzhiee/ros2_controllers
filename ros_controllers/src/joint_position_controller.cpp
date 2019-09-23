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

#include "ros_controllers/joint_position_controller.hpp"

#include <string>
#include <sstream>
#include <random>
#include <memory>
#include <exception>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "rcutils/logging_macros.h"

namespace ros_controllers
{

//Helper functions
unsigned int random_char()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    return dis(gen);
}

std::string generate_hex(const unsigned int len)
{
    std::stringstream ss;
    for (size_t i = 0; i < len; i++)
    {
        const auto rc = random_char();
        std::stringstream hexstream;
        hexstream << std::hex << rc;
        auto hex = hexstream.str();
        ss << (hex.length() < 2 ? '0' + hex : hex);
    }
    return ss.str();
}



JointPositionController::JointPositionController()
    : controller_interface::ControllerInterface()
{
}

controller_interface::controller_interface_ret_t
JointPositionController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
                              const std::string &controller_name)
{
    // initialize lifecycle node
    auto ret = ControllerInterface::init(robot_hardware, controller_name);
    if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCLCPP_WARN(this->get_lifecycle_node()->get_logger(), "JointPositionController init unsuccesful");
        return ret;
    }
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController init successful");

    return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}
controller_interface::controller_interface_ret_t
JointPositionController::update()
{
    // RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController update called");
    auto timeNow = this->get_lifecycle_node()->get_clock()->now();
    auto timeElapsed = timeNow - previous_update_time_;
    previous_update_time_ = timeNow;
    for (size_t i = 0; i < registered_joint_state_handles_.size(); i++)
    {
        auto state_handle = registered_joint_state_handles_[i];
        auto error = desired_pos_vec_[i] - state_handle->get_position();
        auto cmd = pid_controllers_[i]->compute_command(error, timeElapsed);
        auto cmd_handle = registered_joint_cmd_handles_[i];
        cmd_handle->set_cmd(cmd);
    }
    return hardware_interface::HW_RET_OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_configure(const rclcpp_lifecycle::State &previous_state)
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
    desired_pos_vec_ = std::vector<double>();
    desired_pos_vec_.resize(registered_joint_state_handles_.size());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void JointPositionController::desired_position_subscrition_callback(ros2_control_interfaces::msg::JointControl::UniquePtr msg)
{
    // RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Message received, length: %u", msg->desired_positions.size());
    auto msg_size = msg->desired_positions.size();
    auto names_size = msg->joint_names.size();
    auto vec_size = desired_pos_vec_.size();
    if (names_size != msg_size)
    {
        names_size = 0;
        RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(),
                         "Number of joint names don't correspond to number of desired positions, ignoring names");
    }
    if (msg_size > vec_size)
    {
        RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(), "Subscribed desired position more than robot can handle, truncating...");
        msg_size = vec_size;
    }
    else if (msg_size < vec_size)
    {
        RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(),
                         "Subscribed desired position less than total joints in robot, ignoring control for joints at the end...");
    }
    // If number of joint names matches number of desired positions, try to find the index of the position corresponding to each joint name
    // Such that the input data could be mapped correctly to the internal data structure using joint names
    if (names_size != 0)
    {
        for (size_t i = 0; i < msg->joint_names.size(); i++)
        {
            std::string &msg_joint_name = msg->joint_names[i];
            for (size_t j = 0; j < registered_joint_state_handles_.size(); j++)
            {
                auto &state_handle = registered_joint_state_handles_[j];
                auto name = state_handle->get_name();
                if (msg_joint_name.compare(name) == 0)
                {
                    desired_pos_vec_[j] = msg->desired_positions[i];
                }
            }
        }
    }
    else
    {
        for (size_t i = 0; i < msg_size; i++)
        {
            desired_pos_vec_[i] = msg->desired_positions[i];
        }
    }
}

control_helpers::Pid::Gains JointPositionController::get_controller_pid()
{
    using GetControllerPid = parameter_server_interfaces::srv::GetControllerPid;
    using namespace std::chrono_literals;

    auto gain = control_helpers::Pid::Gains();
    auto nodeTemp = std::make_shared<rclcpp::Node>("n_"+ generate_hex(10));
    gain.p_gain_ = 1;
    auto client = nodeTemp->create_client<GetControllerPid>("/GetControllerPid");
    client->wait_for_service(1.5s);
    if (client->service_is_ready())
    {
        auto req = std::make_shared<parameter_server_interfaces::srv::GetControllerPid::Request>();
        req->controller = this->get_lifecycle_node()->get_name();
        auto resp = client->async_send_request(req);
        RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Getting PID parameters for controller %s...", req->controller.c_str());
        auto spin_status = rclcpp::spin_until_future_complete(nodeTemp, resp, 5s);
        if (spin_status == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            auto status = resp.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto res = resp.get();
                gain.p_gain_ = res->p;
                gain.i_gain_ = res->i;
                gain.d_gain_ = res->d;
                gain.i_max_ = res->i_max;
                gain.i_min_ = res->i_min;
                gain.antiwindup_ = res->antiwindup;
                return gain;
            }
            else
            {
                RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "GetControllerPid service failed to execute");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "GetControllerPid service failed to execute (spin failed)");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "GetControllerPid service failed to start, check that parameter server is launched");
    }
    RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "Get controller gain failed, using P only controller with gain of 1");
    return gain;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_activate called");
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
    subscription_ = this->get_lifecycle_node()->create_subscription<ros2_control_interfaces::msg::JointControl>(
        topicName, rclcpp::SensorDataQoS(), std::bind(&JointPositionController::desired_position_subscrition_callback, this, std::placeholders::_1));
    auto pidParams = control_helpers::Pid::Gains();
    pidParams.p_gain_ = 1.0;
    try{
        pidParams = get_controller_pid();
    }
    catch(std::exception &e){
        RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "Error: %s", e.what());
        throw e;
    }
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Finished getting pid params, creating pid controllers");
    for (auto &j : registered_joint_cmd_handles_)
    {
        pid_controllers_.push_back(std::make_shared<control_helpers::Pid>(pidParams));
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_deactivate called");
    pid_controllers_.clear();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_deactivate()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_cleanup called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_cleanup()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_error(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_error called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_error()

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_shutdown called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} //end of on_shutdown()


} // namespace ros_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    ros_controllers::JointPositionController, controller_interface::ControllerInterface)
