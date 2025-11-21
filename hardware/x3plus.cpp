// Copyright 2020 ros2_control Development Team
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

// should based on the dir $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
// which is configured in the cmakelist file

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include <grpcpp/grpcpp.h>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lqtech_ros2_x3plus/x3plus.hpp"
#include "lqtech_ros2_x3plus/x3plus.pb.h"
#include "lqtech_ros2_x3plus/x3plus.grpc.pb.h"


ABSL_FLAG(std::string, target, "192.168.31.142:50051", "Server address");

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using x3plus::RosmasterServices;
using x3plus::JointPosititonArray;
using x3plus::Empty;
using x3plus::SingleJointPositionRequest;
using x3plus::ResultResponse;

namespace lqtech_ros2_x3plus
{
hardware_interface::CallbackReturn X3PlusPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  std::string target_str = absl::GetFlag(FLAGS_target);
  auto channel = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
  stub_ = RosmasterServices::NewStub(channel);

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RRBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X3PlusPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
X3PlusPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
X3PlusPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn X3PlusPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X3PlusPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type X3PlusPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return this->grpc_get_joint_array();
}

hardware_interface::return_type X3PlusPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  return this->grpc_set_joint_single("arm_joint1", hw_commands_[0]);
    // // Simulate sending commands to the hardware
    // ss << std::fixed << std::setprecision(2) << std::endl
    //    << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // return hardware_interface::return_type::OK;
}

// must specify 'X3PlusPositionOnlyHardware::' before method 'SayHello', otherwise, stub_ could not be found 
hardware_interface::return_type X3PlusPositionOnlyHardware::grpc_get_joint_array() {
  Empty request;
  JointPosititonArray reply;
  ClientContext context;
  Status status = stub_->getJointPositionArray(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    const auto& reply_array = reply.joint_array();
    // 2. Create the target std::vector<double>
    std::vector<double> position_in_degree;
    for (int32_t value : reply_array) {
        position_in_degree.push_back(static_cast<double>(value));
    }
    std::vector<double> position_x3plus = degrees_to_radians_x3plus(position_in_degree);
    for (int i = 0; i < position_x3plus.size(); ++i) {
      double value = position_x3plus[i];
      hw_states_[i] = value;
      //std::cout << "read Joint " << i << ": " << value << std::endl;
    }
    return hardware_interface::return_type::OK;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type X3PlusPositionOnlyHardware::grpc_set_joint_single(std::string joint_name, double joint_value_radius){
  // Data we are sending to the server.
  SingleJointPositionRequest request;
  request.set_joint_name(joint_name);
  request.set_joint_value(radian_to_degree_x3plus(joint_value_radius));
  ResultResponse reply;
  ClientContext context;
  Status status = stub_->setJointPositionSingle(&context, request, &reply);

  if (status.ok()) {
    return hardware_interface::return_type::OK;
  } else {
    return hardware_interface::return_type::ERROR;
  }
}


std::vector<double> X3PlusPositionOnlyHardware::degrees_to_radians_x3plus(std::vector<double> joints){
    std::vector<double> mid = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
    std::vector<double> array(joints.size());

    for (size_t i = 0; i < joints.size(); ++i) {
        array[i] = joints[i] - mid[i];
    }
    const double DEG2RAD = M_PI / 180.0;
    std::vector<double> position_x3plus(array.size());
    for (size_t i = 0; i < array.size(); ++i) {
        position_x3plus[i] = array[i] * DEG2RAD;
    }
    return position_x3plus;
}


int X3PlusPositionOnlyHardware::radian_to_degree_x3plus(double joint_radius){
  // Use the built-in M_PI constant
  const double RAD2DEG = 180.0 / M_PI;
  int joint_position_int = static_cast<int>(joint_radius * RAD2DEG + 90);
  return joint_position_int;
}

std::vector<int> X3PlusPositionOnlyHardware::radian_to_degree_x3plus_array(std::vector<double> joint_position_radius){
  std::vector<double> result(joint_position_radius.size());
  for (size_t i = 0; i < joint_position_radius.size(); ++i) {
    result[i] = this->radian_to_degree_x3plus(joint_position_radius[i]);
  }
}

}  // namespace lqtech_ros2_x3plus

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lqtech_ros2_x3plus::X3PlusPositionOnlyHardware, hardware_interface::SystemInterface)
