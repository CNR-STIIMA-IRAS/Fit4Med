// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "plc_controller/plc_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <sys/types.h>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"


namespace
{
template <typename T>
void print_interface(const rclcpp::Logger & logger, const T & command_interfaces)
{
  for (const auto & [interface_name, value] : command_interfaces)
  {
    RCLCPP_ERROR(logger, "Got %s", interface_name.c_str());
  }
}

std::vector<hardware_interface::ComponentInfo> extract_gpios_from_hardware_info(
  const std::vector<hardware_interface::HardwareInfo> & hardware_infos)
{
  std::vector<hardware_interface::ComponentInfo> result;
  for (const auto & hardware_info : hardware_infos)
  {
    std::copy(
      hardware_info.gpios.begin(), hardware_info.gpios.end(), std::back_insert_iterator(result));
  }
  return result;
}
}  // namespace


namespace plc_controller
{

PLCController::PLCController() : controller_interface::ControllerInterface() {}


CallbackReturn PLCController::on_init()
try
{
  param_listener_ = std::make_shared<plc_controller_parameters::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}


CallbackReturn PLCController::on_configure(const rclcpp_lifecycle::State &)
try
{
  if (!update_dynamic_map_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  store_command_interface_types();
  store_state_interface_types();

  if (command_interface_types_.empty() && state_interface_types_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command or state interfaces are configured");
    return CallbackReturn::ERROR;
  }

  if (!command_interface_types_.empty())
  {
    plc_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/plc_commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });
  }

  plc_state_publisher_ =
    get_node()->create_publisher<StateType>("~/plc_states", rclcpp::SystemDefaultsQoS());

  realtime_plc_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StateType>>(plc_state_publisher_);
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}


controller_interface::InterfaceConfiguration
PLCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}


controller_interface::InterfaceConfiguration PLCController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  return state_interfaces_config;
}


CallbackReturn PLCController::on_activate(const rclcpp_lifecycle::State &)
{
  command_interfaces_map_ =
    create_map_of_references_to_interfaces(command_interface_types_, command_interfaces_);
  state_interfaces_map_ =
    create_map_of_references_to_interfaces(state_interface_types_, state_interfaces_);
  if (
    !check_if_configured_interfaces_matches_received(
      command_interface_types_, command_interfaces_map_) ||
    !check_if_configured_interfaces_matches_received(state_interface_types_, state_interfaces_map_))
  {
    return CallbackReturn::ERROR;
  }

  initialize_plc_state_msg();
  rt_command_ptr_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}


CallbackReturn PLCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}


controller_interface::return_type PLCController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  update_plc_states();
  return update_plc_commands();
}


bool PLCController::update_dynamic_map_parameters()
{
  auto logger = get_node()->get_logger();
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();
  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  return true;
}


void PLCController::store_command_interface_types()
{
  for (const auto & [gpio_name, interfaces] : params_.command_interfaces.gpios_map)
  {
    std::transform(
      interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
      std::back_inserter(command_interface_types_),
      [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
  }
}


bool PLCController::should_broadcast_all_interfaces_of_configured_gpios() const
{
  auto are_interfaces_empty = [](const auto & interfaces)
  { return interfaces.second.interfaces.empty(); };
  return std::all_of(
    params_.state_interfaces.gpios_map.cbegin(), params_.state_interfaces.gpios_map.cend(),
    are_interfaces_empty);
}


std::vector<hardware_interface::ComponentInfo> PLCController::get_gpios_from_urdf() const
try
{
  return extract_gpios_from_hardware_info(
    hardware_interface::parse_control_resources_from_urdf(get_robot_description()));
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during extracting gpios info from urdf %s \n", e.what());
  return {};
}


void PLCController::set_all_state_interfaces_of_configured_gpios()
{
  const auto gpios{get_gpios_from_urdf()};
  for (const auto & gpio_name : params_.gpios)
  {
    for (const auto & gpio : gpios)
    {
      if (gpio_name == gpio.name)
      {
        std::transform(
          gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
          std::back_insert_iterator(state_interface_types_),
          [&gpio_name](const auto & interface_name)
          { return gpio_name + '/' + interface_name.name; });
      }
    }
  }
}


void PLCController::store_state_interface_types()
{
  if (should_broadcast_all_interfaces_of_configured_gpios())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "State interfaces are not configured. All available interfaces of configured GPIOs will be "
      "broadcasted.");
    set_all_state_interfaces_of_configured_gpios();
    return;
  }

  for (const auto & [gpio_name, interfaces] : params_.state_interfaces.gpios_map)
  {
    std::transform(
      interfaces.interfaces.cbegin(), interfaces.interfaces.cend(),
      std::back_inserter(state_interface_types_),
      [&](const auto & interface_name) { return gpio_name + "/" + interface_name; });
  }
}


void PLCController::initialize_plc_state_msg()
{
  auto & plc_state_msg = realtime_plc_state_publisher_->msg_;
  const auto gpio_name = params_.gpios.front();
  plc_state_msg.interface_names = get_plc_state_interfaces_names(gpio_name);
  plc_state_msg.values = std::vector<uint8_t>(
    plc_state_msg.interface_names.size(),std::numeric_limits<uint8_t>::quiet_NaN());
}


InterfacesNames PLCController::get_plc_state_interfaces_names(
  const std::string & gpio_name) const
{
  InterfacesNames result;
  for (const auto & interface_name : state_interface_types_)
  {
    const auto it = state_interfaces_map_.find(interface_name);
    if (it != state_interfaces_map_.cend() && it->second.get().get_prefix_name() == gpio_name)
    {
      result.emplace_back(it->second.get().get_interface_name());
    }
  }
  return result;
}


template <typename T>
std::unordered_map<std::string, std::reference_wrapper<T>>
PLCController::create_map_of_references_to_interfaces(
  const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces)
{
  std::unordered_map<std::string, std::reference_wrapper<T>> map;
  for (const auto & interface_name : interfaces_from_params)
  {
    auto interface = std::find_if(
      configured_interfaces.begin(), configured_interfaces.end(),
      [&](const auto & configured_interface)
      {
        const auto full_name_interface_name = configured_interface.get_name();
        return full_name_interface_name == interface_name;
      });
    if (interface != configured_interfaces.end())
    {
      map.emplace(interface_name, std::ref(*interface));
    }
  }
  return map;
}


template <typename T>
bool PLCController::check_if_configured_interfaces_matches_received(
  const InterfacesNames & interfaces_from_params, const T & configured_interfaces)
{
  if (!(configured_interfaces.size() == interfaces_from_params.size()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %ld interfaces, got %ld", interfaces_from_params.size(),
      configured_interfaces.size());
    for (const auto & interface : interfaces_from_params)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %s", interface.c_str());
    }
    print_interface(get_node()->get_logger(), configured_interfaces);
    return false;
  }
  return true;
}


controller_interface::return_type PLCController::update_plc_commands()
{
  auto gpio_commands_ptr = rt_command_ptr_.readFromRT();
  if (!gpio_commands_ptr || !(*gpio_commands_ptr))
  {
    return controller_interface::return_type::OK;
  }

  const auto gpio_commands = *(*gpio_commands_ptr);
  const auto & gpio_name = params_.gpios.front();
  if (
    gpio_commands.values.size() !=
    gpio_commands.interface_names.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), " %s interfaces_names do not match the values",
      gpio_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  try
  {
    for (const auto & [interface_name, command_interface] : command_interfaces_map_)
    {
      auto it = std::find(
        gpio_commands.interface_names.begin(), gpio_commands.interface_names.end(), interface_name);
    
      if (it != gpio_commands.interface_names.end())
      {
        auto index = std::distance(gpio_commands.interface_names.begin(), it);
        command_interface.get().set_value(static_cast<double>(gpio_commands.values[index]));
      }
      else
      {
        RCLCPP_WARN(get_node()->get_logger(), "Interface %s not found in command message", interface_name.c_str());
      }
    }
    
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during applying command stage with message: %s \n",
      e.what());
  }

  return controller_interface::return_type::OK;
}


void PLCController::update_plc_states()
{
  if (!realtime_plc_state_publisher_ || !realtime_plc_state_publisher_->trylock())
  {
    return;
  }

  auto & plc_state_msg = realtime_plc_state_publisher_->msg_;
  try
  {
    plc_state_msg.values.clear();
    plc_state_msg.interface_names.clear();

    for (const auto & interface_name : state_interface_types_)
    {
      auto it = state_interfaces_map_.find(interface_name);
      if (it != state_interfaces_map_.end())
      {
        const auto & state_interface = it->second.get();
        plc_state_msg.interface_names.push_back(state_interface.get_interface_name());

        auto optional_value = state_interface.get_optional<double>();
        if (optional_value.has_value())
        {
          plc_state_msg.values.push_back(optional_value.value());
        }
        else
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve state value for %s", interface_name.c_str());
          plc_state_msg.values.push_back(std::numeric_limits<uint8_t>::quiet_NaN());
        }
      }
    }

  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during reading state interface, error: %s \n", e.what());
  }
  realtime_plc_state_publisher_->unlockAndPublish();
}


}  // namespace plc_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  plc_controller::PLCController, controller_interface::ControllerInterface)