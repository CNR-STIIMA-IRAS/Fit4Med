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

#ifndef PLC_CONTROLLER__PLC_CONTROLLER_HPP_
#define PLC_CONTROLLER__PLC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "plc_controller_msgs/msg/plc_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "plc_controller/plc_controller_parameters.hpp"

namespace plc_controller
{
using CmdType = plc_controller_msgs::msg::PlcController;
using StateType = plc_controller_msgs::msg::PlcController;
using CallbackReturn = controller_interface::CallbackReturn;
using InterfacesNames = std::vector<std::string>;
using MapOfReferencesToCommandInterfaces = std::unordered_map<
  std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using MapOfReferencesToStateInterfaces =
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using StateInterfaces =
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

class PLCController : public controller_interface::ControllerInterface
{
public:
  PLCController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void store_command_interface_types();
  void store_state_interface_types();
  void initialize_plc_state_msg();
  void update_plc_states();
  controller_interface::return_type update_plc_commands();
  template <typename T>
  std::unordered_map<std::string, std::reference_wrapper<T>> create_map_of_references_to_interfaces(
    const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces);
  template <typename T>
  bool check_if_configured_interfaces_matches_received(
    const InterfacesNames & interfaces_from_params, const T & configured_interfaces);
  void apply_state_value(StateType & state_msg, std::size_t interface_index) const;
  void apply_command(const CmdType & gpio_command, std::size_t command_interface_index) const;
  bool should_broadcast_all_interfaces_of_configured_gpios() const;
  void set_all_state_interfaces_of_configured_gpios();
  InterfacesNames get_plc_state_interfaces_names(const std::string & gpio_name) const;
  bool update_dynamic_map_parameters();
  std::vector<hardware_interface::ComponentInfo> get_gpios_from_urdf() const;

protected:
  InterfacesNames command_interface_types_;
  InterfacesNames state_interface_types_;
  MapOfReferencesToCommandInterfaces command_interfaces_map_;
  MapOfReferencesToStateInterfaces state_interfaces_map_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_{};
  rclcpp::Subscription<CmdType>::SharedPtr plc_command_subscriber_{};

  std::shared_ptr<rclcpp::Publisher<StateType>> plc_state_publisher_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_plc_state_publisher_{};

  std::shared_ptr<plc_controller_parameters::ParamListener> param_listener_{};
  plc_controller_parameters::Params params_;
};

}  // namespace plc_controller

#endif  // PLC_CONTROLLER__PLC_CONTROLLER_HPP_