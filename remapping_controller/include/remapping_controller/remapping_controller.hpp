#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "remapping_controller/remapping_controller_parameters.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace remapping_controller
{

class RemappingController : public controller_interface::ChainableControllerInterface
{
public:
  RemappingController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  
  bool has_position_command_interface_;
  bool has_velocity_command_interface_;
  bool has_acceleration_command_interface_;
  bool has_effort_command_interface_;

  std::vector<std::reference_wrapper<double>> position_reference_;
  std::vector<std::reference_wrapper<double>> velocity_reference_;
  std::vector<std::reference_wrapper<double>> acceleration_reference_;
  trajectory_msgs::msg::JointTrajectoryPoint reference_, last_reference_;
  size_t num_joints_ = 0, num_interfaces_ = 0;

  std::vector<double> last_commanded_;
};

}  // namespace remapping_controller
