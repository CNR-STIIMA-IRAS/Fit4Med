#include "fit4med_ee_joint_hw/fit4med_ee_joint_hw.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fit4med_ee_joint_hw
{

hardware_interface::CallbackReturn EEJointPositionSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  node_ = rclcpp::Node::make_shared("fit4med_ee_joint_position_node");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EEJointPositionSystem::on_activate(const rclcpp_lifecycle::State &)
{
  position_ = 0.0;
  command_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EEJointPositionSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EEJointPositionSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EEJointPositionSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back( hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &command_) );
  command_interfaces.emplace_back( hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &command_vel_) );
  return command_interfaces;
}

hardware_interface::return_type EEJointPositionSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Assume joint instantly follows the command
  position_ = command_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EEJointPositionSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // No-op since this is a simulated interface
  return hardware_interface::return_type::OK;
}

}  // namespace fit4med_ee_joint_hw

PLUGINLIB_EXPORT_CLASS(fit4med_ee_joint_hw::EEJointPositionSystem, hardware_interface::SystemInterface)
