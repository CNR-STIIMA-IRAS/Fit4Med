#include "controller_logger/controller_logger.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace controller_logger
{
controller_interface::CallbackReturn ControllerLogger::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  logger_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/logged_commands", 10);

  get_node()->get_parameter("read_from_interface", read_from_interface_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ControllerLogger::command_interface_configuration() const
{
  auto state_interfaces_config_names = std::vector<std::string>{"dummy_interface"};
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

controller_interface::InterfaceConfiguration ControllerLogger::state_interface_configuration() const
{
  auto state_interfaces_config_names = std::vector<std::string>{"dummy_interface"};
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}


controller_interface::return_type update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period) {

  return controller_interface::return_type::OK;
}

} // namespace controller_logger

PLUGINLIB_EXPORT_CLASS(controller_logger::ControllerLogger, controller_interface::ChainableControllerInterface)
