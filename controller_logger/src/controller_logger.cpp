#include "controller_logger/controller_logger.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace controller_logger
{
controller_interface::CallbackReturn ControllerLogger::on_init()
{
  auto ret = ForwardCommandController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;

  param_listener_ = std::make_shared<ParamListener>(get_node());
  logger_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/logged_commands", 10);

  get_node()->get_parameter("read_from_interface", read_from_interface_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ControllerLogger::command_interface_configuration() const
{
  return ForwardCommandController::command_interface_configuration();
}

controller_interface::return_type ControllerLogger::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // if (read_from_interface_ && logger_publisher_)
  // {
  //   logger_publisher_->msg_.data.clear();
  //   for (const auto & interface : command_interfaces_)
  //   {
  //     logger_publisher_->msg_.data.push_back(interface.get().get_value());
  //   }
  //   logger_publisher_->unlockAndPublish();
  // }

  return controller_interface::return_type::OK;
}
} // namespace controller_logger

PLUGINLIB_EXPORT_CLASS(controller_logger::ControllerLogger, controller_interface::ControllerInterface)
