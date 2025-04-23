#include "controller_logger/controller_logger.hpp"
#include <controller_interface/controller_interface_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/helpers.hpp"


namespace controller_logger
{

controller_interface::CallbackReturn ControllerLogger::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerLogger::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    logger_pub_ = get_node()->create_publisher<DataType>("~/logged_commands", 10);
    params_ = param_listener_->get_params();
    command_interface_names_ = params_.interfaces;
  
    // pre-reserve command interfaces
    command_interfaces_.reserve(command_interface_names_.size());
  
    RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");
  
    // The names should be in the same order as for command interfaces for easier matching
    reference_interface_names_ = command_interface_names_;
    // for any case make reference interfaces size of command interfaces
    reference_interfaces_.resize(
      reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());
  
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerLogger::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControllerLogger::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ControllerLogger::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::InterfaceConfiguration ControllerLogger::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ControllerLogger::state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type ControllerLogger::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.reserve(command_interfaces_.size());
  for (const auto & interface : command_interfaces_)
  {
    msg.data.push_back(interface.get_value());
  }
  logger_pub_->publish(msg);

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
ControllerLogger::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

controller_interface::return_type ControllerLogger::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if (!(!joint_commands || !(*joint_commands)))
  {
    if (reference_interfaces_.size() != (*joint_commands)->data.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "command size (%zu) does not match number of reference interfaces (%zu)",
        (*joint_commands)->data.size(), reference_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = (*joint_commands)->data;
  }

  return controller_interface::return_type::OK;
}

} // namespace controller_logger

PLUGINLIB_EXPORT_CLASS(controller_logger::ControllerLogger, controller_interface::ChainableControllerInterface)
