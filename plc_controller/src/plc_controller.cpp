#include "plc_controller/plc_controller.hpp"
#include <gpio_controllers/gpio_command_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plc_controller
{
controller_interface::CallbackReturn PLCController::on_init()
{
  auto ret = GpioCommandController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PLCController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  update_plc_states();
  return update_plc_commands();
}

void PLCController::update_plc_states()
{
  if (!realtime_gpio_state_publisher_ || !realtime_gpio_state_publisher_->trylock())
  {
    return;
  }

  auto & gpio_state_msg = realtime_gpio_state_publisher_->msg_;
  gpio_state_msg.header.stamp = get_node()->now();
  for (std::size_t gpio_index = 0; gpio_index < gpio_state_msg.interface_groups.size();
       ++gpio_index)
  {
    for (std::size_t interface_index = 0;
         interface_index < gpio_state_msg.interface_values[gpio_index].interface_names.size();
         ++interface_index)
    {
      apply_state_value(gpio_state_msg, gpio_index, interface_index);
    }
  }
  realtime_gpio_state_publisher_->unlockAndPublish();
}

void PLCController::apply_state_value(gpio_controllers::StateType & state_msg, std::size_t gpio_index, std::size_t interface_index) const
{
  const auto interface_name =
    state_msg.interface_groups[gpio_index] + '/' +
    state_msg.interface_values[gpio_index].interface_names[interface_index];
  try
  {
    state_msg.interface_values[gpio_index].values[interface_index] =
      state_interfaces_map_.at(interface_name).get().get_value();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during reading state of: %s \n", interface_name.c_str());
  }
}
  
controller_interface::return_type PLCController::update_plc_commands()
{
  auto gpio_commands_ptr = rt_command_ptr_.readFromRT();
  if (!gpio_commands_ptr || !(*gpio_commands_ptr))
  {
    return controller_interface::return_type::OK;
  }

  auto gpio_commands = *(*gpio_commands_ptr);
  for (std::size_t gpio_index = 0; gpio_index < gpio_commands.interface_groups.size(); ++gpio_index)
  {
    const auto & gpio_name = gpio_commands.interface_groups[gpio_index];
    if (
      gpio_commands.interface_values[gpio_index].values.size() !=
      gpio_commands.interface_values[gpio_index].interface_names.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "For gpio %s interfaces_names do not match values",
        gpio_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    for (std::size_t command_interface_index = 0;
         command_interface_index < gpio_commands.interface_values[gpio_index].values.size();
         ++command_interface_index)
    {
      apply_command(gpio_commands, gpio_index, command_interface_index);
    }
  }
  return controller_interface::return_type::OK;
}

void PLCController::apply_command(gpio_controllers::CmdType & gpio_commands, std::size_t gpio_index, std::size_t command_interface_index) const
{
  const auto full_command_interface_name =
    gpio_commands.interface_groups[gpio_index] + '/' +
    gpio_commands.interface_values[gpio_index].interface_names[command_interface_index];
  try
  {
    command_interfaces_map_.at(full_command_interface_name)
      .get()
      .set_value(gpio_commands.interface_values[gpio_index].values[command_interface_index]);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during applying command stage of %s with message: %s \n",
      full_command_interface_name.c_str(), e.what());
  }
}

} // namespace controller_logger

PLUGINLIB_EXPORT_CLASS(plc_controller::PLCController, controller_interface::ControllerInterface)
