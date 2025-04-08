#ifndef PLC_CONTROLLER__PLC_CONTROLLER_HPP_
#define PLC_CONTROLLER__PLC_CONTROLLER_HPP_

#include <controller_interface/controller_interface_base.hpp>
#include <gpio_controllers/gpio_command_controller.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace plc_controller
{
  class PLCController : public gpio_controllers::GpioCommandController
  {
  public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    void update_plc_states();
    void apply_state_value(gpio_controllers::StateType & state_msg, std::size_t gpio_index, std::size_t interface_index) const;
    controller_interface::return_type update_plc_commands();
    void apply_command(gpio_controllers::CmdType & gpio_commands, std::size_t gpio_index, std::size_t command_interface_index) const;

  };
} // namespace plc_controller

#endif  // PLC_CONTROLLER__PLC_CONTROLLER_HPP_