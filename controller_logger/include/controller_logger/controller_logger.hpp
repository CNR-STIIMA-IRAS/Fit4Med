#ifndef CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_
#define CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_


#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_logger/controller_logger_parameters.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace controller_logger
{
  class ControllerLogger : public controller_interface::ChainableControllerInterface
  {
  public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
    controller_interface::CallbackReturn on_init() override;

  protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr logger_publisher_;
    std::shared_ptr<ParamListener> param_listener_;
    bool read_from_interface_;
  };
} // namespace controller_logger

#endif  // CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_