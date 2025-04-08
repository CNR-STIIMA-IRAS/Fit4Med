#ifndef CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_
#define CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_


#include <forward_command_controller/forward_command_controller.hpp>
#include "controller_logger/controller_logger_parameters.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace controller_logger
{
  class ControllerLogger : public forward_command_controller::ForwardCommandController
  {
  public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    controller_interface::CallbackReturn on_init() override;

  protected:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr logger_publisher_;
    std::shared_ptr<ParamListener> param_listener_;
    bool read_from_interface_;
  };
} // namespace controller_logger

#endif  // CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_