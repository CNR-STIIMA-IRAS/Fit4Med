#ifndef CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_
#define CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_


#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_logger/controller_logger_parameters.hpp"
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace controller_logger
{
  using DataType = std_msgs::msg::Float64MultiArray;
  class ControllerLogger : public controller_interface::ChainableControllerInterface
  {
  public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    bool on_set_chained_mode(bool chained_mode) override;

  protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    bool read_from_interface_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>> rt_buffer_ptr_;
    rclcpp::Subscription<DataType>::SharedPtr joints_cmd_sub_;
    rclcpp::Publisher<DataType>::SharedPtr logger_pub_;
    std::vector<std::string> reference_interface_names_;
    std::vector<std::string> command_interface_names_;
    std::vector<std::string> joint_names_;
  };
} // namespace controller_logger

#endif  // CONTROLLER_LOGGER__CONTROLLER_LOGGER_HPP_