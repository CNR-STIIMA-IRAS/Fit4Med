#include "remapping_controller/remapping_controller.hpp"
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>

namespace remapping_controller
{

RemappingController::RemappingController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn RemappingController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  num_joints_ = params_.joints.size();
  num_interfaces_ = params_.input_command_interfaces.size();

  last_reference_.positions.assign(num_joints_, 0.0);
  last_reference_.velocities.assign(num_joints_, 0.0);
  last_reference_.accelerations.assign(num_joints_, 0.0);
  reference_ = last_reference_;
  last_commanded_.assign(num_joints_ * num_interfaces_, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemappingController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();

  num_joints_ = params_.joints.size();

  if (params_.input_command_interfaces.size() != params_.output_command_interfaces.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Mismatch: input_command_interfaces (%zu) and output_command_interfaces (%zu)",
                 params_.input_command_interfaces.size(),
                 params_.output_command_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto contains_interface_type =
    [](const std::vector<std::string> & interface_type_list, const std::string & interface_type)
  {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
           interface_type_list.end();
  };

  has_position_command_interface_ =
    contains_interface_type(params_.input_command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.input_command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.input_command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.input_command_interfaces, hardware_interface::HW_IF_EFFORT);

  RCLCPP_INFO(get_node()->get_logger(), "RemappingController configured with %zu joints.",
              params_.joints.size());

  // auto joint_command_callback =
  //   [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg)
  // { input_joint_command_.set(*msg); };
  // input_joint_command_subscriber_ =
  //   get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
  //     "~/joint_references", rclcpp::SystemDefaultsQoS(), joint_command_callback);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemappingController::on_activate(const rclcpp_lifecycle::State &)
{
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemappingController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RemappingController::update_reference_from_subscribers(
    const rclcpp::Time & , const rclcpp::Duration & )
{
   // update input reference from ros subscriber message
  return controller_interface::return_type::OK;
}

controller_interface::return_type RemappingController::update_and_write_commands(
  const rclcpp::Time & , const rclcpp::Duration & )
{
  for (std::size_t i = 0; i < params_.joints.size();  i++)
  {
    for (std::size_t j = 0; j <params_.output_command_interfaces.size(); j++)
    {
      const auto idx = i * params_.output_command_interfaces.size() + j;
      // RCLCPP_ERROR(get_node()->get_logger(), "Command Interface: %s, idx: %zu", command_interfaces_.at(idx).get_name().c_str(), idx);
      if(!command_interfaces_[idx].set_value(reference_interfaces_[idx]))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value for joint %s and interface %s",
                     params_.joints[i].c_str(), params_.output_command_interfaces[j].c_str());
        return controller_interface::return_type::ERROR;
      }
    }
    last_commanded_ = reference_interfaces_;
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> RemappingController::on_export_reference_interfaces()
{
  // controller_interface::ChainableControllerInterface::on_export_reference_interfaces();
  auto reference_interfaces_resize = [this](std::size_t num_command_interfaces, double default_value) -> void
  {
    reference_interfaces_.resize(num_command_interfaces, default_value); 
    position_reference_ = {};
    velocity_reference_ = {};
    acceleration_reference_ = {};

    for (size_t i = 0; i < params_.joints.size(); ++i) {
      for (size_t j = 0; j < params_.input_command_interfaces.size(); ++j) {

        auto idx = i * params_.input_command_interfaces.size() + j;

        if (hardware_interface::HW_IF_POSITION == params_.input_command_interfaces.at(j))
        {
          position_reference_.emplace_back(reference_interfaces_.at(idx));
        }
        else if (hardware_interface::HW_IF_VELOCITY == params_.input_command_interfaces.at(j))
        {
          velocity_reference_.emplace_back(reference_interfaces_.at(idx));
        }
        else if (hardware_interface::HW_IF_ACCELERATION == params_.input_command_interfaces.at(j))
        {
          acceleration_reference_.emplace_back(reference_interfaces_.at(idx));
        }
      }
    }
  };

  // size of the interfaces
  const size_t num_command_interfaces = params_.joints.size() * params_.input_command_interfaces.size(); 

  // Whent the reference_interfaces_ is resized, we must modify accordingly the position and velocity wrappers.
  reference_interfaces_resize(num_command_interfaces,std::numeric_limits<double>::quiet_NaN());

  exported_reference_interface_names_.clear();
  exported_reference_interface_names_.reserve(num_command_interfaces);
  for (size_t i = 0; i < params_.joints.size(); ++i) 
  {
     for (size_t j = 0; j < params_.input_command_interfaces.size(); ++j) 
     {
       exported_reference_interface_names_.emplace_back(
        params_.joints[i] + "/" + params_.input_command_interfaces[j]);
     }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Exporting %zu reference interfaces.", num_command_interfaces);
  RCLCPP_INFO(get_node()->get_logger(), "Actual Exported Reference Interface Names: (%zu)", exported_reference_interface_names_.size());
  for (size_t i = 0; i < exported_reference_interface_names_.size(); ++i)
  {
    RCLCPP_INFO(get_node()->get_logger(), "- %s", exported_reference_interface_names_[i].c_str());
  }

  return ChainableControllerInterface::on_export_reference_interfaces();
}

  

controller_interface::InterfaceConfiguration RemappingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // The controller claims the output remapped interfaces
  for (const auto &joint : params_.joints)
  {
    for (const auto &iface : params_.output_command_interfaces)
    {
      config.names.push_back(joint + "/" + iface);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration RemappingController::state_interface_configuration() const
{
  // This controller does not read any state interface
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

}  // namespace remapping_controller

PLUGINLIB_EXPORT_CLASS(remapping_controller::RemappingController, controller_interface::ChainableControllerInterface)
