#ifndef CONTROLLER_MANAGER_HPP
#define CONTROLLER_MANAGER_HPP

#include <as2_core/control_mode_utils/control_mode_utils.hpp>
#include <as2_core/node.hpp>
#include <as2_core/yaml_utils/yaml_utils.hpp>
#include <filesystem>
#include <pluginlib/class_loader.hpp>

#include "controller_plugin_base/controller_base.hpp"
#include "as2_msgs/msg/controller_info.hpp"

#define PLUGIN_NAME "controller_plugin_differential_flatness::DFPlugin"

class ControllerManager : public as2::Node
{
public:
  ControllerManager() : as2::Node("controller_manager")
  {
    loader_ = std::make_shared<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>>(
        "controller_plugin_base", "controller_plugin_base::ControllerBase");
    try
    {
      controller_ = loader_->createSharedInstance(PLUGIN_NAME);
      RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
    }
    catch (pluginlib::PluginlibException &ex)
    {
      printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    controller_->initialize(this);
    std::filesystem::path manifest_path = loader_->getPluginManifestPath(PLUGIN_NAME);
    config_available_control_modes(manifest_path.parent_path());

    mode_pub_ = this->create_publisher<as2_msgs::msg::ControllerInfo>(
        as2_names::topics::controller::info,
        as2_names::topics::controller::qos_info);

    using namespace std::chrono_literals;
    // FIXME: Hardcoded timer period
    mode_timer_ = this->create_wall_timer(
      100ms, std::bind(&ControllerManager::mode_timer_callback, this)
    );
  };

private:
  // TODO: move to plugin base?
  void config_available_control_modes(const std::filesystem::path project_path)
  {
    auto available_input_modes = as2::parse_uint_from_string(
        as2::find_tag_from_project_exports_path<std::string>(project_path, "input_control_modes"));
    for (auto mode : available_input_modes)
    {
      std::cout << "mode: " << (int)mode << std::endl;
    }
    auto available_output_modes = as2::parse_uint_from_string(
        as2::find_tag_from_project_exports_path<std::string>(project_path, "output_control_modes"));
    for (auto mode : available_output_modes)
    {
      std::cout << "mode: " << (int)mode << std::endl;
    }

    controller_->setInputControlModesAvailables(available_input_modes);
    controller_->setOutputControlModesAvailables(available_output_modes);
  };

  void mode_timer_callback()
  {
    as2_msgs::msg::ControllerInfo msg;
    msg.header.stamp = this->now();
    msg.current_control_mode = controller_->getMode();
    mode_pub_->publish(msg);
  };

private:
  std::shared_ptr<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>> loader_;
  std::shared_ptr<controller_plugin_base::ControllerBase> controller_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
};

#endif // CONTROLLER_MANAGER_HPP
