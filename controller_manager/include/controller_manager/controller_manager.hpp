#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include <as2_core/control_mode_utils/control_mode_utils.hpp>
#include <as2_core/node.hpp>
#include <as2_msgs/action/take_off.hpp>
#include <filesystem>
#include <pluginlib/class_loader.hpp>

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "controller_plugin_base/controller_base.hpp"



// #define PLUGIN_NAME "controller_plugin_differential_flatness::PDController"
#define PLUGIN_NAME "controller_plugin_differential_flatness::PDController"

class ControllerManager : public as2::Node {
  public:
  ControllerManager() : as2::Node("controller_manager") {
    auto loader_ = std::make_shared<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>>(
        "controller_plugin_base", "controller_plugin_base::ControllerBase");

    try {
      auto controller_ = loader_->createSharedInstance(PLUGIN_NAME);
      std::filesystem::path manifest_path = loader_->getPluginManifestPath(PLUGIN_NAME);

      auto available_modes = as2::parse_available_modes_from_project_exports_path(manifest_path.parent_path());
      for (auto mode : available_modes) {
        RCLCPP_INFO(this->get_logger(), "Available mode: %c", mode);
      }
      controller_->setAvailableModes(available_modes);

      RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
    } catch (pluginlib::PluginlibException& ex) {
      printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }
  };

  private:
  std::shared_ptr<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>> loader_;
  std::shared_ptr<controller_plugin_base::ControllerBase> controller_;
};

#endif  // TAKE_OFF_BEHAVIOUR_HPP
