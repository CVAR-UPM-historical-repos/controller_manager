#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_core/node.hpp>
#include <as2_msgs/action/take_off.hpp>

#include <pluginlib/class_loader.hpp>
#include "controller_plugin_base/controller_base.hpp"

class ControllerManager : public as2::Node
{
public:

    ControllerManager() : as2::Node("controller_manager")
    {
        auto loader_ = std::make_shared<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>>("controller_plugin_base", "controller_plugin_base::ControllerBase");

        try
        {
            auto controller_ = loader_->createSharedInstance("df_plugin::PDController");
            auto a = loader_->getPluginManifestPath("df_plugin::PDController");
            auto b = loader_->getPluginXmlPaths();

            std :: cout << " Pluginlib ManifestPath" << a << std::endl;
            for (auto& c : b)
            {
                std::cout << "PLUGIN XML PATH: " << c << std::endl;
            }
            
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    };

private:
    std::shared_ptr<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>> loader_;
    std::shared_ptr<controller_plugin_base::ControllerBase> controller_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP
