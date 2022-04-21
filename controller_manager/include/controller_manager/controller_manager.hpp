#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/take_off.hpp>

#include <pluginlib/class_loader.hpp>
#include "takeoff_plugin_base/takeoff_base.hpp"

class ControllerManager : public as2::BasicBehaviour<as2_msgs::action::TakeOff>
{
public:
    using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;

    ControllerManager() : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff)
    {
        this->declare_parameter("default_takeoff_altitude");
        this->declare_parameter("default_takeoff_speed");
        this->declare_parameter("takeoff_height_threshold");

        auto loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>("takeoff_plugin_base", "takeoff_base::TakeOffBase");

        try
        {
            takeoff_speed_ = loader_->createSharedInstance("takeoff_plugins::TakeOffSpeed");
            takeoff_speed_->initialize(this, this->get_parameter("takeoff_height_threshold").as_double());
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    };

private:
    std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
    std::shared_ptr<takeoff_base::TakeOffBase> takeoff_speed_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP
