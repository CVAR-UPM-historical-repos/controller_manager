#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include <as2_core/names/topics.hpp>
#include <as2_core/synchronous_service_client.hpp>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include "as2_control_command_handlers/acro_control.hpp"
#include "as2_core/control_mode_utils/control_mode_utils.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace controller_plugin_base {

class ControllerBase {
  private:
  std::vector<uint8_t> available_modes_in_;
  std::vector<uint8_t> available_modes_out_;
  std::vector<uint8_t> platform_available_modes_in_;


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_traj_sub_;

  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  as2_msgs::msg::PlatformInfo platform_info_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr
      set_control_mode_client_;

  bool control_mode_established_ = false;
  bool motion_reference_adquired_ = false;
  bool odometry_adquired_= false;


  public:
  ControllerBase(){};

  void initialize(as2::Node* node_ptr) {
    node_ptr_ = node_ptr;
    odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        as2_names::topics::self_localization::odom, as2_names::topics::self_localization::qos,
        std::bind(&ControllerBase::odom_callback, this, std::placeholders::_1));
    ref_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::motion_reference::pose, as2_names::topics::motion_reference::qos,
        std::bind(&ControllerBase::ref_pose_callback, this, std::placeholders::_1));
    ref_twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::motion_reference::twist, as2_names::topics::motion_reference::qos,
        std::bind(&ControllerBase::ref_twist_callback, this, std::placeholders::_1));
    ref_traj_sub_ = node_ptr_->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        as2_names::topics::motion_reference::trajectory, as2_names::topics::motion_reference::qos,
        std::bind(&ControllerBase::ref_traj_callback, this, std::placeholders::_1));
    platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
        as2_names::topics::platform::info, as2_names::topics::platform::qos,
        std::bind(&ControllerBase::platform_info_callback, this, std::placeholders::_1));

    set_control_mode_client_ =
        std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>(
            as2_names::services::platform::set_platform_control_mode);

    using namespace std::chrono_literals;
    // FIXME: Hardcoded timer period
    control_timer_ = node_ptr_->create_wall_timer(
        10ms, std::bind(&ControllerBase::control_timer_callback, this));

    set_control_mode_srv_ = node_ptr->create_service<as2_msgs::srv::SetControlMode>(
        as2_names::services::controller::set_control_mode,
        std::bind(&ControllerBase::setControlModeSrvCall, this,
                  std::placeholders::_1,  // Corresponds to the 'request'  input
                  std::placeholders::_2   // Corresponds to the 'response' input
                  ));

    //
    ownInitialize();
  };

  virtual void ownInitialize(){};
  virtual void updateState(const nav_msgs::msg::Odometry& odom) = 0;

  virtual void updateReference(const geometry_msgs::msg::PoseStamped& ref){};
  virtual void updateReference(const geometry_msgs::msg::TwistStamped& ref){};
  virtual void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint& ref){};
  virtual void updateReference(const as2_msgs::msg::Thrust& ref){};

  virtual void computeOutput(geometry_msgs::msg::PoseStamped& pose,
                             geometry_msgs::msg::TwistStamped& twist,
                             as2_msgs::msg::Thrust& thrust) = 0;

  virtual bool setMode(const as2_msgs::msg::ControlMode& mode_in,
                       const as2_msgs::msg::ControlMode& mode_out) = 0;

  void setInputControlModesAvailables(const std::vector<uint8_t>& available_modes) {
    available_modes_in_ = available_modes;
  };

  void setOutputControlModesAvailables(const std::vector<uint8_t>& available_modes) {
    available_modes_out_ = available_modes;
  };

  virtual ~ControllerBase(){};

  protected:
  as2::Node* node_ptr_;

  private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry_adquired_ = true;
      updateState(*msg); };
  void ref_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    motion_reference_adquired_ = true;
    updateReference(*msg);
  };
  void ref_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    motion_reference_adquired_ = true;
    updateReference(*msg);
  };
  void ref_traj_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
    motion_reference_adquired_ = true;
    updateReference(*msg);
  };

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
    platform_info_ = *msg;
  };

  void control_timer_callback() {
    if (!platform_info_.offboard || !platform_info_.armed) {
      return;
    }

    if (! odometry_adquired_ || ! motion_reference_adquired_) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for odometry and motion reference");
      return;
    }

    if (!control_mode_established_) {
      // FIXME: Hardcoded control mode
      as2_msgs::msg::ControlMode mode_in, mode_out;

      mode_in.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
      mode_in.control_mode = as2_msgs::msg::ControlMode::SPEED;
      mode_in.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

      mode_out.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
      mode_out.control_mode = as2_msgs::msg::ControlMode::ACRO;
      mode_out.reference_frame = as2_msgs::msg::ControlMode::BODY_FLU_FRAME;

      control_mode_established_ = setMode(mode_in, mode_out);
    }

    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TwistStamped twist;
    as2_msgs::msg::Thrust thrust;
    computeOutput(pose, twist, thrust);
    static as2::controlCommandsHandlers::AcroControl acro_control(node_ptr_);
    acro_control.sendAngleRatesWithThrust(twist.twist.angular.x, twist.twist.angular.y,
                                          twist.twist.angular.z, thrust.thrust);
  };

  void setControlModeSrvCall(
      const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
      as2_msgs::srv::SetControlMode::Response::SharedPtr response) {
    // FIXME: Hardcoded control mode

    RCLCPP_WARN(node_ptr_->get_logger(), "setControlModeSrvCall TRUE HARD CODED");
    response->success = true;
  }

  bool setPlatformControlMode(const as2_msgs::msg::ControlMode& mode) {
    // FIXME : THIS SHOULD SET THE CONTROL MODE OF THE PLATFORM
    as2_msgs::srv::SetControlMode::Request set_control_mode_req;
    set_control_mode_req.control_mode = mode;

    auto request = set_control_mode_client_->sendRequest(set_control_mode_req);
    if (request.success) {
      platform_info_.current_control_mode = mode;
    }

    return request.success;
  };
};  //  ControllerBase

};  // namespace controller_plugin_base

#endif  // CONTROLLER_BASE_HPP
