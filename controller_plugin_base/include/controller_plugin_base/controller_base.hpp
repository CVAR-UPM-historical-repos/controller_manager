#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include <as2_core/names/topics.hpp>
#include <as2_core/synchronous_service_client.hpp>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <vector>
#include <algorithm>
#include <cstdint>

#include "as2_core/control_mode_utils/control_mode_utils.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <as2_core/control_mode_utils/control_mode_utils.hpp>


namespace controller_plugin_base {

class ControllerBase {
  private:
  std::vector<uint8_t> controller_available_modes_in_;
  std::vector<uint8_t> controller_available_modes_out_;
  std::vector<uint8_t> platform_available_modes_in_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_traj_sub_;


  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;


  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  as2_msgs::msg::PlatformInfo platform_info_;

  as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;
  as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>::SharedPtr
      list_control_modes_client_;

  bool control_mode_established_ = false;
  bool motion_reference_adquired_ = false;
  bool odometry_adquired_ = false;

  as2_msgs::msg::ControlMode input_mode_;
  as2_msgs::msg::ControlMode output_mode_;

  uint8_t prefered_output_mode_ = 0b00000000; // by default, no output mode is prefered

  public:
  ControllerBase(){};

  void initialize(as2::Node* node_ptr);

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
    controller_available_modes_in_ = available_modes;
    // sort modes in ascending order
    std::sort(controller_available_modes_in_.begin(), controller_available_modes_in_.end());
  };

  void setOutputControlModesAvailables(const std::vector<uint8_t>& available_modes) {
    controller_available_modes_out_ = available_modes;
    // sort modes in ascending order
    std::sort(controller_available_modes_out_.begin(), controller_available_modes_out_.end());
  };

  virtual ~ControllerBase(){};

  protected:
  as2::Node* node_ptr_;

  private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void ref_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void ref_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void ref_traj_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);
  void control_timer_callback();
  void setControlModeSrvCall(const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
                             as2_msgs::srv::SetControlMode::Response::SharedPtr response);

  void sendCommand();
  bool setPlatformControlMode(const as2_msgs::msg::ControlMode& mode);

  bool negotiateOutputMode();

};  //  ControllerBase

};  // namespace controller_plugin_base

#endif  // CONTROLLER_BASE_HPP
