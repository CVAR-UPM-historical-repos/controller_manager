#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace controller_plugin_base{

class ControllerBase {
  public:
  ControllerBase(){};

  virtual void initialize(as2::Node* node_ptr) { node_ptr_ = node_ptr; };

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

  virtual ~ControllerBase(){};

  protected:

  as2::Node* node_ptr_;

};  // ControllerBase class

}  // namespace controller_plugin_base

#endif  // CONTROLLER_BASE_HPP
