#include "controller_plugin_base/controller_base.hpp"

namespace controller_plugin_base {
void ControllerBase::initialize(as2::Node* node_ptr) {
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

  pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::actuator_command::pose, as2_names::topics::actuator_command::qos);
  twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
  thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
      as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);

  using namespace std::chrono_literals;
  // FIXME: Hardcoded timer period
  control_timer_ =
      node_ptr_->create_wall_timer(10ms, std::bind(&ControllerBase::control_timer_callback, this));

  set_control_mode_srv_ = node_ptr->create_service<as2_msgs::srv::SetControlMode>(
      as2_names::services::controller::set_control_mode,
      std::bind(&ControllerBase::setControlModeSrvCall, this,
                std::placeholders::_1,  // Corresponds to the 'request'  input
                std::placeholders::_2   // Corresponds to the 'response' input
                ));

  input_mode_.control_mode = as2_msgs::msg::ControlMode::UNSET;
  output_mode_.control_mode = as2_msgs::msg::ControlMode::UNSET;

  ownInitialize();
};

void ControllerBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odometry_adquired_ = true;
  updateState(*msg);
};
void ControllerBase::ref_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  motion_reference_adquired_ = true;
  updateReference(*msg);
};
void ControllerBase::ref_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  motion_reference_adquired_ = true;
  updateReference(*msg);
};
void ControllerBase::ref_traj_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
  motion_reference_adquired_ = true;
  updateReference(*msg);
};

void ControllerBase::platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
  platform_info_ = *msg;
};

void ControllerBase::control_timer_callback() {
  if (!platform_info_.offboard || !platform_info_.armed) {
    return;
  }

  if (!control_mode_established_) {
    return;
  }

  if (!odometry_adquired_ || !motion_reference_adquired_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for odometry and motion reference");
    return;
  }

  sendCommand();
};

bool ControllerBase::setPlatformControlMode(const as2_msgs::msg::ControlMode& mode) {
  as2_msgs::srv::SetControlMode::Request set_control_mode_req;
  set_control_mode_req.control_mode = mode;
  auto request = set_control_mode_client_->sendRequest(set_control_mode_req);
  if (request.success) {
    output_mode_ = mode;
  }
  return request.success;
};

bool ControllerBase::negotiateOutputMode() {
  // check if the list of available modes is empty

  if (platform_available_modes_in_.empty()) {
    // if the list is empty, send a request to the platform to get the list of available modes
    as2_msgs::srv::ListControlModes::Request list_control_modes_req;
    as2_msgs::srv::ListControlModes::Response response =
        list_control_modes_client_->sendRequest(list_control_modes_req);
    if (response.control_modes.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "No available control modes");
      return false;
    }
    platform_available_modes_in_ = response.control_modes;
  }

  // compare platform_available_modes_in_ with controller_available_modes_out_
  // and chose the one in common with LOWER VALUE both are std::vector<uint8_t>
  // if no common mode is found, return false
  // if a common mode is found, set the output_mode_ and return true
  // if the common mode is the same as the input_mode_, return true

  uint8_t common_mode = 0;

  // check if the prefered mode is available
  if (prefered_output_mode_) {
    // search if the prefered mode is available
    for (auto mode : platform_available_modes_in_) {
      if (mode == prefered_output_mode_) {
        common_mode = mode;
        break;
      }
    }
  }

  // if the prefered mode is not available, search for the first common mode
  if (!common_mode) {
    for (auto& mode_out : controller_available_modes_out_) {
      // skip unset modes and hover
      if ((mode_out & 0b1111000) == 0b00000000 || (mode_out & 0b1111000) == 0b00001000) {
        continue;
      }
      for (auto& mode_in : platform_available_modes_in_) {
        if (mode_in == mode_out) {
          common_mode = mode_in;
          break;
        }
      }
    }
  }

  // check if the common mode exist
  if (common_mode == 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "No common control mode");
    return false;
  }

  // request the common mode to the platform
  auto mode_to_request = as2::convertUint8tToAS2ControlMode(common_mode);
  if (!setPlatformControlMode(mode_to_request)) {
    return false;
  }

  output_mode_ = mode_to_request;
  return true;
}

void ControllerBase::setControlModeSrvCall(
    const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
    as2_msgs::srv::SetControlMode::Response::SharedPtr response) {
  control_mode_established_ = false;

  // check if the output_mode is already settled
  if (output_mode_.control_mode == as2_msgs::msg::ControlMode::UNSET) {
    if (!negotiateOutputMode()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Platform control_mode negotiation failed");
      response->success = false;
      return;
    }
  }

  uint8_t output_conversion = as2::convertAS2ControlModeToUint8t(output_mode_);
  uint8_t input_conversion = as2::convertAS2ControlModeToUint8t(request->control_mode);

  // check if the input mode is compatible with the output mode
  if ((input_conversion & 0b1111000) < (output_conversion & 0b1111000)) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "Input control mode has lower level than output control mode");
    response->success = false;
    return;
  }

  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  for (auto& mode : controller_available_modes_in_) {
    if (mode == input_conversion) {
      mode_found = true;
      break;
    }
  }

  if (!mode_found) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Input control mode is not available");
    response->success = false;
    return;
  }

  input_mode_ = request->control_mode;

  // finally set the output mode
  response->success = setMode(input_mode_, output_mode_);
  if (!response->success) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Output control mode negotiation failed");
  }
  control_mode_established_ = response->success;

  RCLCPP_INFO(node_ptr_->get_logger(), "Control mode established:");
  RCLCPP_INFO(node_ptr_->get_logger(), "  Input:");
  as2::printControlMode(input_mode_);
  RCLCPP_INFO(node_ptr_->get_logger(), "  Output:");
  as2::printControlMode(output_mode_);
}

void ControllerBase::sendCommand() {
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::TwistStamped twist;
  as2_msgs::msg::Thrust thrust;
  computeOutput(pose, twist, thrust);

  // set time stamp
  pose.header.stamp = node_ptr_->now();
  twist.header.stamp = pose.header.stamp;
  thrust.header = pose.header;

  pose_pub_->publish(pose);
  twist_pub_->publish(twist);
  thrust_pub_->publish(thrust);
};

}  // namespace controller_plugin_base
