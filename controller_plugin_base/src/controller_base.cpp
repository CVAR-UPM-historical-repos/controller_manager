#include "controller_plugin_base/controller_base.hpp"

#include <as2_core/control_mode_utils/control_mode_utils.hpp>
#include <rclcpp/logging.hpp>

namespace controller_plugin_base {

static inline bool checkMatchWithMask(const uint8_t mode1, const uint8_t mode2,
                                      const uint8_t mask) {
  return (mode1 & mask) == (mode2 & mask);
}

static uint8_t findBestMatchWithMask(const uint8_t mode, const std::vector<uint8_t>& mode_list,
                                     const uint8_t mask) {
  uint8_t best_match = 0;
  for (const auto& candidate : mode_list) {
    if (checkMatchWithMask(mode, candidate, mask)) {
      best_match = candidate;
      if (candidate == mode) {
        return candidate;
      }
    }
  }
  return best_match;
}

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

  list_control_modes_client_ =
      std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>>(
          as2_names::services::platform::list_control_modes);

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
}

void ControllerBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odometry_adquired_ = true;
  odom_ = *msg;
  if (!bypass_controller_) updateState(odom_);
}

void ControllerBase::ref_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  motion_reference_adquired_ = true;
  ref_pose_ = *msg;
  if (!bypass_controller_) updateReference(ref_pose_);
}

void ControllerBase::ref_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  motion_reference_adquired_ = true;
  ref_twist_ = *msg;
  if (!bypass_controller_) updateReference(ref_twist_);
}

void ControllerBase::ref_traj_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
  motion_reference_adquired_ = true;
  ref_traj_ = *msg;
  if (!bypass_controller_) updateReference(ref_traj_);
}

void ControllerBase::platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
  platform_info_ = *msg;
}

void ControllerBase::control_timer_callback() {
  if (!platform_info_.offboard || !platform_info_.armed) {
    return;
  }

  if (!control_mode_established_) {
    return;
  }

  if (!odometry_adquired_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for odometry ");
    return;
  }
  // if (!motion_reference_adquired_) {
  //   RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for motion reference");
  //   return;
  // }

  sendCommand();
};

// TODO: move to ControllerManager?
bool ControllerBase::setPlatformControlMode(const as2_msgs::msg::ControlMode& mode) {
  as2_msgs::srv::SetControlMode::Request set_control_mode_req;
  set_control_mode_req.control_mode = mode;
  auto request = set_control_mode_client_->sendRequest(set_control_mode_req);
  return request.success;
};

bool ControllerBase::listPlatformAvailableControlModes() {
  if (platform_available_modes_in_.empty()) {
    RCLCPP_INFO(node_ptr_->get_logger(), "LISTING AVAILABLE MODES");
    // if the list is empty, send a request to the platform to get the list of available modes
    as2_msgs::srv::ListControlModes::Request list_control_modes_req;
    as2_msgs::srv::ListControlModes::Response response =
        list_control_modes_client_->sendRequest(list_control_modes_req);
    if (response.control_modes.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "No available control modes");
      return false;
    }

    // log the available modes
    for (auto& mode : response.control_modes) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Available mode: %d", mode);
    }

    platform_available_modes_in_ = response.control_modes;
  }
  return true;
}

bool ControllerBase::tryToBypassController(const uint8_t input_mode, uint8_t& output_mode) {
  // check if platform available modes are set
  if ((input_mode & MATCH_MODE) == UNSET_MODE_MASK ||
      (input_mode & MATCH_MODE) == HOVER_MODE_MASK) {
    return false;
  }

  uint8_t candidate_mode =
      findBestMatchWithMask(input_mode, platform_available_modes_in_, MATCH_ALL);
  if (candidate_mode) {
    output_mode = candidate_mode;
    return true;
  }
  return false;
}

bool ControllerBase::checkSuitabilityInputMode(const uint8_t input_mode,
                                               const uint8_t output_mode) {
  // check if the input mode is compatible with the output mode
  if ((input_mode & MATCH_MODE) < (output_mode & 0b1111000)) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
                 "Input control mode has lower level than output control mode");
    return false;
  }

  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  for (auto& mode : controller_available_modes_in_) {
    if (mode == input_mode) {
      mode_found = true;
      break;
    }
  }

  return mode_found;
}

bool ControllerBase::findSuitableOutputControlModeForPlatformInputMode(uint8_t& output_mode,
                                                                       const uint8_t input_mode) {
  //  check if the prefered mode is available
  if (prefered_output_mode_) {
    auto match =
        findBestMatchWithMask(prefered_output_mode_, platform_available_modes_in_, MATCH_ALL);
    if (match) {
      output_mode = match;
      return true;
    }
  }

  // if the prefered mode is not available, search for the first common mode

  uint8_t common_mode = 0;
  bool same_yaw = false;

  for (auto& mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_ALL);
  }

  // check if the common mode exist
  if (common_mode == 0) {
    return false;
  }
  output_mode = common_mode;
  return true;
}

void ControllerBase::setControlModeSrvCall(
    const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
    as2_msgs::srv::SetControlMode::Response::SharedPtr response) {

  control_mode_established_ = false;
  // input_control_mode_desired
  uint8_t input_control_mode_desired = as2::convertAS2ControlModeToUint8t(request->control_mode);

  // check if platform_available_modes is set
  if (!listPlatformAvailableControlModes()) {
    response->success = false;
    return;
  }

  // 1st: check if a bypass is possible for the input_control_mode_desired ( DISCARDING YAW
  // COMPONENT)

  uint8_t output_control_mode_candidate = 0;

  bypass_controller_ =
      tryToBypassController(input_control_mode_desired, output_control_mode_candidate);

  if (!bypass_controller_) {
    bool success = findSuitableOutputControlModeForPlatformInputMode(output_control_mode_candidate,
                                                                     input_control_mode_desired);
    if (!success) {
      RCLCPP_WARN(node_ptr_->get_logger(), "No suitable output control mode found");
      response->success = false;
      return;
    }

    success = checkSuitabilityInputMode(input_control_mode_desired, output_control_mode_candidate);
    if (!success) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Input control mode is not suitable for this controller");
      response->success = false;
      return;
    }
  }

  // request the common mode to the platform
  auto mode_to_request = as2::convertUint8tToAS2ControlMode(output_control_mode_candidate);
  if (!setPlatformControlMode(mode_to_request)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to set platform control mode");
    response->success = false;
    return;
  }

  input_mode_ = request->control_mode;
  output_mode_ = mode_to_request;

  if (bypass_controller_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Bypassing controller with  input_mode:");
    as2::printControlMode(input_mode_);
    RCLCPP_INFO(node_ptr_->get_logger(), "and output_mode:");
    as2::printControlMode(output_mode_);
    auto unset_mode = as2::convertUint8tToAS2ControlMode(UNSET_MODE_MASK);
    response->success = setMode(unset_mode, unset_mode);
    control_mode_established_= response -> success;
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Setting controller with input_mode:");
  as2::printControlMode(input_mode_);
  RCLCPP_INFO(node_ptr_->get_logger(), "and output_mode:");
  as2::printControlMode(output_mode_);

  response->success = setMode(input_mode_, output_mode_);
  control_mode_established_= response -> success;

  if (!response->success) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to set control mode in the controller");
  }
  odometry_adquired_ = false;
  motion_reference_adquired_ = false;
  return;
}

void ControllerBase::sendCommand() {
  if (bypass_controller_) {
    if (!motion_reference_adquired_) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for motion reference");
      return;
    }
    pose_pub_->publish(ref_pose_);
    twist_pub_->publish(ref_twist_);
    return;
  }
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
