#include <iostream>
#include <stdexcept>
#include "gtest/gtest.h"

#include "as2_core/utils/control_mode_utils.hpp"
#include "controller_manager/controller_handler.hpp"

std::vector<uint8_t> controller_available_modes_in_ = {
    0b00000000, 0b00010000, 0b01000000, 0b01000001, 0b01000100,
    0b01000101, 0b01100001, 0b01100101, 0b01110001, 0b01110101};

std::vector<uint8_t> controller_available_modes_out_ = {0b00000000, 0b01000100, 0b01000101};

std::vector<uint8_t> platform_available_modes_in_ = {0b01000100};

uint8_t prefered_output_mode_ = 0b00000000;  // by default, no output mode is prefered

bool checkMatchWithMask(const uint8_t mode1, const uint8_t mode2, const uint8_t mask) {
  return (mode1 & mask) == (mode2 & mask);
}

uint8_t findBestMatchWithMask(const uint8_t mode,
                              const std::vector<uint8_t> &mode_list,
                              const uint8_t mask) {
  uint8_t best_match = 0;
  for (const auto &candidate : mode_list) {
    if (checkMatchWithMask(mode, candidate, mask)) {
      best_match = candidate;
      if (candidate == mode) {
        return candidate;
      }
    }
  }
  return best_match;
}

bool tryToBypassController(const uint8_t input_mode, uint8_t &output_mode) {
  // check if platform available modes are set
  if ((input_mode & MATCH_MODE) == UNSET_MODE_MASK ||
      (input_mode & MATCH_MODE) == HOVER_MODE_MASK) {
    return false;
  }

  uint8_t candidate_mode =
      findBestMatchWithMask(input_mode, platform_available_modes_in_, MATCH_MODE_AND_YAW);
  if (candidate_mode) {
    output_mode = candidate_mode;
    return true;
  }
  return false;
}

bool checkSuitabilityInputMode(const uint8_t input_mode, const uint8_t output_mode) {
  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  for (auto &mode : controller_available_modes_in_) {
    if ((input_mode & MATCH_MODE) == HOVER_MODE_MASK && (input_mode & MATCH_MODE) == mode) {
      mode_found = true;
      return true;
    } else if (mode == input_mode) {
      mode_found = true;
      break;
    }
  }

  // check if the input mode is compatible with the output mode
  if ((input_mode & MATCH_MODE) < (output_mode & 0b1111000)) {
    std::cout << "Input control mode has lower level than output control mode" << std::endl;
    return false;
  }

  return mode_found;
}

bool findSuitableOutputControlModeForPlatformInputMode(uint8_t &output_mode) {
  //  check if the prefered mode is available
  if (prefered_output_mode_) {
    auto match = findBestMatchWithMask(prefered_output_mode_, platform_available_modes_in_,
                                       MATCH_MODE_AND_YAW);
    if (match) {
      output_mode = match;
      return true;
    }
  }

  // if the prefered mode is not available, search for the first common mode

  uint8_t common_mode = 0;
  bool same_yaw       = false;

  for (auto &mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_MODE_AND_YAW);
    if (common_mode) {
      break;
    }
  }

  // check if the common mode exist
  if (common_mode == 0) {
    return false;
  }
  output_mode = common_mode;
  return true;
}

std::string enu_frame_id_ = "odom";
std::string flu_frame_id_ = "base_link";

std::string getFrameId(const uint8_t reference_frame) {
  switch (reference_frame) {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      return enu_frame_id_;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      return flu_frame_id_;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
    default:
      return "undefined";
  }
}

std::pair<as2_msgs::msg::ControlMode, as2_msgs::msg::ControlMode> setControlModeSrvCall(
    const as2_msgs::msg::ControlMode &control_mode_request,
    const bool &use_bypass_) {
  bool bypass_controller_ = false;

  std::string state_pose_frame_id_  = "odom";
  std::string state_twist_frame_id_ = "odom";

  std::string reference_pose_frame_id_  = "odom";
  std::string reference_twist_frame_id_ = "odom";

  std::string output_pose_frame_id_  = "odom";
  std::string output_twist_frame_id_ = "odom";


  as2_msgs::msg::ControlMode control_mode_in;
  as2_msgs::msg::ControlMode control_mode_out;
  uint8_t controller_control_mode = 0;

  uint8_t input_control_mode_desired =
      as2::control_mode::convertAS2ControlModeToUint8t(control_mode_request);


  // Check if a bypass is possible for the input_control_mode_desired ( DISCARDING REFERENCE
  // COMPONENT)
  uint8_t platform_control_mode = 0;
  if (use_bypass_) {
    bypass_controller_ = tryToBypassController(input_control_mode_desired, platform_control_mode);
  } else {
    bypass_controller_ = false;
  }

  // Now, platform_control_mode stored the desired control mode for the platform



  // 1. If the input mode is HOVER, set:
  //   - Control mode in: to Hover, with yaw mode None and reference frame Undefined.
  //   - Control mode out: selected automatically.
  // 
  //   -> reference_pose_frame_id_  = No transformation
  //   -> reference_twist_frame_id_ = No transformation

  if (control_mode_request.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    input_control_mode_desired = HOVER_MODE_MASK;

    // Get the platform control mode for Hover -> lowest available mode
    // platform_control_mode = ...

    // Get the suitable control mode in for platform_control_mode
    // controller_control_mode = ...

    // Get control mode msg
    control_mode_in = as2::control_mode::convertUint8tToAS2ControlMode(controller_control_mode);
    control_mode_out = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);

    // Set the reference frame
    reference_pose_frame_id_ = ""; // No transformation
    reference_twist_frame_id_ = ""; // No transformation
  
  // 2. If the input mode is NOT HOVER, BYPASS and reference frame, set:
  //  - Control mode out: selected to match the input mode, with yaw mode request.
  //  - Control mode in: to the request one, with yaw mode request.
  //    - Reference frame: selected to match the output mode.
  //
  //  -> reference_pose_frame_id_  = output_pose_frame_id_
  //  -> reference_twist_frame_id_ = output_twist_frame_id_
  } else if (bypass_controller_) {
    // Set the platform control mode for request mode in (without reference frame)
    // platform_control_mode = ...

    // Set input control mode to the request one. Set control_mode_in.reference_frame to the
    // platform one.
    // controller_control_mode = ...

    // Get control mode msg
    control_mode_in = as2::control_mode::convertUint8tToAS2ControlMode(controller_control_mode);
    control_mode_out = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);

    // Set the reference frame
    reference_pose_frame_id_  = getFrameId(control_mode_in.reference_frame);
    reference_twist_frame_id_ = getFrameId(control_mode_in.reference_frame);

  // 3. If the input mode is not Hover and try bypass is false, set:
  //  - Control mode out: selected to match the input mode, with yaw mode request.
  //  - Control mode in: to the request one, with yaw mode request.
  //    - Reference frame: selected by plugin.
  // 
  //  -> reference_pose_frame_id_  = control mode in reference frame -> ENU_frame_id / FLU_frame_id
  //  -> reference_twist_frame_id_ = control mode in reference frame -> ENU_frame_id / FLU_frame_id
  } else {
    // Set the platform control mode for suitable mode in (without reference frame)
    // platform_control_mode = ...

    // Set input control mode to the request one. Set control_mode_in.reference_frame to the
    // one selected by plugin.
    // controller_control_mode = ...

    // Get control mode msg
    control_mode_in = as2::control_mode::convertUint8tToAS2ControlMode(controller_control_mode);
    control_mode_out = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);

    // Set the reference frame
    reference_pose_frame_id_  = getFrameId(control_mode_in.reference_frame);
    reference_twist_frame_id_ = getFrameId(control_mode_in.reference_frame);
  }



  // If the input mode is Hover, set desired control mode in to Hover,
  // else, set desired control mode in to the request one
  uint8_t input_control_mode_desired = 0;
  if (control_mode_request.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    input_control_mode_desired = HOVER_MODE_MASK;
  } else {
    input_control_mode_desired =
        as2::control_mode::convertAS2ControlModeToUint8t(control_mode_request);
  }

  // input_control_mode_desired is Hover or Control Mode Request

  // 1st: check if a bypass is possible for the input_control_mode_desired ( DISCARDING REFERENCE
  // COMPONENT)
  uint8_t platform_control_mode = 0;
  as2_msgs::msg::ControlMode control_mode_in;
  as2_msgs::msg::ControlMode control_mode_out;

  if (use_bypass_) {
    bypass_controller_ = tryToBypassController(input_control_mode_desired, platform_control_mode);
  } else {
    bypass_controller_ = false;
  }

  if (bypass_controller_) {
    // Controller control mode in = Platform control mode = Request control mode
    control_mode_in  = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);
    control_mode_out = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);
  } else {
    bool success = findSuitableOutputControlModeForPlatformInputMode(platform_control_mode);
    if (!success) {
      std::cout << "Error: No suitable output control mode found for input control mode"
                << std::endl;
    }

    success = checkSuitabilityInputMode(input_control_mode_desired, platform_control_mode);
    if (!success) {
      std::cout << "Error: Input control mode is not suitable for this controller" << std::endl;
    }
  }

  // SET MODE

  // request the common mode to the platform
  auto mode_to_request = as2::control_mode::convertUint8tToAS2ControlMode(platform_control_mode);

  output_pose_frame_id_  = getFrameId(control_mode_out.reference_frame);
  output_twist_frame_id_ = getFrameId(control_mode_out.reference_frame);

  reference_pose_frame_id_  = getFrameId(control_mode_in.reference_frame);
  reference_twist_frame_id_ = getFrameId(control_mode_in.reference_frame);

  return std::make_pair(control_mode_request, mode_to_request);
}

/**
 * @brief Expected behavior:
 *
 *  -> state_pose_frame_id_ = "odom"
 *  -> state_twist_frame_id_ = "odom"
 *
 *  -> output_pose_frame_id_  = control mode out reference frame -> ENU_frame_id / FLU_frame_id
 *  -> output_twist_frame_id_ = control mode out reference frame -> ENU_frame_id / FLU_frame_id
 *
 * 1. If the input mode is HOVER, set:
 *  - Control mode in: to Hover, with yaw mode None and reference frame Undefined.
 *  - Control mode out: selected automatically.
 *
 *  -> reference_pose_frame_id_  = No transformation
 *  -> reference_twist_frame_id_ = No transformation
 *
 * 2. If the input mode is NOT HOVER, BYPASS and reference frame, set:
 *  - Control mode out: selected to match the input mode, with yaw mode request.
 *  - Control mode in: to the request one, with yaw mode request.
 *    - Reference frame: selected to match the output mode.
 *
 *  -> reference_pose_frame_id_  = output_pose_frame_id_
 *  -> reference_twist_frame_id_ = output_twist_frame_id_
 *
 * 3. If the input mode is not Hover and try bypass is false, set:
 *  - Control mode out: selected to match the input mode, with yaw mode request.
 *  - Control mode in: to the request one, with yaw mode request.
 *    - Reference frame: selected by plugin.
 *
 *  -> reference_pose_frame_id_  = control mode in reference frame -> ENU_frame_id / FLU_frame_id
 *  -> reference_twist_frame_id_ = control mode in reference frame -> ENU_frame_id / FLU_frame_id
 */
void test() {
  std::cout << std::endl;

  as2_msgs::msg::ControlMode control_mode_request;
  control_mode_request.control_mode    = as2_msgs::msg::ControlMode::POSITION;
  control_mode_request.yaw_mode        = as2_msgs::msg::ControlMode::NONE;
  control_mode_request.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;

  auto result = setControlModeSrvCall(control_mode_request, true);

  std::cout << "Control mode request: " << std::endl;
  std::cout << "Control mode: " << std::to_string(control_mode_request.control_mode) << std::endl;
  std::cout << "Yaw mode: " << std::to_string(control_mode_request.yaw_mode) << std::endl;
  std::cout << "Reference frame: " << std::to_string(control_mode_request.reference_frame)
            << std::endl;
  std::cout << std::endl;

  std::cout << "Control mode input get: " << std::endl;
  std::cout << "Control mode: " << std::to_string(result.first.control_mode) << std::endl;
  std::cout << "Yaw mode: " << std::to_string(result.first.yaw_mode) << std::endl;
  std::cout << "Reference frame: " << std::to_string(result.first.reference_frame) << std::endl;
  std::cout << std::endl;

  std::cout << "Control mode output get: " << std::endl;
  std::cout << "Control mode: " << std::to_string(result.second.control_mode) << std::endl;
  std::cout << "Yaw mode: " << std::to_string(result.second.yaw_mode) << std::endl;
  std::cout << "Reference frame: " << std::to_string(result.second.reference_frame) << std::endl;
  std::cout << std::endl;
  return;
}

// uint8_t findMode(const std::vector<uint8_t> &controller_available_modes_out_,
//                  const std::vector<uint8_t> &platform_available_modes_in_) {
//   uint8_t common_mode = 0;
//   for (auto &mode_out : controller_available_modes_out_) {
//     // skip unset modes and hover
//     if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK)
//     {
//       continue;
//     }
//     common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_ALL);
//     if (common_mode) {
//       break;
//     }
//   }
//   return common_mode;
// }

// TEST(HandlerTest, check_mode) {
//   // std::vector<uint8_t> controller_list = {0, 16, 64, 65, 68, 69, 97, 101, 113, 117};
//   // std::vector<uint8_t> platform_list   = {68};
//   // uint8_t expected                     = 68;
//   as2_msgs::msg::ControlMode control_mode_request;
//   control_mode_request.control_mode    = as2_msgs::msg::ControlMode::HOVER;
//   control_mode_request.yaw_mode        = as2_msgs::msg::ControlMode::NONE;
//   control_mode_request.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;

//   as2_msgs::msg::ControlMode control_mode_expected;
//   control_mode_request.control_mode    = as2_msgs::msg::ControlMode::HOVER;
//   control_mode_request.yaw_mode        = as2_msgs::msg::ControlMode::NONE;
//   control_mode_request.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;

//   EXPECT_EQ(findMode(controller_list, platform_list), expected);
// }

int main(int argc, char *argv[]) {
  test();
  return 0;
  // rclcpp::init(argc, argv);
  // ::testing::InitGoogleTest(&argc, argv);
  // return RUN_ALL_TESTS();
}