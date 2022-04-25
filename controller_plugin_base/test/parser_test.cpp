#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "as2_msgs/msg/control_mode.hpp"

const std::string config_path =
    "/home/miguel/aerostack2_ws/src/aerostack2/stack_devel/controller_manager/"
    "controller_plugin_base/available_modes.yaml";

std::vector<uint8_t> parse_available_modes(const std::string& filename) {
  std::ifstream config_file(filename);
  YAML::Node config_node = YAML::Load(config_file);
  std::vector<uint8_t> available_modes_vec;

  auto modes = config_node["available_modes"];
  for (const auto& mode : modes) {
    // check if the mode is a string that begins with 0b and contents 10 digits
    // remove the 0b prefix
    std::string binary_string = mode.as<std::string>().substr(2);
    // convert string from binary notation to uint8_t
    available_modes_vec.emplace_back(stoi(binary_string, 0, 2));
  }
  return available_modes_vec;
}

uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode& mode) {
  // # ------------- mode codification (4 bits) ----------------------
  // #
  // # unset             = 0 = 0b00000000
  // # hover             = 1 = 0b00010000
  // # acro              = 2 = 0b00100000
  // # attitude          = 3 = 0b00110000
  // # speed             = 4 = 0b01000000
  // # speed_in_a_plane  = 5 = 0b01010000
  // # position          = 6 = 0b01100000
  // # trajectory        = 7 = 0b01110000
  // #
  // #-------------- yaw codification --------------------------------
  // #
  // # angle             = 0 = 0b00000000
  // # speed             = 1 = 0b00000100
  // #
  // # frame codification
  // #
  // # local_frame_flu   = 0 = 0b00000000
  // # global_frame_enu  = 1 = 0b00000001
  // # global_frame_lla  = 2 = 0b00000010
  // #
  // #-----------------------------------------------------------------

  uint8_t control_mode_uint8t = 0;
  switch (mode.control_mode) {
    case as2_msgs::msg::ControlMode::ACRO:
      control_mode_uint8t = 0b00100000;
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      control_mode_uint8t = 0b00110000;
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      control_mode_uint8t = 0b01000000;
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      control_mode_uint8t = 0b01010000;
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      control_mode_uint8t = 0b01100000;
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      control_mode_uint8t = 0b01110000;
      break;
    case as2_msgs::msg::ControlMode::UNSET:
      control_mode_uint8t = 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::HOVER:
      control_mode_uint8t = 0b00010000;
      break;
    default:
      std::cout << "Control mode not recognized" << std::endl;
      break;
  }

  switch (mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      control_mode_uint8t |= 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      control_mode_uint8t |= 0b00000100;
      break;
    default:
      std::cout << "Yaw mode not recognized" << std::endl;
      break;
  }

  switch (mode.reference_frame) {
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      control_mode_uint8t |= 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      control_mode_uint8t |= 0b00000001;
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      control_mode_uint8t |= 0b00000010;
      break;
    default:
      std::cout << "Reference frame not recognized" << std::endl;
      break;
  }
  return control_mode_uint8t;
}

as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t) {
  as2_msgs::msg::ControlMode mode;
  // # ------------- mode codification (4 bits) ----------------------
  // #
  // # unset             = 0 = 0b00000000
  // # hover             = 1 = 0b00010000
  // # acro              = 2 = 0b00100000
  // # attitude          = 3 = 0b00110000
  // # speed             = 4 = 0b01000000
  // # speed_in_a_plane  = 5 = 0b01010000
  // # position          = 6 = 0b01100000
  // # trajectory        = 7 = 0b01110000
  // #
  // #-------------- yaw codification --------------------------------
  // #
  // # angle             = 0 = 0b00000000
  // # speed             = 1 = 0b00000100
  // #
  // # frame codification
  // #
  // # local_frame_flu   = 0 = 0b00000000
  // # global_frame_enu  = 1 = 0b00000001
  // # global_frame_lla  = 2 = 0b00000010
  // #
  // #-----------------------------------------------------------------

  if ((control_mode_uint8t & 0b11110000) == 0b00000000) {
    mode.control_mode = as2_msgs::msg::ControlMode::UNSET;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00010000) {
    mode.control_mode = as2_msgs::msg::ControlMode::HOVER;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00100000) {
    mode.control_mode = as2_msgs::msg::ControlMode::ACRO;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00110000) {
    mode.control_mode = as2_msgs::msg::ControlMode::ATTITUDE;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01000000) {
    mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01010000) {
    mode.control_mode = as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01100000) {
    mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01110000) {
    mode.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  } else {
    std::cout << "Control mode not recognized" << std::endl;
  }

  if ((control_mode_uint8t & 0b00001100) == 0b00000100) {
    mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  } else if ((control_mode_uint8t & 0b00000110) == 0b00000000) {
    mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  } else {
    std::cout << "Yaw mode not recognized" << std::endl;
  }

  if ((control_mode_uint8t & 0b00000011) == 0b00000001) {
    mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  } else if ((control_mode_uint8t & 0b00000011) == 0b00000010) {
    mode.reference_frame = as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML;
  } else if ((control_mode_uint8t & 0b00000011) == 0b00000000) {
    mode.reference_frame = as2_msgs::msg::ControlMode::BODY_FLU_FRAME;
  } else {
    std::cout << "Reference frame not recognized" << std::endl;
  }

  return mode;
}

void printControlMode(as2_msgs::msg::ControlMode mode) {
  std::cout << "Control mode: ";
  switch (mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET: {
      std::cout << "UNSET" << std::endl;
      return;
    } break;
    case as2_msgs::msg::ControlMode::HOVER: {
      std::cout << "HOVER" << std::endl;
      return;
    } break;
    case as2_msgs::msg::ControlMode::ACRO:
      std::cout << "ACRO" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      std::cout << "ATTITUDE" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      std::cout << "SPEED" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      std::cout << "SPEED_IN_A_PLANE" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      std::cout << "POSITION" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      std::cout << "TRAJECTORY" << std::endl;
      break;
    default:
      std::cout << "Control mode not recognized" << std::endl;
      break;
  }

  std::cout << "\t\tYaw mode: ";
  switch (mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      std::cout << "YAW_SPEED" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      std::cout << "YAW_ANGLE" << std::endl;
      break;
    default:
      std::cout << "Yaw mode not recognized" << std::endl;
      break;
  }

  std::cout << "\t\tReference frame: ";
  switch (mode.reference_frame) {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      std::cout << "LOCAL_ENU_FRAME" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      std::cout << "GLOBAL_LAT_LONG_ASML" << std::endl;
      break;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      std::cout << "BODY_FLU_FRAME" << std::endl;
      break;
    default:
      std::cout << "Reference frame not recognized" << std::endl;
      break;
  }
}

int main(int argc, char** argv) {
  // print the config file
  std::cout << "config file: " << config_path << std::endl;
  // parse the config file
  auto out = parse_available_modes(config_path);
  std::cout << "available modes: " << std::endl;
  for (auto mode : out) {
    std::cout << std::bitset<8>(mode) << std::endl;
    as2_msgs::msg::ControlMode mode_msg = convertUint8tToAS2ControlMode(mode);
    //Test the inverse operation 
    uint8_t mode_msg_uint8t = convertAS2ControlModeToUint8t(mode_msg);
    if (mode_msg_uint8t != mode) {
      std::cout << "Error: conversion from AS2 to uint8_t and back does not work" << std::endl;
      return -1;
    }
    printControlMode(mode_msg);
  }
  return 0;
}

