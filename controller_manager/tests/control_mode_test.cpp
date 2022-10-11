#include <iostream>
#include <stdexcept>
#include "gtest/gtest.h"

#include "controller_manager/controller_handler.hpp"

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

uint8_t findMode(const std::vector<uint8_t> &controller_available_modes_out_,
                 const std::vector<uint8_t> &platform_available_modes_in_) {
  uint8_t common_mode = 0;
  for (auto &mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_ALL);
    if (common_mode) {
      break;
    }
  }
  return common_mode;
}

TEST(HandlerTest, check_mode) {
  std::vector<uint8_t> controller_list = {0, 16, 64, 65, 68, 69, 97, 101, 113, 117};
  std::vector<uint8_t> platform_list   = {68};
  uint8_t expected                     = 68;
  EXPECT_EQ(findMode(controller_list, platform_list), expected);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}