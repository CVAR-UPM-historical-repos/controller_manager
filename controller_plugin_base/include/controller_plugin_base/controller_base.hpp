/********************************************************************************************
 *  \file       controller_base.hpp
 *  \brief      Declares the controller_plugin_base class which is the base
 *class for all controller plugins.
 *
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef CONTROLLER_BASE_HPP
#define CONTROLLER_BASE_HPP

#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace controller_plugin_base {

class ControllerBase {
public:
  ControllerBase(){};

  void initialize(rclcpp::Node* node_ptr) {
    node_ptr_ = node_ptr;
    ownInitialize();
  };
  virtual void ownInitialize(){};
  virtual void updateState(const geometry_msgs::msg::PoseStamped& pose_msg,
                           const geometry_msgs::msg::TwistStamped& twist_msg) = 0;

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
  rclcpp::Node* node_ptr_;

};  //  ControllerBase

};  // namespace controller_plugin_base

#endif  // CONTROLLER_BASE_HPP
