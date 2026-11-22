/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot_ik/generated_model.hpp"
namespace infrabot_ik {
class Kinematics {
 public:
  Pose forward(const JointVector& q) const;
  Jacobian jacobian(const JointVector& q) const;
  const JointSpec& joint(std::size_t i) const { return kJointSpecs[i]; }
  bool within_limits(const JointVector& q,double margin=0.0) const;
  JointVector clamp(const JointVector& q,double margin=0.0) const;
};
}  // namespace infrabot_ik
