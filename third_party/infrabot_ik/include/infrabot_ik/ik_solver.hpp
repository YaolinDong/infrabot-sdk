/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <cstdint>
#include "infrabot_ik/kinematics.hpp"

namespace infrabot_ik {
// Existing 0/1/2 values retained. QP-specific failures add new values without
// changing the C ABI, Python fields, or solve()/step() signatures.
enum class IkStatus : int {
  Converged = 0, MaxIterations = 1, InvalidInput = 2,
  QpIterationLimit = 3, QpInfeasible = 4, QpNumericalFailure = 5,
  TimeBudgetExceeded = 6
};
const char* status_name(int status);

// Optional platform clock used by an embedded adapter. It must return a
// monotonically increasing microsecond counter. When it is null, the host
// implementation uses steady_clock; an embedded build without a clock keeps
// the deterministic iteration limit but cannot enforce wall-clock time.
using TimeSource = std::uint64_t (*)(void* context) noexcept;

struct IkOptions {
  int max_iterations{64};
  double tolerance{1e-4};
  double position_weight{1.0};
  double orientation_weight{0.35};
  double damping{1e-3};
  double step_size{0.8};
  double max_step_norm{0.12};
  double max_joint_step{0.08};
  // Soft redundancy-resolution bias toward the midpoint of every URDF joint
  // limit. It is projected into the pose-Jacobian nullspace, so a non-zero
  // default prevents 7-axis drift while keeping the Cartesian task dominant.
  // Set to zero to recover the legacy behavior.
  double posture_weight{0.02};
  double singularity_damping{1e-2}; // Reserved legacy field; not used.
  // A zero value disables the deadline. The default is one 500 us control
  // budget. The last verified joint vector is returned on timeout.
  double time_budget_us{500.0};
  TimeSource time_source{nullptr};
  void* time_source_context{nullptr};
};

// Limits applied to one periodic control update. This is intentionally a
// separate object so the legacy C ABI remains source/layout compatible. The
// constraints are intended for IKSolver::step(): q_out is limited relative to
// the input q for one period, while the optional previous velocity constrains
// the acceleration between updates.
struct MotionConstraints {
  bool enabled{false};
  double period_s{0.001};
  JointVector velocity_max_rad_s{};      // zero => use the URDF velocity limit
  JointVector acceleration_max_rad_s2{}; // zero => no acceleration limit
  JointVector previous_velocity_rad_s{};
  bool previous_velocity_valid{false};

  static MotionConstraints one_khz() {
    MotionConstraints out;
    out.enabled = true;
    for (std::size_t i = 0; i < out.velocity_max_rad_s.size(); ++i)
      out.velocity_max_rad_s[i] = kJointSpecs[i].velocity;
    return out;
  }
};
struct IkResult {
  JointVector q{};
  bool success{false};
  int iterations{0};
  double position_error{0.0};
  double orientation_error{0.0};
  double error{0.0};
  double solve_time_ms{0.0};
  int status{0};
};
class IKSolver {
 public:
  explicit IKSolver(const Kinematics& kinematics=Kinematics{}) : kin_(kinematics) {}
  IkResult solve(const Pose& target,const JointVector& seed,const IkOptions& options=IkOptions{}) const;
  IkResult solve(const Pose& target,const JointVector& seed,const IkOptions& options,
                 const MotionConstraints& motion) const;
  // One QP/IK iteration. Pass MotionConstraints::one_khz() for a bounded
  // periodic update suitable for a 1 kHz joint/cartesian control loop.
  IkResult step(const Pose& target,const JointVector& current,const IkOptions& options=IkOptions{}) const;
  IkResult step(const Pose& target,const JointVector& current,const IkOptions& options,
                const MotionConstraints& motion) const;
  const Kinematics& kinematics() const { return kin_; }
 private:
  IkResult iterate(const Pose& target,const JointVector& start,const IkOptions& options,
                   const MotionConstraints& motion,int iterations) const;
  Kinematics kin_;
};
} // namespace infrabot_ik
