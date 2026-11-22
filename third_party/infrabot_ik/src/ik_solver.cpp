/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot_ik/ik_solver.hpp"
#include "infrabot_ik/box_qp.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#if !defined(INFRABOT_IK_EMBEDDED)
#include <chrono>
#endif

namespace infrabot_ik {
namespace {
using QP = BoxQP<7>;
constexpr int kQpMaxIterations = 64;
struct Error { Vec3 p{}, o{}; double pn{0.0}, on{0.0}, n{0.0}; };

struct Budget {
  double limit_us{0.0};
  TimeSource source{nullptr};
  void* context{nullptr};
  std::uint64_t source_start{0};
#if !defined(INFRABOT_IK_EMBEDDED)
  std::chrono::steady_clock::time_point host_start{};
#endif

  explicit Budget(const IkOptions& options)
      : limit_us(options.time_budget_us), source(options.time_source),
        context(options.time_source_context) {
    if (source != nullptr) source_start = source(context);
#if !defined(INFRABOT_IK_EMBEDDED)
    host_start = std::chrono::steady_clock::now();
#endif
  }

  bool enabled() const { return limit_us > 0.0; }

  bool expired() const {
    if (!enabled()) return false;
    if (source != nullptr) {
      const std::uint64_t now = source(context);
      return now >= source_start &&
          static_cast<double>(now-source_start) >= limit_us;
    }
#if !defined(INFRABOT_IK_EMBEDDED)
    return std::chrono::duration<double,std::micro>(
               std::chrono::steady_clock::now()-host_start).count() >= limit_us;
#else
    return false;
#endif
  }
};

bool budget_deadline(void* context) noexcept {
  return static_cast<const Budget*>(context)->expired();
}

Error pose_error(const Pose& current, const Pose& target, const IkOptions& options) {
  Error e;
  e.p = target.position-current.position;
  e.o = rotation_log(target.rotation*current.rotation.transpose());
  e.pn = norm(e.p);
  e.on = norm(e.o);
  e.n = std::sqrt(options.position_weight*e.pn*e.pn +
                  options.orientation_weight*e.on*e.on);
  return e;
}

bool valid_input(const Pose& target, const JointVector& q, const IkOptions& o) {
  const double values[] = {o.tolerance, o.position_weight, o.orientation_weight,
      o.damping, o.step_size, o.max_step_norm, o.max_joint_step, o.posture_weight,
      o.time_budget_us};
  for (double value : values) if (!std::isfinite(value) || value < 0.0) return false;
  if (o.max_iterations < 1 || o.tolerance == 0.0 || o.step_size == 0.0 ||
      o.step_size > 1.0 || o.max_step_norm == 0.0 || o.max_joint_step == 0.0 ||
      (o.position_weight == 0.0 && o.orientation_weight == 0.0)) return false;
  for (std::size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(q[i]) || q[i] < kJointSpecs[i].lower || q[i] > kJointSpecs[i].upper) return false;
  }
  if (!std::isfinite(target.position.x) || !std::isfinite(target.position.y) ||
      !std::isfinite(target.position.z)) return false;
  const Mat3& r = target.rotation;
  for (const auto& row : r.m) for (double value : row) if (!std::isfinite(value)) return false;
  const Mat3 check = r.transpose()*r;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
    if (!std::isfinite(check.m[i][j]) || std::abs(check.m[i][j]-(i == j ? 1.0 : 0.0)) > 1e-6) return false;
  }
  const Vec3 x{r.m[0][0],r.m[1][0],r.m[2][0]};
  const Vec3 y{r.m[0][1],r.m[1][1],r.m[2][1]};
  const Vec3 z{r.m[0][2],r.m[1][2],r.m[2][2]};
  return std::abs(dot(x,cross(y,z))-1.0) <= 1e-6;
}

bool valid_motion(const MotionConstraints& motion) {
  if (!motion.enabled) return true;
  if (!std::isfinite(motion.period_s) || motion.period_s <= 0.0) return false;
  for (std::size_t i = 0; i < motion.velocity_max_rad_s.size(); ++i) {
    if (!std::isfinite(motion.velocity_max_rad_s[i]) ||
        !std::isfinite(motion.acceleration_max_rad_s2[i]) ||
        !std::isfinite(motion.previous_velocity_rad_s[i]) ||
        motion.velocity_max_rad_s[i] < 0.0 ||
        motion.acceleration_max_rad_s2[i] < 0.0) return false;
  }
  return true;
}

void set_error(IkResult& result, const Error& e) {
  result.position_error = e.pn;
  result.orientation_error = e.on;
  result.error = e.n;
}

// Find the one-dimensional task nullspace of a regular 7-axis pose Jacobian
// and project the neutral-posture error into it.  Keeping the posture term in
// this nullspace makes it a true secondary objective: it can use redundancy
// without trading away Cartesian accuracy.  At a rank-deficient pose there
// may be more than one nullspace direction; in that case we conservatively
// disable the bias for this iteration rather than disturb the primary task.
bool nullspace_posture(const Jacobian& jacobian, const IkOptions& options,
                       const JointVector& posture_error,
                       JointVector& null_vector,
                       JointVector& projected_error) {
  std::array<std::array<double,7>,6> matrix{};
  const double position_scale = std::sqrt(options.position_weight);
  const double orientation_scale = std::sqrt(options.orientation_weight);
  for (int row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 7; ++col)
      matrix[row][col] = position_scale*jacobian.linear[row][col];
    for (std::size_t col = 0; col < 7; ++col)
      matrix[row+3][col] = orientation_scale*jacobian.angular[row][col];
  }

  std::array<int,6> pivot_columns{};
  int rank = 0;
  constexpr double rank_epsilon = 1e-10;
  for (int col = 0; col < 7 && rank < 6; ++col) {
    int pivot = rank;
    double largest = std::abs(matrix[pivot][col]);
    for (int row = rank+1; row < 6; ++row) {
      const double value = std::abs(matrix[row][col]);
      if (value > largest) { largest = value; pivot = row; }
    }
    if (!std::isfinite(largest) || largest <= rank_epsilon) continue;
    if (pivot != rank) std::swap(matrix[pivot], matrix[rank]);
    const double divisor = matrix[rank][col];
    for (int c = col; c < 7; ++c) matrix[rank][c] /= divisor;
    for (int row = 0; row < 6; ++row) {
      if (row == rank) continue;
      const double factor = matrix[row][col];
      for (int c = col; c < 7; ++c) matrix[row][c] -= factor*matrix[rank][c];
    }
    pivot_columns[rank] = col;
    ++rank;
  }
  if (rank != 6) return false;
  int free_column = -1;
  for (int col = 0; col < 7; ++col) {
    bool pivot = false;
    for (int row = 0; row < rank; ++row) if (pivot_columns[row] == col) {
      pivot = true; break;
    }
    if (!pivot) { free_column = col; break; }
  }
  if (free_column < 0) return false;

  null_vector = JointVector{};
  null_vector[free_column] = 1.0;
  for (int row = rank-1; row >= 0; --row) {
    const int pivot = pivot_columns[row];
    double value = 0.0;
    for (int col = pivot+1; col < 7; ++col) value += matrix[row][col]*null_vector[col];
    null_vector[pivot] = -value;
  }
  double squared_norm = 0.0;
  for (double value : null_vector) squared_norm += value*value;
  if (!std::isfinite(squared_norm) || squared_norm <= rank_epsilon*rank_epsilon) return false;
  const double scale = 1.0/std::sqrt(squared_norm);
  double scalar = 0.0;
  for (std::size_t i = 0; i < 7; ++i) {
    null_vector[i] *= scale;
    scalar += null_vector[i]*posture_error[i];
  }
  for (std::size_t i = 0; i < 7; ++i) projected_error[i] = null_vector[i]*scalar;
  return std::isfinite(scalar);
}
} // namespace

const char* status_name(int status) {
  switch (static_cast<IkStatus>(status)) {
    case IkStatus::Converged: return "CONVERGED";
    case IkStatus::MaxIterations: return "MAX_ITERATIONS";
    case IkStatus::InvalidInput: return "INVALID_INPUT";
    case IkStatus::QpIterationLimit: return "QP_ITERATION_LIMIT";
    case IkStatus::QpInfeasible: return "QP_INFEASIBLE";
    case IkStatus::QpNumericalFailure: return "QP_NUMERICAL_FAILURE";
    case IkStatus::TimeBudgetExceeded: return "TIME_BUDGET_EXCEEDED";
  }
  return "UNKNOWN";
}

IkResult IKSolver::iterate(const Pose& target, const JointVector& start,
                          const IkOptions& o, const MotionConstraints& motion,
                          int iterations) const {
  IkResult result{};
  result.q = start;
  if (!valid_input(target,start,o) || !valid_motion(motion)) {
    result.status = static_cast<int>(IkStatus::InvalidInput);
    result.position_error = result.orientation_error = result.error =
        std::numeric_limits<double>::infinity();
    return result;
  }
  // A conservative box guarantees the Euclidean step norm without a nonlinear
  // constraint or post-solve rescaling, as in tron2-ik's trust box.
  const double trust = std::min(o.max_joint_step, o.max_step_norm/std::sqrt(7.0));
  JointVector cycle_lower{}, cycle_upper{};
  for (std::size_t i = 0; i < 7; ++i) {
    cycle_lower[i] = kJointSpecs[i].lower;
    cycle_upper[i] = kJointSpecs[i].upper;
    if (!motion.enabled) continue;

    // A zero velocity entry means "use the static URDF motor limit". This
    // makes MotionConstraints::one_khz() useful without duplicating limits.
    const double velocity = motion.velocity_max_rad_s[i] > 0.0
        ? motion.velocity_max_rad_s[i] : kJointSpecs[i].velocity;
    const double velocity_delta = velocity*motion.period_s;
    double lower_delta = -velocity_delta;
    double upper_delta = velocity_delta;
    if (motion.previous_velocity_valid &&
        motion.acceleration_max_rad_s2[i] > 0.0) {
      const double acceleration_delta =
          motion.acceleration_max_rad_s2[i]*motion.period_s;
      const double lower_velocity =
          motion.previous_velocity_rad_s[i]-acceleration_delta;
      const double upper_velocity =
          motion.previous_velocity_rad_s[i]+acceleration_delta;
      lower_delta = std::max(lower_delta, lower_velocity*motion.period_s);
      upper_delta = std::min(upper_delta, upper_velocity*motion.period_s);
    }
    cycle_lower[i] = std::max(cycle_lower[i], start[i]+lower_delta);
    cycle_upper[i] = std::min(cycle_upper[i], start[i]+upper_delta);
  }
  Budget budget(o);
  QP qp;
  for (int it = 0; it < iterations; ++it) {
    if (budget.expired()) {
      set_error(result,pose_error(kin_.forward(result.q),target,o));
      result.status = static_cast<int>(IkStatus::TimeBudgetExceeded);
      return result;
    }
    const Error e = pose_error(kin_.forward(result.q),target,o);
    set_error(result,e);
    result.iterations = it+1;
    if (!std::isfinite(e.n)) {
      result.status = static_cast<int>(IkStatus::QpNumericalFailure);
      return result;
    }
    if (e.n <= o.tolerance) {
      result.success = true;
      result.status = static_cast<int>(IkStatus::Converged);
      return result;
    }
    const Jacobian j = kin_.jacobian(result.q);
    QP::Matrix h{};
    QP::Vector gradient{}, lower{}, upper{};
    JointVector posture_error{};
    for (std::size_t i = 0; i < 7; ++i)
      posture_error[i] = 0.5*(kJointSpecs[i].lower+kJointSpecs[i].upper)-result.q[i];
    JointVector projected_posture{};
    JointVector posture_nullspace{};
    const bool use_posture = o.posture_weight > 0.0 &&
        nullspace_posture(j,o,posture_error,posture_nullspace,projected_posture);
    for (std::size_t i = 0; i < 7; ++i) {
      for (std::size_t k = 0; k < 7; ++k) {
        for (int row = 0; row < 3; ++row) {
          h[i][k] += o.position_weight*j.linear[row][i]*j.linear[row][k] +
                     o.orientation_weight*j.angular[row][i]*j.angular[row][k];
        }
        if (use_posture)
          h[i][k] += o.posture_weight*posture_nullspace[i]*posture_nullspace[k];
      }
      // Preserve the old damping parameter's direct-diagonal meaning.
      // A small positive floor is needed by the strictly convex QP backend.
      h[i][i] += std::max(1e-10,o.damping);
      double rhs = o.position_weight*(j.linear[0][i]*e.p.x+j.linear[1][i]*e.p.y+j.linear[2][i]*e.p.z) +
                   o.orientation_weight*(j.angular[0][i]*e.o.x+j.angular[1][i]*e.o.y+j.angular[2][i]*e.o.z);
      if (use_posture) rhs += o.posture_weight*projected_posture[i];
      // Gain belongs in the objective. Scaling/clipping the optimal step
      // afterwards would no longer solve this constrained QP.
      gradient[i] = -o.step_size*rhs;
      lower[i] = std::max({kJointSpecs[i].lower-result.q[i],
                           cycle_lower[i]-result.q[i],-trust});
      upper[i] = std::min({kJointSpecs[i].upper-result.q[i],
                           cycle_upper[i]-result.q[i],trust});
    }
    const auto solution = qp.solve(h,gradient,lower,upper,kQpMaxIterations,
                                   &budget_deadline,&budget);
    if (solution.status != QpStatus::Optimal) {
      if (solution.status == QpStatus::TimeBudgetExceeded) {
        set_error(result,pose_error(kin_.forward(result.q),target,o));
        result.status = static_cast<int>(IkStatus::TimeBudgetExceeded);
        return result;
      }
      result.status = static_cast<int>(
          solution.status == QpStatus::IterationLimit ? IkStatus::QpIterationLimit :
          solution.status == QpStatus::Infeasible ? IkStatus::QpInfeasible : IkStatus::QpNumericalFailure);
      return result; // Never apply an unverified intermediate QP iterate.
    }
    JointVector candidate = result.q;
    double squared_step = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      const double d = solution.x[i];
      if (!std::isfinite(d) || d < lower[i]-1e-12 || d > upper[i]+1e-12) {
        result.status = static_cast<int>(IkStatus::QpNumericalFailure);
        return result;
      }
      candidate[i] += d;
      squared_step += d*d;
      if (!std::isfinite(candidate[i]) || candidate[i] < kJointSpecs[i].lower-1e-12 ||
          candidate[i] > kJointSpecs[i].upper+1e-12) {
        result.status = static_cast<int>(IkStatus::QpNumericalFailure);
        return result;
      }
      // Only roundoff at an already optimized active bound, <=1e-12 rad.
      candidate[i] = std::clamp(candidate[i],kJointSpecs[i].lower,kJointSpecs[i].upper);
    }
    if (std::sqrt(squared_step) > o.max_step_norm+1e-12) {
      result.status = static_cast<int>(IkStatus::QpNumericalFailure);
      return result;
    }
    result.q = candidate;
    // Do not start another iteration after the deadline. Re-evaluate the
    // accepted candidate so callers receive its actual residual, rather than
    // the previous iterate's residual.
    if (budget.expired()) {
      const Error final_error = pose_error(kin_.forward(result.q),target,o);
      set_error(result,final_error);
      result.success = std::isfinite(result.error) && result.error <= o.tolerance;
      result.status = static_cast<int>(result.success ? IkStatus::Converged :
                                                     IkStatus::TimeBudgetExceeded);
      return result;
    }
  }
  set_error(result,pose_error(kin_.forward(result.q),target,o));
  result.success = std::isfinite(result.error) && result.error <= o.tolerance;
  result.status = static_cast<int>(result.success ? IkStatus::Converged : IkStatus::MaxIterations);
  return result;
}

IkResult IKSolver::solve(const Pose& target,const JointVector& seed,const IkOptions& o) const {
  return solve(target,seed,o,MotionConstraints{});
}
IkResult IKSolver::solve(const Pose& target,const JointVector& seed,const IkOptions& o,
                         const MotionConstraints& motion) const {
#if defined(INFRABOT_IK_EMBEDDED)
  return iterate(target,seed,o,motion,std::max(1,o.max_iterations));
#else
  const auto started = std::chrono::steady_clock::now();
  IkResult result = iterate(target,seed,o,motion,std::max(1,o.max_iterations));
  result.solve_time_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return result;
#endif
}
IkResult IKSolver::step(const Pose& target,const JointVector& current,const IkOptions& o) const {
  return step(target,current,o,MotionConstraints{});
}
IkResult IKSolver::step(const Pose& target,const JointVector& current,const IkOptions& o,
                        const MotionConstraints& motion) const {
#if defined(INFRABOT_IK_EMBEDDED)
  return iterate(target,current,o,motion,1);
#else
  const auto started = std::chrono::steady_clock::now();
  IkResult result = iterate(target,current,o,motion,1);
  result.solve_time_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return result;
#endif
}
} // namespace infrabot_ik
