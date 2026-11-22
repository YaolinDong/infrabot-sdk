/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace infrabot_ik {
enum class QpStatus { Optimal, InvalidInput, Infeasible, IterationLimit,
                      NumericalFailure, TimeBudgetExceeded };

// Primal feasible active-set solver: min 0.5*x'Hx+g'x, lower<=x<=upper.
// Free coordinates are reoptimized when bounds activate. No heap/dependencies.
template <std::size_t N> class BoxQP {
 public:
  using Vector = std::array<double, N>;
  using Matrix = std::array<Vector, N>;
  struct Result {
    Vector x{};
    QpStatus status{QpStatus::InvalidInput};
    int iterations{0};
    int active_constraints{0};
  };
  using Deadline = bool (*)(void* context) noexcept;
  Result solve(const Matrix& h, const Vector& g, const Vector& lower,
               const Vector& upper, int max_iterations = 64,
               Deadline deadline = nullptr, void* deadline_context = nullptr) const {
    Result result;
    if (max_iterations < 1) return result;
    for (std::size_t i = 0; i < N; ++i) {
      if (!std::isfinite(g[i]) || !std::isfinite(lower[i]) || !std::isfinite(upper[i])) return result;
      for (std::size_t j = 0; j < N; ++j) {
        if (!std::isfinite(h[i][j]) ||
            std::abs(h[i][j]-h[j][i]) > 1e-12*(1.0+std::abs(h[i][j]))) return result;
      }
      if (lower[i] > upper[i]) { result.status = QpStatus::Infeasible; return result; }
    }
    std::array<std::size_t, N> indices{};
    for (std::size_t i = 0; i < N; ++i) indices[i] = i;
    Matrix factor{};
    if (!cholesky(h, indices, N, factor)) {
      result.status = QpStatus::NumericalFailure; return result;
    }
    // -1 lower, +1 upper, 2 fixed equality, 0 free.
    std::array<int, N> active{};
    for (std::size_t i = 0; i < N; ++i) {
      result.x[i] = std::clamp(0.0, lower[i], upper[i]);
      if (lower[i] == upper[i]) active[i] = 2;
      else if (result.x[i] == lower[i]) active[i] = -1;
      else if (result.x[i] == upper[i]) active[i] = 1;
    }
    for (int iteration = 1; iteration <= max_iterations; ++iteration) {
      if (deadline != nullptr && deadline(deadline_context)) {
        result.status = QpStatus::TimeBudgetExceeded;
        return result;
      }
      result.iterations = iteration;
      Vector gradient = g;
      double tolerance = 1e-11;
      for (std::size_t i = 0; i < N; ++i) {
        double scale = 1.0+std::abs(g[i]);
        for (std::size_t j = 0; j < N; ++j) {
          gradient[i] += h[i][j]*result.x[j];
          scale += std::abs(h[i][j]*result.x[j]);
        }
        if (!std::isfinite(gradient[i]) || !std::isfinite(scale)) {
          result.status = QpStatus::NumericalFailure; return result;
        }
        tolerance = std::max(tolerance, 1e-11*scale);
      }
      if (deadline != nullptr && deadline(deadline_context)) {
        result.status = QpStatus::TimeBudgetExceeded;
        return result;
      }
      std::size_t count = 0;
      double free_residual = 0.0;
      for (std::size_t i = 0; i < N; ++i) {
        if (active[i] == 0) {
          indices[count++] = i;
          free_residual = std::max(free_residual, std::abs(gradient[i]));
        }
      }
      if (free_residual <= tolerance) {
        std::size_t release = N;
        double worst = tolerance;
        for (std::size_t i = 0; i < N; ++i) {
          const double violation = active[i] == -1 ? -gradient[i] : active[i] == 1 ? gradient[i] : 0.0;
          if (violation > worst) { worst = violation; release = i; }
        }
        if (release == N) {
          result.status = QpStatus::Optimal;
          for (int bound : active) if (bound != 0) ++result.active_constraints;
          return result; // Primal feasibility + dual signs + free stationarity.
        }
        active[release] = 0;
        continue;
      }
      if (!cholesky(h, indices, count, factor)) {
        result.status = QpStatus::NumericalFailure; return result;
      }
      Vector y{}, p{}, free_step{};
      for (std::size_t i = 0; i < count; ++i) {
        double value = -gradient[indices[i]];
        for (std::size_t j = 0; j < i; ++j) value -= factor[i][j]*y[j];
        y[i] = value/factor[i][i];
      }
      for (std::size_t ii = count; ii > 0; --ii) {
        const std::size_t i = ii-1;
        double value = y[i];
        for (std::size_t j = i+1; j < count; ++j) value -= factor[j][i]*free_step[j];
        free_step[i] = value/factor[i][i];
        p[indices[i]] = free_step[i];
      }
      double alpha = 1.0;
      std::size_t blocking = N;
      int bound = 0;
      for (std::size_t i = 0; i < N; ++i) {
        if (!std::isfinite(p[i])) { result.status = QpStatus::NumericalFailure; return result; }
        if (active[i] != 0 || p[i] == 0.0) continue;
        const double limit = p[i] > 0.0 ? (upper[i]-result.x[i])/p[i] : (lower[i]-result.x[i])/p[i];
        if (limit <= alpha) { alpha = std::max(0.0, limit); blocking = i; bound = p[i] > 0.0 ? 1 : -1; }
      }
      for (std::size_t i = 0; i < N; ++i) {
        // Roundoff cleanup on a feasible line-search step, NOT clipping an
        // unconstrained minimizer: the active coordinates are reoptimized.
        result.x[i] = std::clamp(result.x[i]+alpha*p[i], lower[i], upper[i]);
      }
      if (blocking != N) {
        result.x[blocking] = bound == 1 ? upper[blocking] : lower[blocking];
        active[blocking] = bound;
      }
    }
    result.status = QpStatus::IterationLimit;
    return result; // Caller must not apply a non-optimal intermediate iterate.
  }
 private:
  static bool cholesky(const Matrix& h, const std::array<std::size_t, N>& indices,
                       std::size_t count, Matrix& l) {
    l = {};
    for (std::size_t i = 0; i < count; ++i) {
      for (std::size_t j = 0; j <= i; ++j) {
        double value = h[indices[i]][indices[j]];
        for (std::size_t k = 0; k < j; ++k) value -= l[i][k]*l[j][k];
        if (!std::isfinite(value)) return false;
        if (i == j) {
          if (value <= 1e-15) return false;
          l[i][j] = std::sqrt(value);
        } else l[i][j] = value/l[j][j];
      }
    }
    return true;
  }
};
} // namespace infrabot_ik
