// cubic_spline.hpp
// Natural cubic spline through (xs, ys): C2, zero second derivative at the ends.
// eval(x) returns {value, first derivative, second derivative} w.r.t. x.
// Outside [xs.front(), xs.back()] it clamps to the end segment (no extrapolation
// blow-up); the trajectory generator keeps s within range anyway.

#ifndef TRAJECTORY_GENERATION_CUBIC_SPLINE_HPP
#define TRAJECTORY_GENERATION_CUBIC_SPLINE_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>

namespace trajectory_generation {

class CubicSpline {
 public:
  struct Eval { double y, dy, ddy; };

  CubicSpline() = default;
  CubicSpline(const std::vector<double>& xs, const std::vector<double>& ys) { build(xs, ys); }

  void build(const std::vector<double>& xs, const std::vector<double>& ys) {
    const int n = static_cast<int>(xs.size());
    if (n < 2 || static_cast<int>(ys.size()) != n)
      throw std::invalid_argument("CubicSpline: need >= 2 knots of equal length");
    x_ = xs;
    a_ = ys;
    b_.assign(n, 0.0); c_.assign(n, 0.0); d_.assign(n, 0.0);

    std::vector<double> h(n - 1);
    for (int i = 0; i + 1 < n; ++i) h[i] = xs[i + 1] - xs[i];

    // second derivatives M; natural boundary -> M[0] = M[n-1] = 0
    std::vector<double> M(n, 0.0);
    if (n >= 3) {
      std::vector<double> diag(n, 0.0), upper(n, 0.0), rhs(n, 0.0);
      for (int i = 1; i <= n - 2; ++i) {
        diag[i]  = 2.0 * (h[i - 1] + h[i]);
        upper[i] = h[i];                                   // coeff on M[i+1]; lower coeff is h[i-1]
        rhs[i]   = 6.0 * ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1]);
      }
      for (int i = 2; i <= n - 2; ++i) {                   // forward elimination (Thomas)
        const double w = h[i - 1] / diag[i - 1];
        diag[i] -= w * upper[i - 1];
        rhs[i]  -= w * rhs[i - 1];
      }
      M[n - 2] = rhs[n - 2] / diag[n - 2];                 // back substitution
      for (int i = n - 3; i >= 1; --i)
        M[i] = (rhs[i] - upper[i] * M[i + 1]) / diag[i];
    }

    // per-segment coefficients: y = a + b t + c t^2 + d t^3,  t = x - x_i
    for (int i = 0; i + 1 < n; ++i) {
      c_[i] = M[i] / 2.0;
      d_[i] = (M[i + 1] - M[i]) / (6.0 * h[i]);
      b_[i] = (ys[i + 1] - ys[i]) / h[i] - h[i] * (2.0 * M[i] + M[i + 1]) / 6.0;
    }
  }

  Eval eval(double x) const {
    const int n = static_cast<int>(x_.size());
    int i;
    if (x <= x_.front())      i = 0;
    else if (x >= x_.back())  i = n - 2;
    else {
      auto it = std::upper_bound(x_.begin(), x_.end(), x);
      i = static_cast<int>(it - x_.begin()) - 1;
    }
    if (i < 0) i = 0;
    if (i > n - 2) i = n - 2;
    const double t = x - x_[i];
    return { a_[i] + b_[i] * t + c_[i] * t * t + d_[i] * t * t * t,
             b_[i] + 2.0 * c_[i] * t + 3.0 * d_[i] * t * t,
             2.0 * c_[i] + 6.0 * d_[i] * t };
  }

 private:
  std::vector<double> x_, a_, b_, c_, d_;
};

}  // namespace trajectory_generation
#endif