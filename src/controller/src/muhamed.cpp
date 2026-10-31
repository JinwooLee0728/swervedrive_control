// muhamed.cpp : NMPC baseline implementation (Muhammed et al. 2026, n robots).

#include "controller/muhamed.hpp"

#include <algorithm>
#include <cmath>

using Eigen::Matrix2d;
using Eigen::Vector2d;

namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Matrix2d R2(double a) {
  Matrix2d m; m << std::cos(a), -std::sin(a), std::sin(a), std::cos(a); return m;
}
}  // namespace

namespace controller {

Muhamed::Muhamed(rclcpp::Node* node, const ControllerConfig& cfg) : cfg_(cfg) {
  N_        = node->declare_parameter<int>("muhamed.N", 10);
  dt_       = node->declare_parameter<double>("muhamed.dt", 0.01);   // should equal 1/rate_hz
  max_iters_= node->declare_parameter<int>("muhamed.max_iters", 10);
  Wpos_     = node->declare_parameter<double>("muhamed.Wpos", 200.0);
  WposN_    = node->declare_parameter<double>("muhamed.Wpos_N", 480.0);
  Wtan_     = node->declare_parameter<double>("muhamed.Wtan", 300.0);
  Wspace_   = node->declare_parameter<double>("muhamed.Wspace", 10.0);
  Walign_   = node->declare_parameter<double>("muhamed.Walign", 0.0);
  Wdmd_     = node->declare_parameter<double>("muhamed.Wdmd", 0.5);
  Wv_       = node->declare_parameter<double>("muhamed.Wv", 0.01);
  Wom_      = node->declare_parameter<double>("muhamed.Wom", 0.01);
  Wav_      = node->declare_parameter<double>("muhamed.Wav", 0.1);
  Waom_     = node->declare_parameter<double>("muhamed.Waom", 0.1);
  a_v_max_  = node->declare_parameter<double>("muhamed.a_v_max", 10.0);
  a_om_max_ = node->declare_parameter<double>("muhamed.a_om_max", 20.0);
  diamond_  = node->declare_parameter<bool>("muhamed.diamond_saturation", true);
  reset();
}

void Muhamed::reset() {
  const int nu = 2 * cfg_.n;
  U_prev_.assign(nu * N_, 0.0);
  v_prev_.assign(cfg_.n, 0.0);
  omega_prev_.assign(cfg_.n, 0.0);
}

void Muhamed::setPath(const std::vector<PathSample>& path) {
  const int Np = static_cast<int>(path.size());
  px_.resize(Np); py_.resize(Np); pth_.resize(Np);
  for (int i = 0; i < Np; ++i) { px_[i] = path[i].x; py_[i] = path[i].y; pth_[i] = path[i].theta; }
  path_ready_ = (Np >= 2);
}

Muhamed::Smp Muhamed::interp(double sk) const {
  const int Np = static_cast<int>(px_.size());
  double fidx = (sk - s_start_) / ds_samp_;
  fidx = std::clamp(fidx, 0.0, static_cast<double>(Np - 1));
  const int i0 = static_cast<int>(std::floor(fidx));
  const int i1 = std::min(i0 + 1, Np - 1);
  const double a = fidx - i0;
  Smp s;
  s.x   = px_[i0] * (1 - a) + px_[i1] * a;
  s.y   = py_[i0] * (1 - a) + py_[i1] * a;
  s.theta = wrap(pth_[i0] + a * wrap(pth_[i1] - pth_[i0]));
  s.xp  = pxp_[i0] * (1 - a) + pxp_[i1] * a;
  s.yp  = pyp_[i0] * (1 - a) + pyp_[i1] * a;
  s.thp = pthp_[i0] * (1 - a) + pthp_[i1] * a;
  return s;
}

void Muhamed::buildHorizonRef(double s_now, double dot_s) {
  pref_.assign(N_ + 1, {0, 0, 0});
  dpref_.assign(N_ + 1, {0, 0, 0});
  for (int k = 0; k <= N_; ++k) {
    const double sk = std::min(s_now + k * dt_ * dot_s, s_end_);
    const Smp s = interp(sk);
    pref_[k]  = {s.x, s.y, s.theta};
    dpref_[k] = {dot_s * s.xp, dot_s * s.yp, dot_s * s.thp};
  }
}

// stacked decision vector U (length nu*N); per step k:
//   U[nu*k + i]     = a_v_i,   U[nu*k + n + i] = a_omega_i
double Muhamed::cost(const std::vector<double>& U) const {
  const int n = cfg_.n, nu = 2 * n;
  std::vector<double> s = s0_;
  double total = 0.0;
  for (int k = 0; k < N_; ++k) {
    const double* Uk = &U[nu * k];
    for (int i = 0; i < n; ++i) {                       // explicit-Euler rollout
      const int b = 5 * i;
      const double x = s[b], y = s[b + 1], th = s[b + 2], v = s[b + 3], om = s[b + 4];
      const double vn = v + dt_ * Uk[i];
      s[b]     = x + dt_ * vn * std::cos(th);
      s[b + 1] = y + dt_ * vn * std::sin(th);
      s[b + 2] = th + dt_ * om;
      s[b + 3] = vn;
      s[b + 4] = om + dt_ * Uk[n + i];
    }
    const Matrix2d Rref = R2(pref_[k + 1][2]);
    const double ppx = pref_[k + 1][0], ppy = pref_[k + 1][1];
    const double vpx = dpref_[k + 1][0], vpy = dpref_[k + 1][1], omp = dpref_[k + 1][2];
    for (int i = 0; i < n; ++i) {
      const int b = 5 * i;
      const double xi = s[b], yi = s[b + 1], thi = s[b + 2], vi = s[b + 3], omi = s[b + 4];
      const Vector2d rw = Rref * Vector2d(cfg_.r_x[i], cfg_.r_y[i]);
      const double ex = xi - (ppx + rw(0)), ey = yi - (ppy + rw(1));
      const double W = (k < N_ - 1) ? Wpos_ : WposN_;
      total += W * ex * ex + W * ey * ey;
      const double vdx = vpx - omp * rw(1), vdy = vpy + omp * rw(0);
      if (vdx * vdx + vdy * vdy > 1e-8) {
        const double dth = wrap(thi - std::atan2(vdy, vdx));
        total += Wtan_ * std::sin(dth) * std::sin(dth);
      }
      total += Wv_ * vi * vi + Wom_ * omi * omi
             + Wav_ * Uk[i] * Uk[i] + Waom_ * Uk[n + i] * Uk[n + i];
      const double kap = std::abs(vi) / cfg_.v_max + std::abs(omi) / cfg_.w_max;
      const double sl = std::max(0.0, kap - 1.0);
      total += Wdmd_ * sl * sl;
    }
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j) {
        const Vector2d rd(cfg_.r_x[i] - cfg_.r_x[j], cfg_.r_y[i] - cfg_.r_y[j]);
        const double dref2 = rd.squaredNorm();
        const double dx = s[5 * i] - s[5 * j], dy = s[5 * i + 1] - s[5 * j + 1];
        const double d2 = dx * dx + dy * dy;
        total += Wspace_ * (d2 - dref2) * (d2 - dref2);
        const double dth = wrap(s[5 * i + 2] - s[5 * j + 2]);
        total += Walign_ * dth * dth;
      }
  }
  return total;
}

// warm-started projected gradient (forward-difference gradient, backtracking)
void Muhamed::solve(std::vector<double>& U) {
  const int n = cfg_.n, nu = 2 * n, dim = nu * N_;
  std::vector<double> g(dim), Ut(dim);
  auto project = [&](std::vector<double>& X) {
    for (int k = 0; k < N_; ++k)
      for (int i = 0; i < n; ++i) {
        X[nu * k + i]     = std::clamp(X[nu * k + i],     -a_v_max_,  a_v_max_);
        X[nu * k + n + i] = std::clamp(X[nu * k + n + i], -a_om_max_, a_om_max_);
      }
  };
  project(U);
  const double h = 1e-6;
  for (int it = 0; it < max_iters_; ++it) {
    const double c0 = cost(U);
    for (int d = 0; d < dim; ++d) {                     // forward-difference gradient
      const double o = U[d]; U[d] = o + h;
      g[d] = (cost(U) - c0) / h; U[d] = o;
    }
    double gn2 = 0.0; for (double gi : g) gn2 += gi * gi;
    if (gn2 < 1e-12) break;
    double step = 1.0 / std::max(1.0, std::sqrt(gn2));
    bool improved = false;
    for (int ls = 0; ls < 20; ++ls) {                   // backtracking line search
      for (int d = 0; d < dim; ++d) Ut[d] = U[d] - step * g[d];
      project(Ut);
      if (cost(Ut) < c0 - 1e-4 * step * gn2) { U = Ut; improved = true; break; }
      step *= 0.5;
    }
    if (!improved) break;
  }
}

void Muhamed::compute(const ControllerInput& in, double /*dt*/, ControllerOutput& out) {
  const int n = cfg_.n, nu = 2 * n;
  out.v.assign(n, 0.0); out.w.assign(n, 0.0);

  // need both the path and a valid progress s-range before optimizing
  if (!path_ready_ || !(in.s_end > in.s_start)) { out.diag = {0, 0, 0, 0}; return; }

  // Recompute the s-range and path s-derivatives EVERY tick from the current
  // progress. In planner mode the reference path (and its s_end) is replaced on
  // every replan, and the path (setPath) and s-range (progress) arrive on separate
  // callbacks; caching froze a STALE s_end for a whole replan cycle -> a mis-scaled
  // reference and bad tracking (this is why muhamed alone broke under the online
  // planner; waypoint mode is static so caching was fine). O(Np) finite differences,
  // negligible next to the NMPC solve.
  {
    const int Np = static_cast<int>(px_.size());
    s_start_ = in.s_start; s_end_ = in.s_end;
    ds_samp_ = (s_end_ - s_start_) / (Np - 1);
    pxp_.resize(Np); pyp_.resize(Np); pthp_.resize(Np);
    for (int i = 0; i < Np; ++i) {
      const int ip = std::min(i + 1, Np - 1), im = std::max(i - 1, 0);
      const double den = (ip - im) * ds_samp_;
      pxp_[i]  = (px_[ip] - px_[im]) / den;
      pyp_[i]  = (py_[ip] - py_[im]) / den;
      pthp_[i] = wrap(pth_[ip] - pth_[im]) / den;
    }
  }

  // initial state: rigid robot positions, measured headings, last commanded velocities
  const Vector2d p_obj(in.q(0), in.q(1));
  const Matrix2d Robj = R2(in.q(2));
  s0_.assign(5 * n, 0.0);
  for (int i = 0; i < n; ++i) {
    const Vector2d p = p_obj + Robj * Vector2d(cfg_.r_x[i], cfg_.r_y[i]);
    const int b = 5 * i;
    s0_[b] = p(0); s0_[b + 1] = p(1); s0_[b + 2] = in.theta_arr[i];
    s0_[b + 3] = v_prev_[i]; s0_[b + 4] = omega_prev_[i];
  }

  buildHorizonRef(in.s, in.ds);

  // warm start: shift previous optimal sequence forward one step
  std::vector<double> U(nu * N_);
  for (int k = 0; k < N_; ++k)
    for (int d = 0; d < nu; ++d)
      U[nu * k + d] = (k < N_ - 1) ? U_prev_[nu * (k + 1) + d] : U_prev_[nu * (N_ - 1) + d];

  solve(U);
  U_prev_ = U;

  // first-step accelerations -> integrate from previous velocities
  for (int i = 0; i < n; ++i) {
    out.v[i] = v_prev_[i] + dt_ * U[i];
    out.w[i] = omega_prev_[i] + dt_ * U[n + i];
  }

  const double s_max = applyDiamond(out.v, out.w, cfg_.v_max, cfg_.w_max, diamond_);
  v_prev_ = out.v; omega_prev_ = out.w;                 // velocity memory (post-saturation)
  out.diag = {0.0, 0.0, 0.0, s_max};
}

}  // namespace controller