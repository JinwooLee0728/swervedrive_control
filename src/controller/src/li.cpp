// li.cpp : implementation of the Li (2016) CLM phase-machine baseline.

#include "controller/li.hpp"

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

Li::Li(rclcpp::Node* node, const ControllerConfig& cfg) : cfg_(cfg) {
  planner_mode_  = node->declare_parameter<std::string>("li.planner_mode", "arc_fit");
  K_waypoints_   = node->declare_parameter<int>("li.K_waypoints", 30);
  eps_arc_fit_   = node->declare_parameter<double>("li.eps_arc_fit", 0.02);
  eps_psi_line_  = node->declare_parameter<double>("li.eps_psi_line", 0.05);
  max_partitions_= node->declare_parameter<int>("li.max_partitions", 30);
  k_align_       = node->declare_parameter<double>("li.k_align", 2.0);
  k1_trans_      = node->declare_parameter<double>("li.k1_trans", 0.6);
  k2_trans_      = node->declare_parameter<double>("li.k2_trans", 1.5);
  k_rot_         = node->declare_parameter<double>("li.k_rot", 1.0);
  tol_pos_       = node->declare_parameter<double>("li.tol_pos", 0.05);
  tol_ang_       = node->declare_parameter<double>("li.tol_ang", 0.05);
  diamond_       = node->declare_parameter<bool>("li.diamond_saturation", true);

  mean_r_.setZero();
  for (int k = 0; k < cfg_.n; ++k) mean_r_ += Vector2d(cfg_.r_x[k], cfg_.r_y[k]);
  if (cfg_.n > 0) mean_r_ /= static_cast<double>(cfg_.n);
}

void Li::reset() { phases_.clear(); idx_ = 0; idx_last_ = -1; plan_built_ = false; }

void Li::setPath(const std::vector<PathSample>& path) {
  path_ = path;
  plan_built_ = false;   // rebuild on next compute, using the live initial pose
}

PathSample Li::interpAt(double fidx) const {
  const int N = static_cast<int>(path_.size());
  fidx = std::clamp(fidx, 0.0, static_cast<double>(N - 1));
  const int i0 = static_cast<int>(std::floor(fidx));
  const int i1 = std::min(i0 + 1, N - 1);
  const double a = fidx - i0;
  PathSample s;
  s.x = path_[i0].x * (1 - a) + path_[i1].x * a;
  s.y = path_[i0].y * (1 - a) + path_[i1].y * a;
  s.theta = wrap(path_[i0].theta + a * wrap(path_[i1].theta - path_[i0].theta));
  return s;
}

// ----- planner dispatch -----
void Li::buildPhases(double theta_obj_init) {
  phases_.clear();
  if (planner_mode_ == "uniform") buildUniform(theta_obj_init);
  else                            buildArcFit(theta_obj_init);
  Phase done; done.mode = Mode::Done;
  phases_.push_back(done);
  idx_ = 0; idx_last_ = -1;
}

// align_translate + translate, plus optional in-place align_rotate + rotate
void Li::appendLinePartition(std::vector<Phase>& phases, const Eigen::Vector2d& p_cur,
                                 double psi_cur, const Eigen::Vector2d& p_tgt, double psi_tgt) {
  const double bearing = std::atan2(p_tgt(1) - p_cur(1), p_tgt(0) - p_cur(0));
  Phase at; at.mode = Mode::AlignTranslate; at.theta_target.assign(cfg_.n, bearing);
  phases.push_back(at);

  Phase tr; tr.mode = Mode::Translate; tr.body_waypoint = p_tgt;
  phases.push_back(tr);

  if (std::abs(wrap(psi_tgt - psi_cur)) > tol_ang_) {
    Phase ar; ar.mode = Mode::AlignRotate; ar.arc = false; ar.target_psi = psi_tgt;
    phases.push_back(ar);
    Phase ro; ro.mode = Mode::Rotate; ro.arc = false; ro.target_psi = psi_tgt; ro.i_ref = 0;
    phases.push_back(ro);
  }
}

// uniform planner: K+1 equal-arc-length waypoints, every rotation in-place
void Li::buildUniform(double theta_obj_init) {
  const int N = static_cast<int>(path_.size());
  const int n_wp = K_waypoints_ + 1;
  Vector2d p_cur(path_[0].x, path_[0].y);
  double psi_cur = theta_obj_init;
  for (int j = 1; j <= n_wp; ++j) {
    const double fidx = static_cast<double>(N - 1) * j / (n_wp);
    const PathSample wp = interpAt(fidx);
    const Vector2d p_tgt(wp.x, wp.y);
    appendLinePartition(phases_, p_cur, psi_cur, p_tgt, wp.theta);
    p_cur = p_tgt; psi_cur = wp.theta;
  }
}

// arc-fit planner: RDP-style splitting; line vs arc partitions
void Li::buildArcFit(double theta_obj_init) {
  const int N = static_cast<int>(path_.size());
  std::vector<int> bp = {0, N - 1};

  while (static_cast<int>(bp.size()) < max_partitions_) {
    double max_err = 0.0; int seg = -1, pt = -1;
    for (int k = 0; k + 1 < static_cast<int>(bp.size()); ++k) {
      const int i1 = bp[k], i2 = bp[k + 1];
      if (i2 - i1 < 2) continue;
      const Vector2d p1(path_[i1].x, path_[i1].y), p2(path_[i2].x, path_[i2].y);
      const double dpsi = wrap(path_[i2].theta - path_[i1].theta);
      const bool line = std::abs(dpsi) < eps_psi_line_;
      Vector2d p_r;
      if (!line) {
        const Matrix2d Minv = (Matrix2d::Identity() - R2(dpsi)).inverse();
        p_r = p1 + Minv * (p2 - p1);
      }
      for (int i = i1 + 1; i < i2; ++i) {
        const double alpha = static_cast<double>(i - i1) / (i2 - i1);
        Vector2d pred;
        if (line) pred = p1 + alpha * (p2 - p1);
        else      pred = R2(alpha * dpsi) * (p1 - p_r) + p_r;
        const double err = (pred - Vector2d(path_[i].x, path_[i].y)).norm();
        if (err > max_err) { max_err = err; seg = k; pt = i; }
      }
    }
    if (max_err <= eps_arc_fit_ || seg < 0) break;
    bp.insert(bp.begin() + seg + 1, pt);
  }

  Vector2d p_cur(path_[bp[0]].x, path_[bp[0]].y);
  double psi_cur = theta_obj_init;
  for (int k = 0; k + 1 < static_cast<int>(bp.size()); ++k) {
    const int i2 = bp[k + 1];
    const Vector2d p_tgt(path_[i2].x, path_[i2].y);
    const double psi_tgt = path_[i2].theta;
    if (std::abs(wrap(psi_tgt - psi_cur)) < eps_psi_line_) {
      appendLinePartition(phases_, p_cur, psi_cur, p_tgt, psi_tgt);
    } else {
      Phase ar; ar.mode = Mode::AlignRotate; ar.arc = true; ar.p_end = p_tgt; ar.target_psi = psi_tgt;
      phases_.push_back(ar);
      Phase ro; ro.mode = Mode::Rotate; ro.arc = true; ro.p_end = p_tgt; ro.target_psi = psi_tgt; ro.i_ref = 0;
      phases_.push_back(ro);
    }
    p_cur = p_tgt; psi_cur = psi_tgt;
  }
}

// ----- snapshot phase-entry parameters (once per transition) -----
void Li::snapshotPhaseEntry(const ControllerInput& in,
                                const std::vector<Eigen::Vector2d>& p_arr, const Eigen::Vector2d& p_bar) {
  Phase& ph = phases_[idx_];
  const int n = cfg_.n;

  if (ph.mode == Mode::Translate) {
    ph.d_list.resize(n);
    for (int k = 0; k < n; ++k) ph.d_list[k] = p_arr[k] - p_bar;          // d_i = p_i - p_bar (Eq.10)
    const double theta_body = in.q(2);
    ph.target_centroid = ph.body_waypoint + R2(theta_body) * mean_r_;     // body_wp -> centroid

  } else if (ph.mode == Mode::AlignRotate) {
    const Vector2d body_pos(in.q(0), in.q(1));
    const double body_theta = in.q(2);
    const double dpsi = wrap(ph.target_psi - body_theta);
    Vector2d p_r;
    if (!ph.arc) {
      p_r = body_pos;
    } else {
      const Matrix2d M = Matrix2d::Identity() - R2(dpsi);
      p_r = (std::abs(M.determinant()) < 1e-6) ? body_pos
                                               : body_pos + M.inverse() * (ph.p_end - body_pos);
    }
    ph.p_r = p_r; ph.delta_psi = dpsi;
    ph.theta_target.assign(n, 0.0); ph.sign_choice.assign(n, 0.0);
    for (int k = 0; k < n; ++k) {                                         // least-headings sign select
      const Vector2d g = p_arr[k] - p_r;
      const double phi = std::atan2(g(1), g(0));
      const double tp = wrap(phi + M_PI / 2.0), tm = wrap(phi - M_PI / 2.0);
      const double dp = std::abs(wrap(tp - in.theta_arr[k])), dm = std::abs(wrap(tm - in.theta_arr[k]));
      if (dp <= dm) { ph.theta_target[k] = tp; ph.sign_choice[k] = +1.0; }
      else          { ph.theta_target[k] = tm; ph.sign_choice[k] = -1.0; }
    }

  } else if (ph.mode == Mode::Rotate) {
    const Phase& al = phases_[idx_ - 1];                                  // inherit from align_rotate
    ph.p_r = al.p_r; ph.delta_psi = al.delta_psi; ph.sign_choice = al.sign_choice;
    ph.gamma_list.resize(n);
    for (int k = 0; k < n; ++k) ph.gamma_list[k] = p_arr[k] - ph.p_r;     // gamma_i = p_i - p_r (Eq.7)
    const Vector2d gr = ph.gamma_list[ph.i_ref];
    const double phi_ref = std::atan2(gr(1), gr(0));
    ph.target_theta_ref = wrap(phi_ref + ph.sign_choice[ph.i_ref] * M_PI / 2.0 + ph.delta_psi);
  }
  // AlignTranslate / Done: nothing
}

void Li::compute(const ControllerInput& in, double /*dt*/, ControllerOutput& out) {
  const int n = cfg_.n;
  out.v.assign(n, 0.0); out.w.assign(n, 0.0);

  if (path_.empty()) { out.diag = {0.0, 0.0, 0.0, 0.0}; return; }         // no path yet -> hold
  if (!plan_built_) { buildPhases(in.q(2)); plan_built_ = true; }

  // robot positions from the rigid relation, and the centroid
  const Vector2d p_obj(in.q(0), in.q(1));
  const Matrix2d Robj = R2(in.q(2));
  std::vector<Vector2d> p_arr(n);
  Vector2d p_bar(0, 0);
  for (int k = 0; k < n; ++k) { p_arr[k] = p_obj + Robj * Vector2d(cfg_.r_x[k], cfg_.r_y[k]); p_bar += p_arr[k]; }
  p_bar /= static_cast<double>(n);

  idx_ = std::min(idx_, static_cast<int>(phases_.size()) - 1);            // cap at done
  if (idx_ != idx_last_) { snapshotPhaseEntry(in, p_arr, p_bar); idx_last_ = idx_; }

  Phase& ph = phases_[idx_];
  switch (ph.mode) {
    case Mode::AlignTranslate:
    case Mode::AlignRotate: {                                              // in-place rotate, v=0
      double max_err = 0.0;
      for (int k = 0; k < n; ++k) {
        const double e = wrap(ph.theta_target[k] - in.theta_arr[k]);
        out.w[k] = k_align_ * e; out.v[k] = 0.0;
        max_err = std::max(max_err, std::abs(e));
      }
      if (max_err < tol_ang_) ++idx_;
      break;
    }
    case Mode::Translate: {                                               // Prop.3 / Eq.(11)
      const double x_d = ph.target_centroid(0), y_d = ph.target_centroid(1);
      for (int k = 0; k < n; ++k) {
        const double e_xi = (x_d + ph.d_list[k](0)) - p_arr[k](0);
        const double e_yi = (y_d + ph.d_list[k](1)) - p_arr[k](1);
        const double e_th = wrap(std::atan2(e_yi, e_xi) - in.theta_arr[k]);
        out.v[k] = k1_trans_ * std::hypot(e_xi, e_yi) * std::cos(e_th);
        out.w[k] = k2_trans_ * e_th;
      }
      if ((p_bar - ph.target_centroid).norm() < tol_pos_) ++idx_;
      break;
    }
    case Mode::Rotate: {                                                  // Prop.2, per-robot sign
      const double u_t = k_rot_ * wrap(ph.target_theta_ref - in.theta_arr[ph.i_ref]);
      for (int k = 0; k < n; ++k) {
        out.v[k] = ph.sign_choice[k] * ph.gamma_list[k].norm() * u_t;
        out.w[k] = u_t;
      }
      if (std::abs(wrap(ph.target_theta_ref - in.theta_arr[ph.i_ref])) < tol_ang_) ++idx_;
      break;
    }
    case Mode::Done:
      break;                                                              // zeros
  }

  const double s_max = applyDiamond(out.v, out.w, cfg_.v_max, cfg_.w_max, diamond_);
  out.diag = {static_cast<double>(idx_), static_cast<double>(static_cast<int>(ph.mode)), 0.0, s_max};
}

}  // namespace controller