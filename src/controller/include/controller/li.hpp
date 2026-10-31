// li.hpp : Li (2016) CLM-based baseline (stateful phase machine).
//
//   W. Li, "Notion of Control-Law Module and Modular Framework of Cooperative
//   Transportation Using Multiple Nonholonomic Robotic Agents With Physical
//   Rigid-Formation-Motion Constraints," IEEE T-Cybern 46(5):1242-1248, 2016.
//
// Ignores the regulated reference. Builds a plan from the whole path once
// (received via setPath), partitioning it into line/arc segments, then tracks
// it through an alternating align -> translate / align -> rotate phase sequence.
// Stateful: persistent phase index. In ROS each run is a fresh node, so the
// "clear between scenarios" requirement of li2016.m is met automatically.

#ifndef CONTROLLER_LI_HPP
#define CONTROLLER_LI_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "controller/controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller {

class Li : public ControllerBase {
 public:
  Li(rclcpp::Node* node, const ControllerConfig& cfg);
  void compute(const ControllerInput& in, double dt, ControllerOutput& out) override;
  void setPath(const std::vector<PathSample>& path) override;
  void reset() override;
  std::string name() const override { return "li"; }

 private:
  enum class Mode { AlignTranslate, Translate, AlignRotate, Rotate, Done };

  struct Phase {
    Mode mode;
    std::vector<double> theta_target;        // align phases (per robot)
    // translate
    Eigen::Vector2d body_waypoint{0, 0};
    Eigen::Vector2d target_centroid{0, 0};   // filled at entry
    std::vector<Eigen::Vector2d> d_list;     // filled at entry
    // align_rotate / rotate
    bool arc = false;                        // rotation_type: false=in_place, true=arc
    double target_psi = 0.0;
    Eigen::Vector2d p_end{0, 0};
    Eigen::Vector2d p_r{0, 0};               // filled at entry
    double delta_psi = 0.0;                  // filled at entry
    std::vector<double> sign_choice;         // filled at entry
    // rotate
    std::vector<Eigen::Vector2d> gamma_list; // filled at entry
    double target_theta_ref = 0.0;           // filled at entry
    int i_ref = 0;
  };

  // planner
  void buildPhases(double theta_obj_init);
  void buildUniform(double theta_obj_init);
  void buildArcFit(double theta_obj_init);
  void appendLinePartition(std::vector<Phase>& phases, const Eigen::Vector2d& p_cur,
                           double psi_cur, const Eigen::Vector2d& p_tgt, double psi_tgt);
  PathSample interpAt(double fidx) const;

  // per-tick
  void snapshotPhaseEntry(const ControllerInput& in,
                          const std::vector<Eigen::Vector2d>& p_arr, const Eigen::Vector2d& p_bar);

  ControllerConfig cfg_;
  std::vector<PathSample> path_;
  std::vector<Phase> phases_;
  int idx_ = 0, idx_last_ = -1;
  bool plan_built_ = false;
  Eigen::Vector2d mean_r_{0, 0};

  // gains / planner params
  double k_align_, k1_trans_, k2_trans_, k_rot_, tol_pos_, tol_ang_;
  std::string planner_mode_;
  int K_waypoints_, max_partitions_;
  double eps_arc_fit_, eps_psi_line_;
  bool diamond_;
};

}  // namespace controller
#endif