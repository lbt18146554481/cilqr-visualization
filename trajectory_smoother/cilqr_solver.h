#pragma once
#include "cilqr_model.h"
#include "src/planning/math/curve1d/spline.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include "planning/planning_map/planning_map.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace ceshi {
namespace planning {
constexpr int num_constrain = 6;
using PathCarModelOutput = std::pair<std::vector<CilqrCarModel::PathModelControl>,
                                  std::vector<CilqrCarModel::PathModelState>>;

using MatCX = Eigen::Matrix<double, num_constrain, 5>;
using MatCU = Eigen::Matrix<double, num_constrain, 2>;
class PathCilqrSolverAML {
public:
  ~PathCilqrSolverAML() = default;
  PathCilqrSolverAML() = default;
  PathCilqrSolverAML(
      const CilqrCarModel::PathModelState &ego_state,
      const std::vector<CilqrCarModel::OptVariables> &path_goal,
      const std::vector<CilqrCarModel::OptVariables> &path_goal_la, // 历史轨迹信息
      const std::vector<double> &s_gap_seq,
      const std::vector<CilqrCarModel::PieceBoundary> &piece_boundaries,
      const size_t &steps, const tk::spline &bound_left_l_spline,
      const tk::spline &bound_right_l_spline,
      std::vector<CilqrCarModel::PieceBoundary> orin_boundaries,
      const ReferenceLine& reference_line);
  PathCarModelOutput Optimize();

  bool CheckIllness(std::vector<CilqrCarModel::PathModelState> &ego_deduction_data);

  PathCarModelOutput GetOptimalControlSeq();

  std::pair<std::vector<CilqrCarModel::K>, std::vector<CilqrCarModel::k>> BackwardPass(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
      const double &lambda,
      const std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,
      const std::vector<Eigen::Matrix<double, num_constrain, num_constrain>>
          &pho,
      const std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck);

  CilqrCarModel::PathBackwardPerturbationPara GetCostDerivatives(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq);

  CilqrCarModel::PathBackwardPerturbationParaControl GetControlCostDerivatives(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq);

  CilqrCarModel::PathBackwardPerturbationParaState GetStateCostDerivatives(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq);

  std::pair<std::vector<CilqrCarModel::PathModelControl>,
            std::vector<CilqrCarModel::X_state>>
  ForwardPass(const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
              const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
              const std::pair<std::vector<CilqrCarModel::K>,
                              std::vector<CilqrCarModel::k>> &best_control_rate,
              double aplha);
  CilqrCarModel::X_state
  ForwardSimulation(const CilqrCarModel::X_state &x_state_old,
                    const Eigen::Matrix<double, 2, 1> best_control,
                    const int &index);
  double GetTotalCost(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
      std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,
      std::vector<Eigen::Matrix<double, num_constrain, num_constrain>> &pho,
      std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck);

  Eigen::Matrix<double, num_constrain, 1>
  GetConstrain(const CilqrCarModel::PathModelState &rough_state,
               const CilqrCarModel::PathModelControl &rough_control, size_t index);

  std::pair<std::vector<MatCX>, std::vector<MatCU>>
  GetConstrainDre(
      const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
      const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq);
  void UpdatePara(
      std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,
      std::vector<Eigen::Matrix<double, num_constrain, num_constrain>> &pho,
      std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck, int m);

  bool IsIllness() { return is_illness_; };
bool CheckKappaIllness(
    CilqrCarModel::PathModelState& ego_data);
bool CheckColIllness(
    CilqrCarModel::PathModelState& ego_data,double dt);
  void
  UpdateData(std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
             std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
             const std::pair<std::vector<CilqrCarModel::PathModelControl>,
                             std::vector<CilqrCarModel::X_state>> &forward_result);

  void reset();
  // 根据坐标点x,y计算边界以及对应的坐标l；
inline void calc_s_l_heading_from_reference(
    double x, double y, const std::vector<CilqrCarModel::PieceBoundary> &ref_path,
    double &s_out, double &l_out, double &heading_out,double &left_bound,double &right_bound,
    const ReferenceLine& reference_line_);
  CilqrCarModel::X_state
  ForwardStep(const CilqrCarModel::X_state &x_state_old,
              const Eigen::Matrix<double, 2, 1> best_control);
inline Eigen::Matrix2d fastInverse2x2(const Eigen::Matrix2d& m) {
    double det = m(0,0) * m(1,1) - m(0,1) * m(1,0);
    assert(std::abs(det) > 1e-12 && "Q_uu nearly singular!");
    Eigen::Matrix2d inv;
    inv(0,0) =  m(1,1) / det;
    inv(0,1) = -m(0,1) / det;
    inv(1,0) = -m(1,0) / det;
    inv(1,1) =  m(0,0) / det;
    return inv;
}
private:
    std::ostringstream oss_cilqr;

  CilqrCarModel::PathModelState ego_state_;
  std::vector<CilqrCarModel::OptVariables> path_goal_{};
std::vector<CilqrCarModel::OptVariables> path_goal_cur{};
std::vector<CilqrCarModel::OptVariables> path_goal_last{};
  std::vector<double> t_gap_seq_{};
  std::vector<CilqrCarModel::PieceBoundary> piece_boundaries_{};
  std::vector<CilqrCarModel::PathKappa> kappa_seq_{};
  std::shared_ptr<PathCilqrModelCar> cilqr_model_;
  std::vector<planning::CilqrCarModel::PieceBoundary> orin_boundaries_{};

  size_t steps_;
  size_t max_iters_ = 30;
  double dt = 0;
  double r_{0.0};
  double c_f_{0.0};
  double min_r_{0.0};
  double width;
  double length;
    const ReferenceLine* reference_line_;



  tk::spline bound_left_l_spline_;
  tk::spline bound_right_l_spline_;
  constexpr static int kNoise = 5;
  bool is_illness_{false};
  bool iter_effective_flag = false;
  // 线性搜索相关项，可适当减少
  std::vector<double> alpha_options = {1.0,
                                       0.25,
                                       0.0625,
                                       0.0018125};

double delt_v = 0;                         // 理论delta_V，通常用于iLQR步长线性搜索中的代价预测

double control_a_cost = 1;              // 控制输入：加速度的代价权重（用于cost二次型）
double control_dkappa_cost = 1;          // 控制输入：曲率变化率的代价权重

double max_acc = 1.6,min_acc=-6.;                       // 最大允许加速度（m/s^2），正负方向都用此界限
double max_dkappa = 0.3187;          // 最大允许的曲率变化率（rad/m）+buffer
double max_kappa = 0.3187 - 0.0187;        // 最大允许的绝对曲率（rad/m）-buffer
double origin_max_kappa = 0.3187;  // 最大允许的绝对曲率（rad/m），即|kappa|上限
// 状态项权重
double final_x_weight = 100;                // 终点状态的cost权重
double ref_x_weight = 1;                // 过程中的状态跟踪cost权重

// 车辆角点在车体坐标系下的相对坐标（6个点：四角+左右中点），用于投影到道路边界做角点约束
std::vector<std::vector<double>> dxys;

size_t max_num = 0;                        // 角点数（dxys.size），车辆多边形边界约束数量

// 增广拉格朗日法相关
double safe_bound = 0.2;                   // 角点与道路边界的最小安全距离（安全裕度，单位米）
double uki = 20;                           // 增广拉格朗日惩罚参数，决定罚函数“刚度”或收敛速度

};
} // namespace planning
}  // namespace ceshi