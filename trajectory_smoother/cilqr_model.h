#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace ceshi {
namespace planning {

namespace CilqrCarModel {
constexpr int nx = 5;
constexpr int nu = 2;
using A = Eigen::Matrix<double, nx, nx>;
using B = Eigen::Matrix<double, nx, nu>;
// using K = Eigen::Matrix<double, nx, nx>;
// using k = Eigen::Matrix<double, nx, nu>;
using X_state = Eigen::Matrix<double, nx, 1>;
using U_state = Eigen::Matrix<double, nu, 1>;

using l_x = Eigen::Matrix<double, nx, 1>;
using l_xx = Eigen::Matrix<double, nx, nx>;
using V_x = Eigen::Matrix<double, nx, 1>;
using V_xx = Eigen::Matrix<double, nx, nx>;
using l_u = Eigen::Matrix<double, nu, 1>;
using l_uu = Eigen::Matrix<double, nu, nu>;
using l_ux = Eigen::Matrix<double, nu, nx>;
using k = Eigen::Matrix<double, nu, 1>;
using K = Eigen::Matrix<double, nu, nx>;
using Q_x = Eigen::Matrix<double, nx, 1>;
using Q_xx = Eigen::Matrix<double, nx, nx>;
using Q_u = Eigen::Matrix<double, nu, 1>;
using Q_uu = Eigen::Matrix<double, nu, nu>;
using Q_ux = Eigen::Matrix<double, nu, nx>;
using Q_ux_rug = Eigen::Matrix<double, nu, nx>;
using Q_uu_rug = Eigen::Matrix<double, nu, nu>;

// 增益矩阵
using K = Eigen::Matrix<double, nu, nx>;

// 前馈增益向量 k：nu × 1
using k = Eigen::Matrix<double, nu, 1>;
struct TransMatrix {
  CilqrCarModel::A A{};
  CilqrCarModel::B B{};
};

struct PieceBoundary {
  double s;           // 采样点的纵向位置
  double left_bound;  // 左边界（横向距离/相对l坐标）
  double right_bound; // 右边界
  double x{0.};
  double y{0.};
  // 可选: 增加标记、障碍物信息等
};
struct PathCostPara {
  Eigen::Matrix<double, nx, 1> P_x;
  Eigen::Matrix<double, nx, 1> P_y;
  Eigen::Matrix<double, nx, 1> P_heading;
  Eigen::Matrix<double, nx, 1> P_v;
  Eigen::Matrix<double, nx, 1> P_kappa;
  CilqrCarModel::A state_cost_matrix;
};

struct PathModelState {
  double state_x{0.};
  double state_y{0.};
  double state_v{0.};
  double state_heading{0.};
  double state_kappa{0.};
  PathModelState() = default;
  PathModelState(double x, double y, double v, double heading, double kappa)
      : state_x(x), state_y(y), state_v(v), state_heading(heading),
        state_kappa(kappa) {}
  void SetZero() {
    state_x = 0.;
    state_y = 0.;
    state_v = 0.;
    state_heading = 0.;
    state_kappa = 0.;
  }
};
struct PathModelControl {
  double control_a{0.};
  double control_dkappa{0.};
  void SetZero() {
    control_a = 0.;
    control_dkappa = 0.;
  }
};
struct PathKappa {
  double kappa{0.};
  double s{0.};
  void SetZero() {
    kappa = 0.;
    s = 0.;
  }
};
struct state {
  double state_x{0.};
  double state_y{0.};
  double state_v{0.};
  double state_heading{0.};
  double state_kappa{0.};
};
struct OptVariables {
  PathModelState xk;   // 当前时刻的状态
  PathModelControl uk; // 当前时刻的控制量
};
struct PathBackwardPerturbationPara {
  std::vector<CilqrCarModel::l_x> l_x{};
  std::vector<CilqrCarModel::l_xx> l_xx{};
  std::vector<CilqrCarModel::l_u> l_u{};
  std::vector<CilqrCarModel::l_uu> l_uu{};
  std::vector<CilqrCarModel::l_ux> l_ux{};
  PathBackwardPerturbationPara(std::vector<CilqrCarModel::l_x> l_x1,
                               std::vector<CilqrCarModel::l_xx> l_xx1,
                               std::vector<CilqrCarModel::l_u> l_u1,
                               std::vector<CilqrCarModel::l_uu> l_uu1,
                               std::vector<CilqrCarModel::l_ux> l_ux1) {
    l_x = l_x1;
    l_xx = l_xx1;
    l_u = l_u1;
    l_uu = l_uu1;
    l_ux = l_ux1;
  }
};

struct PathBackwardPerturbationParaControl {
  std::vector<CilqrCarModel::l_u> l_u{};
  std::vector<CilqrCarModel::l_uu> l_uu{};
};

struct PathBackwardPerturbationParaState {
  std::vector<CilqrCarModel::l_x> l_x{};
  std::vector<CilqrCarModel::l_xx> l_xx{};
};
} // namespace CilqrCarModel

class PathCilqrModelCar {
public:
  PathCilqrModelCar(const std::string &name,
                 std::vector<CilqrCarModel::PathModelState> state_seq_input,
                 std::vector<CilqrCarModel::PathModelControl> control_input,
                 double dt_input);
  PathCilqrModelCar(const std::string &name,
                 std::vector<CilqrCarModel::PathModelState> state_seq_input);

  ~PathCilqrModelCar() = default;
  CilqrCarModel::X_state
  ForwardStep(const CilqrCarModel::X_state &x_state_old,
              const Eigen::Matrix<double, 2, 1> best_control);
  bool Process();
  bool Process1();

  const std::vector<CilqrCarModel::TransMatrix> &error_path_model_matrix() const {
    return error_path_model_matrix_;
  }
  const std::vector<CilqrCarModel::TransMatrix> &path_forward_model_matrix() const {
    return path_forward_model_matrix_;
  }
  const CilqrCarModel::PathCostPara &path_cost_para() const {
    return path_cost_para_;
  }

private:
  double dt = 0.;
  std::string name_{""};
  std::vector<CilqrCarModel::PathModelState> state_seq;
  std::vector<CilqrCarModel::PathModelControl> control_seq;
  std::vector<CilqrCarModel::TransMatrix> error_path_model_matrix_{};
  std::vector<CilqrCarModel::TransMatrix> path_forward_model_matrix_{};
  CilqrCarModel::PathCostPara path_cost_para_{};
};

} // namespace planning
}  // namespace ceshi