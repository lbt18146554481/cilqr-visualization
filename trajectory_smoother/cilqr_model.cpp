
#include "cilqr_model.h"
namespace ceshi {
namespace planning {
// 考虑线性模型中二阶项对状态量影响的模型构造方法
PathCilqrModelCar::PathCilqrModelCar(
    const std::string &name,
    std::vector<CilqrCarModel::PathModelState> state_seq_input,
    std::vector<CilqrCarModel::PathModelControl> control_input, double dt_input) {
  name_ = name;
  dt = dt_input;
  state_seq = state_seq_input;
  control_seq = control_input;
  error_path_model_matrix_.resize(state_seq.size() - 1);
  for (auto &matrix : error_path_model_matrix_) {
    matrix.A = CilqrCarModel::A::Zero();
    matrix.B = CilqrCarModel::B::Zero();
  }
  path_forward_model_matrix_.resize(state_seq.size() - 1);
  for (auto &matrix : path_forward_model_matrix_) {
    matrix.A = CilqrCarModel::A::Zero();
    matrix.B = CilqrCarModel::B::Zero();
  }
  Process();
}
// 不考虑线性模型中二阶项影响的模型构造方法
PathCilqrModelCar::PathCilqrModelCar(
    const std::string &name,
    std::vector<CilqrCarModel::PathModelState> state_seq_input) {
  name_ = name;
  state_seq = state_seq_input;
  error_path_model_matrix_.resize(state_seq.size());
  for (auto &matrix : error_path_model_matrix_) {
    matrix.A = CilqrCarModel::A::Zero();
    matrix.B = CilqrCarModel::B::Zero();
  }
  path_forward_model_matrix_.resize(state_seq.size());
  for (auto &matrix : path_forward_model_matrix_) {
    matrix.A = CilqrCarModel::A::Zero();
    matrix.B = CilqrCarModel::B::Zero();
  }
  Process();
}

bool PathCilqrModelCar::Process() {
  //   const auto& path_cilqr =
  //       config::PlanningConfig::Instance()->planning_research_config().path_cilqr;
  // line model
  error_path_model_matrix_.resize(state_seq.size() - 1);
  path_forward_model_matrix_.resize(state_seq.size() - 1);
  for (size_t i = 0; i < state_seq.size() - 1; i++) {
    double v = state_seq[i].state_v;
    double kappa = state_seq[i].state_kappa;
    double heading = state_seq[i].state_heading;
    error_path_model_matrix_[i].A << 1.0, 0, cos(heading) * dt,
        -v * sin(heading) * dt, 0, 0, 1, sin(heading) * dt,
        v * cos(heading) * dt, 0, 0, 0, 1, 0, 0, 0, 0, kappa * dt, 1, v * dt, 0,
        0, 0, 0, 1;
    error_path_model_matrix_[i].B <<
        /* row1 */ 0,
        0,
        /* row2 */ 0, 0,
        /* row3 */ dt, 0,
        /* row4 */ 0, 0,
        /* row5 */ 0, dt;
    path_forward_model_matrix_[i].A << 1.0, 0, cos(heading) * dt, 0, 0, 0, 1,
        sin(heading) * dt, 0, 0, 0, 0, 1, 0, 0, 0, 0, kappa * dt, 1, 0, 0, 0, 0,
        0, 1;
    path_forward_model_matrix_[i].B <<
        /* row1 */ 0,
        0,
        /* row2 */ 0, 0,
        /* row3 */ dt, 0,
        /* row4 */ 0, 0,
        /* row5 */ 0, dt;

    // 矩阵维度为5*2；
    //  // cost weight matrix
    path_cost_para_.P_x << 1, 0, 0, 0, 0;
    path_cost_para_.P_y << 0, 1, 0, 0, 0;
    path_cost_para_.P_v << 0, 0, 1, 0, 0;
    path_cost_para_.P_heading << 0, 0, 0, 1, 0;
    path_cost_para_.P_kappa << 0, 0, 0, 0, 1;
    // //矩阵维度为5*5；
  }
  path_cost_para_.state_cost_matrix.setZero();
  path_cost_para_.state_cost_matrix.diagonal() << 1, 1, 1, 1, 1;

  return true;
}

bool PathCilqrModelCar::Process1() {
  error_path_model_matrix_.resize(state_seq.size());
  path_forward_model_matrix_.resize(state_seq.size());
  for (size_t i = 0; i < state_seq.size() - 1; i++) {
    double v = state_seq[i].state_v;
    double kappa = state_seq[i].state_kappa;
    double heading = state_seq[i].state_heading;
    double a = control_seq[i].control_a;
    double dk = control_seq[i].control_dkappa;
    error_path_model_matrix_[i].A << 1.0, 0, cos(heading) * dt,
        -v * sin(heading) * dt - 0.5 * a * sin(heading) * dt * dt, 0, 0, 1,
        sin(heading) * dt,
        v * cos(heading) * dt + 0.5 * a * cos(heading) * dt * dt, 0, 0, 0, 1, 0,
        0, 0, 0, kappa * dt + 0.5 * dk * dt * dt, 1, v * dt + 0.5 * a * dt * dt,
        0, 0, 0, 0, 1;
    error_path_model_matrix_[i].B <<
        /* row1 */ 0.5 * cos(heading) * dt * dt,
        0,
        /* row2 */ 0.5 * sin(heading) * dt * dt, 0,
        /* row3 */ dt, 0,
        /* row4 */ 0.5 * kappa * dt * dt, 0.5 * v * dt * dt,
        /* row5 */ 0, dt;
    path_forward_model_matrix_[i].A << 1.0, 0, cos(heading) * dt, 0, 0, 0, 1,
        sin(heading) * dt, 0, 0, 0, 0, 1, 0, 0, 0, 0, kappa * dt, 1, 0, 0, 0, 0,
        0, 1;
    path_forward_model_matrix_[i].B <<
        /* row1 */ 0.5 * cos(heading) * dt * dt,
        0,
        /* row2 */ 0.5 * sin(heading) * dt * dt, 0,
        /* row3 */ dt, 0,
        /* row4 */ 0.5 * kappa * dt * dt, 0.5 * v * dt * dt,
        /* row5 */ 0, dt;
  }
  path_cost_para_.P_x << 1, 0, 0, 0, 0;
  path_cost_para_.P_y << 0, 1, 0, 0, 0;
  path_cost_para_.P_v << 0, 0, 1, 0, 0;
  path_cost_para_.P_heading << 0, 0, 0, 1, 0;
  path_cost_para_.P_kappa << 0, 0, 0, 0, 1;

  path_cost_para_.state_cost_matrix.setZero();
  path_cost_para_.state_cost_matrix.diagonal() << 1, 1, 1, 1, 1;

  return true;
}

CilqrCarModel::X_state
PathCilqrModelCar::ForwardStep(const CilqrCarModel::X_state &x_state_old,
                            const Eigen::Matrix<double, 2, 1> best_control) {
  CilqrCarModel::X_state next;
  // 当前状态
  double x = x_state_old(0, 0);
  double y = x_state_old(1, 0);
  double v = x_state_old(2, 0);
  double psi = x_state_old(3, 0);
  double kappa = x_state_old(4, 0);
  // 控制量
  double a = best_control(0, 0);
  double dkappa = best_control(1, 0);

  // Euler 离散化更新
  next(0, 0) = x + v * std::cos(psi) * dt + 0.5 * a * std::cos(psi) * dt * dt;
  next(1, 0) = y + v * std::sin(psi) * dt + 0.5 * a * std::sin(psi) * dt * dt;
  next(2, 0) = v + a * dt;
  next(3, 0) = psi + v * kappa * dt + 0.5 * (a * kappa + v * dkappa) * dt * dt;
  next(4, 0) = kappa + dkappa * dt;
  return next;
}
} // namespace planning
}  // namespace ceshi