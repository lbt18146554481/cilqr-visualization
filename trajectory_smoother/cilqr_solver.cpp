
#include "cilqr_solver.h"
#include "../../single_frame/mock_headers/src/planning/common/log.h"
#include "../../single_frame/mock_headers/neodrive/common/config/common_config.h"
namespace ceshi {
namespace planning {
// 信息加载
PathCilqrSolverAML::PathCilqrSolverAML(
    const CilqrCarModel::PathModelState &ego_state,            // 车辆初始状态
    const std::vector<CilqrCarModel::OptVariables> &path_goal, // 参考线轨迹信息
    const std::vector<CilqrCarModel::OptVariables> &path_goal_la, // 历史轨迹信息
    const std::vector<double> &t_gap_seq, // t时间序列信息；
    const std::vector<CilqrCarModel::PieceBoundary>
        &piece_boundaries, // 应该是期望轨迹分成多段的s信息，左右边界
    const size_t &steps,                   // 总规划的点数
    const tk::spline &bound_left_l_spline, // 道路左右边界
    const tk::spline &bound_right_l_spline,
    std::vector<planning::CilqrCarModel::PieceBoundary> orin_boundaries
    ,const ReferenceLine& reference_line):reference_line_(&reference_line){
  //   const auto& path_cilqr_conf =
  //       config::PlanningConfig::Instance()->planning_research_config().path_cilqr;
  
  reset();
  ego_state_ = CilqrCarModel::PathModelState{
  ego_state.state_x, ego_state.state_y, ego_state.state_v,
  ego_state.state_heading, ego_state.state_kappa};
  steps_ = steps;
  path_goal_cur = path_goal;
  path_goal_last = path_goal_la;
  t_gap_seq_ = t_gap_seq;
  LOG_INFO(" t_seq_size={}", t_gap_seq.size());
  if (steps_ != path_goal.size()) {
    LOG_WARN("steps_ != path_goal.size(): steps_={}, path_goal.size()={}", steps_, path_goal.size());
}
if (!path_goal_last.empty() && steps_ != path_goal_last.size()) {
    LOG_WARN("steps_ != path_goal_last.size(): steps_={}, path_goal_last.size()={}", steps_, path_goal_last.size());
}
  dt = t_gap_seq[1] - t_gap_seq[0]; // 计算dt
  piece_boundaries_ = piece_boundaries;
  orin_boundaries_ = orin_boundaries;
  bound_left_l_spline_ = bound_left_l_spline;
  bound_right_l_spline_ = bound_right_l_spline;
  // 以下是原始程序中的相关数据，考虑适配可能用到，暂未删除

  // double df = 0.4;  // 安全冗余距离？
    auto& config =
        neodrive::common::config::CommonConfig::Instance()->ego_car_config();//车辆配置参数，可以替换成数值；
  width=config.width;
  length=config.length;

  dxys={
    {length / 2, width / 2},   // 前左角
    {0, width / 2} ,          // 左中点
    {-length / 2, width / 2},  // 后左角
    {length / 2, -width / 2},  // 前右角
    {-length / 2, -width / 2}, // 后右角
    {0, -width / 2},           // 右中点
};

  // double wheel_base=2.33;
  // double max_steer_angle=8.726646;
  // double steer_ratio=13.80;
  // double width=1.425;
  // double front_edge_to_center=1.84712;
  //   min_r_ = wheel_base /
  //            std::tan(max_steer_angle / steer_ratio);  // 2.5，最小转弯半径；
  //   r_ = std::sqrt(std::pow(df, 2) + std::pow((width / 2),
  //   2));//车辆的包络半径； c_f_ = front_edge_to_center - df;
  if (t_gap_seq.size() > 1) {
      oss_cilqr << ", t_gap_seq.size()=" << t_gap_seq.size()
          << ", steps=" << steps
          << ", dt=" << dt
          << ", width=" << width
          << ", length=" << length<<std::endl;
  }
  LOG_INFO("CILQR");
}

void PathCilqrSolverAML::reset() {
  ego_state_.SetZero();
  path_goal_.clear();
  path_goal_cur.clear();
path_goal_last.clear();
  t_gap_seq_.clear();
  orin_boundaries_.clear();
  piece_boundaries_.clear();
  is_illness_ = false;

}

PathCarModelOutput PathCilqrSolverAML::Optimize() {
  // 获取模型以及偏导数等等
  // 正式开始优化
  auto result = GetOptimalControlSeq(); // 开始优化
  if (!is_illness_ && CheckIllness(result.second)) {
    is_illness_ = true;
  }
  return result;
}

// 判断求解结果是否超出边界
bool PathCilqrSolverAML::CheckIllness(
    std::vector<CilqrCarModel::PathModelState>& ego_deduction_data) {
      return false;
  int error_cnt = 0;
  for (size_t i = 0; i < ego_deduction_data.size(); i++) {
    if(CheckKappaIllness(ego_deduction_data[i])){
      ++error_cnt;
    }
  }
  LOG_INFO("CILQR_Illness_num:{}",error_cnt);
  return error_cnt > kNoise;
}
// 判断kappa超出约束：
bool PathCilqrSolverAML::CheckKappaIllness(
    CilqrCarModel::PathModelState& ego_data){
      return ego_data.state_kappa>=origin_max_kappa||ego_data.state_kappa<=(-origin_max_kappa);
    }
//判断超出边界
bool PathCilqrSolverAML::CheckColIllness(
    CilqrCarModel::PathModelState& ego_data,double dt){
  bool result=false;
  
  double state_x = ego_data.state_x;
  double state_y = ego_data.state_y;
  double state_v = ego_data.state_v;
  double state_heading = ego_data.state_heading;
  double kappa = ego_data.state_kappa;
  // 下面是每个角点的边界约束
  double state_s = 0, state_l = 0, heading = 0;
  // 用于存储6个角点每个的[右边界剩余距离, 左边界剩余距离]
  std::vector<std::vector<double>> corner_constrains(
      max_num, std::vector<double>(2, 0.));
  // 遍历每4角点+两个左右中点，分别计算其对边界的约束剩余
  for (size_t i = 0; i < max_num; ++i) {
      double state_s = 0, state_l = 0, heading = 0;
      std::vector<double> dxy = dxys[i]; // 当前角点相对于车辆中心的坐标偏移

      // 旋转+平移，得到车辆在当前heading下，角点的全局坐标
      double state_x1 = state_x + std::cos(state_heading) * dxy[0] -
                        std::sin(state_heading) * dxy[1];
      double state_y1 = state_y + std::sin(state_heading) * dxy[0] +
                        std::cos(state_heading) * dxy[1];
      //根据坐标点x,y计算对应s的左右边界以及L和参考线的heading
      double left_bound=0,right_bound=0;
      calc_s_l_heading_from_reference(state_x1, state_y1, orin_boundaries_,
                                      state_s, state_l, heading,left_bound,right_bound,*reference_line_);


      // 右约束（距离右边界还有多少裕量）/左约束
      double right_leave = (right_bound) - state_l;
      double left_leave = state_l - left_bound;
      if(right_leave>0||left_leave>0){
        result=true;
        break;
      }
    }
    return result;
  }

void PathCilqrSolverAML::UpdateData(
    std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
    const std::pair<std::vector<CilqrCarModel::PathModelControl>,
                    std::vector<CilqrCarModel::X_state>> &forward_result) {
  rough_control_seq.clear();
  rough_state_seq.clear();
  for (size_t i = 0; i < forward_result.first.size(); i++) {
    CilqrCarModel::PathModelState ego_forward_state = {
        forward_result.second[i](0), forward_result.second[i](1),
        forward_result.second[i](2), forward_result.second[i](3),
        forward_result.second[i](4)};
    rough_state_seq.emplace_back(ego_forward_state);

    CilqrCarModel::PathModelControl control_res{forward_result.first[i]};
    rough_control_seq.emplace_back(control_res);
  }
  CilqrCarModel::PathModelState ego_forward_state{
      forward_result.second.back()(0), forward_result.second.back()(1),
      forward_result.second.back()(2), forward_result.second.back()(3),
      forward_result.second.back()(4)};
  rough_state_seq.emplace_back(ego_forward_state);
}

PathCarModelOutput PathCilqrSolverAML::GetOptimalControlSeq() {
  std::vector<CilqrCarModel::PathModelControl> rough_control_seq_cur{};
  rough_control_seq_cur.resize(steps_);
  // 两种方式得到初始解
  // oss_cilqr<<"Path_goal"<<std::endl;

  for (size_t i = 0; i < steps_; ++i) {
    rough_control_seq_cur[i].control_a = path_goal_cur[i].uk.control_a;
    rough_control_seq_cur[i].control_dkappa = path_goal_cur[i].uk.control_dkappa;
  }
  std::vector<CilqrCarModel::PathModelState> rough_state_seq_cur;
  std::transform(path_goal_cur.begin(), path_goal_cur.end(),
                 std::back_inserter(rough_state_seq_cur),
                 [](const CilqrCarModel::OptVariables& a) {
                   CilqrCarModel::PathModelState b;
                   b.state_x = a.xk.state_x;
                   b.state_y = a.xk.state_y;
                   b.state_v = a.xk.state_v;
                   b.state_heading = a.xk.state_heading;
                   b.state_kappa = a.xk.state_kappa;
                   return b;
                 });
     std::vector<CilqrCarModel::PathModelControl> rough_control_seq{};
  std::vector<CilqrCarModel::PathModelState> rough_state_seq{};

  std::vector<Eigen::Matrix<double, num_constrain, 1>> lambda_ALM,
      ck; // 定义拉格朗日乘子和约束计算值
  std::vector<Eigen::Matrix<double, num_constrain, num_constrain>>
      pho; // 定义惩罚矩阵

  double lambda = 5; // 病态矩阵使用
  double lambda_factor = 5;
  double lambda_decay = 0.7;
  double max_lambda = 600.0; //最大的lambda

  lambda_ALM.resize(steps_);
  pho.resize(steps_);
  ck.resize(steps_);
  //将拉格朗日乘子初始化为0;
  for (int i = 0; i < steps_; ++i) {
      lambda_ALM[i] = Eigen::Matrix<double, num_constrain, 1>::Constant(0.0); // 全0列向量
      pho[i] = Eigen::Matrix<double, num_constrain, num_constrain>::Zero();   // 全0矩阵
      ck[i] = Eigen::Matrix<double, num_constrain, 1>::Zero();                // 可选：初始化ck为全0
  }
  //赋值初始解和path_gaol；
  path_goal_=path_goal_cur;
  rough_control_seq=rough_control_seq_cur;
  rough_state_seq=rough_state_seq_cur;

// oss_cilqr << "==== rough_control_seq BEGIN ====\n";
// oss_cilqr << std::fixed << std::setprecision(6);
// for (size_t i = 0; i < rough_control_seq.size(); ++i) {
//     oss_cilqr << "[" << i << "]: a=" << rough_control_seq[i].control_a
//              << ", dkappa=" << rough_control_seq[i].control_dkappa << "\n";
// }
// oss_cilqr << "==== rough_control_seq END ====";
// oss_cilqr << "==== rough_state_seq BEGIN ====\n";
// oss_cilqr << std::fixed << std::setprecision(6);
// for (size_t i = 0; i < rough_state_seq.size(); ++i) {
//     const auto& s = rough_state_seq[i];
//     oss_cilqr << "[" << i << "]: x=" << s.state_x
//               << ", y=" << s.state_y
//               << ", v=" << s.state_v
//               << ", heading=" << s.state_heading
//               << ", kappa=" << s.state_kappa << "\n";
// }
// oss_cilqr << "==== rough_state_seq END ====";

  // const auto& path_cilqr_conf =
  //     config::PlanningConfig::Instance()->planning_research_config().path_cilqr;//cilqr的config，可以替换成数值
cilqr_model_ = std::make_shared<PathCilqrModelCar>(
      "cilqr_speed_model", rough_state_seq, rough_control_seq,
      dt);            // 根据初始状态得到线性模型
  double cost_old = std::numeric_limits<double>::max();
  cost_old =
      GetTotalCost(rough_state_seq, rough_control_seq, lambda_ALM, pho, ck);//计算总cost
  oss_cilqr<<"CILQR_origin_cost:"<<cost_old<<std::endl;
  int max_iter = 0;


  for (size_t iter = 0; iter < max_iters_; ++iter) {

    if (iter != 0) {
      //根据每次得到的名义轨迹重新生成线性模型
      cilqr_model_ = std::make_shared<PathCilqrModelCar>(
          "cilqr_speed_model", rough_state_seq, rough_control_seq, dt);
    }

    //backuppass过程,计算前馈项和反馈项
    std::pair<std::vector<CilqrCarModel::K>, std::vector<CilqrCarModel::k>>
        backward_result = BackwardPass(rough_state_seq, rough_control_seq,
                                       lambda, lambda_ALM, pho, ck);

    iter_effective_flag = false;
    std::pair<std::vector<CilqrCarModel::PathModelControl>,
              std::vector<CilqrCarModel::X_state>>
        forward_result;
    double cost_new = 1000000;
    //将backup相关结果保存用于线性搜索
    auto ck1 = ck;
    auto pho1 = pho;
    double best_alpha = 0.;
    //线性搜索
    oss_cilqr<<"Start_linear_search"<<std::endl;

    for (auto &alpha : alpha_options) {
      std::vector<CilqrCarModel::PathModelState> rough_state_seq1 = rough_state_seq;
      std::vector<CilqrCarModel::PathModelControl> rough_control_seq1 =
          rough_control_seq;

      forward_result = ForwardPass(rough_state_seq1, rough_control_seq1,
                                   backward_result, alpha);

      UpdateData(rough_state_seq1, rough_control_seq1, forward_result);

      cost_new = GetTotalCost(rough_state_seq1, rough_control_seq1, lambda_ALM,
                              pho1, ck1);

      oss_cilqr 
        << ",CILQR:delta_v" << delt_v
        << ":alpha:" << alpha 
        << ",cost_old:" << cost_old
        << ",cost_new:" << cost_new
        <<  std::endl;
          if (cost_new < cost_old) {
            best_alpha = alpha;
            iter_effective_flag = true;
            break;
          }

    }
    oss_cilqr<<",lambda:"<<lambda<<std::endl;

    //有效性判断
    if (iter_effective_flag) {
      ck = ck1;
      pho = pho1;
      UpdateData(rough_state_seq, rough_control_seq, forward_result);
      if (abs(cost_old - cost_new) < 0.02) {
        break;
      }
      cost_old = cost_new;
      lambda *= lambda_decay;
      UpdatePara(lambda_ALM, pho, ck, (int)iter);
    } else {
      lambda *= lambda_factor;
    }

    if (lambda > max_lambda) break;
    max_iter++;

  //   oss_cilqr 
  // << "[CILQR_iter] "
  // << "get_model " << t01 - t00 << " s, "
  // << "BackwardPass: " << t02 - t01 << " s, "
  // << "Backward_linears: " << t03 - t02 << " s, "
  // << "linearsh: " << t04 - t03 << " s, "
  // << "last " << t05 - t04 << " s, "
  // << std::endl;

  }
  if (max_iter == max_iters_) {
    is_illness_ = true;
  }

  // oss_cilqr << "==== final rough_control_seq BEGIN ====\n";
  // oss_cilqr << std::fixed << std::setprecision(6);
  // for (size_t i = 0; i < rough_control_seq.size(); ++i) {
  //     oss_cilqr << "[" << i << "]: a=" << rough_control_seq[i].control_a
  //             << ", dkappa=" << rough_control_seq[i].control_dkappa << "\n";
  // }
  // oss_cilqr << "==== final rough_control_seq END ====";
  // oss_cilqr << "==== fianl rough_state_seq BEGIN ====\n";
  // oss_cilqr << std::fixed << std::setprecision(6);
  // for (size_t i = 0; i < rough_state_seq.size(); ++i) {
  //     const auto& s = rough_state_seq[i];
  //     oss_cilqr << "[" << i << "]: x=" << s.state_x
  //               << ", y=" << s.state_y
  //               << ", v=" << s.state_v
  //               << ", heading=" << s.state_heading
  //               << ", kappa=" << s.state_kappa << "\n";
  // }
  // oss_cilqr << "==== rough_state_seq END ====";
  // oss_cilqr<<"Time_solver: rougg_state_control:"<<t1-t0<<",get_model_cost:"<<t2-t1<<
  // ",optimal: "<<t3-t2<<", print_final:"<<t4-t3;

  LOG_INFO("{}", oss_cilqr.str().c_str());
  return {rough_control_seq, rough_state_seq};
}

std::pair<std::vector<CilqrCarModel::PathModelControl>,
          std::vector<CilqrCarModel::X_state>>
PathCilqrSolverAML::ForwardPass(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
    const std::pair<std::vector<CilqrCarModel::K>, std::vector<CilqrCarModel::k>>
        &best_control_rate,
    double alpha) {

  std::vector<CilqrCarModel::X_state> ego_deduction_seq;
  std::vector<CilqrCarModel::X_state> old_state_seq;
  std::vector<CilqrCarModel::U_state> old_control_seq;
  std::vector<CilqrCarModel::PathModelControl> ego_control_seq;
ego_control_seq.reserve(steps_ - 1);
ego_deduction_seq.reserve(steps_);
old_state_seq.reserve(rough_state_seq.size());
old_control_seq.reserve(rough_control_seq.size());

  // 转换为矩阵形式的旧状态和控制量
  for (const auto &s : rough_state_seq) {
    CilqrCarModel::X_state x_state;
    x_state << s.state_x, s.state_y, s.state_v, s.state_heading, s.state_kappa;
    old_state_seq.emplace_back(x_state);
  }
  for (const auto &u : rough_control_seq) {
    CilqrCarModel::U_state u_state;
    u_state << u.control_a, u.control_dkappa;
    old_control_seq.emplace_back(u_state);
  }

  // 初始状态作为 forward 仿真的起点
  CilqrCarModel::X_state x_state_new = old_state_seq.front();
  ego_deduction_seq.emplace_back(x_state_new);
Eigen::Matrix<double, 2, 1> best_control;
  // 迭代构造控制和状态序列
  for (size_t i = 0; i < steps_ - 1; ++i) {

    // 正确地基于当前状态计算控制扰动
    best_control =
        old_control_seq[i] + alpha * best_control_rate.second[i] +
        best_control_rate.first[i] * (x_state_new - old_state_seq[i]);

    // 存储控制量
    CilqrCarModel::PathModelControl ego_control;
    ego_control.control_a = best_control(0, 0);
    ego_control.control_dkappa = best_control(1, 0);
    ego_control_seq.emplace_back(ego_control);

    // 推进至下一个状态
    x_state_new = cilqr_model_->ForwardStep(x_state_new, best_control);

    ego_deduction_seq.emplace_back(x_state_new);
  }

  return std::make_pair(ego_control_seq, ego_deduction_seq);
}

std::pair<std::vector<CilqrCarModel::K>, std::vector<CilqrCarModel::k>>
PathCilqrSolverAML::BackwardPass(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,
    const double &lambda,
    const std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,
    const std::vector<Eigen::Matrix<double, num_constrain, num_constrain>> &pho,
    const std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck) {
  // 获取梯度以及hessian矩阵

  CilqrCarModel::PathBackwardPerturbationPara cost_derivatives =
      GetCostDerivatives(rough_state_seq, rough_control_seq);

  //计算约束的梯度

  auto ck_x_u =
      GetConstrainDre(rough_state_seq, rough_control_seq);

  CilqrCarModel::V_x V_x = cost_derivatives.l_x.back();
  CilqrCarModel::V_xx V_xx = cost_derivatives.l_xx.back();
  auto cnx = ck_x_u.first.back();

  //total_cost计算中对约束做了正投影，相应的约束需要取消；
  Eigen::Matrix<double, num_constrain, 5> masked_ckx;
  Eigen::Matrix<double, num_constrain, 2> masked_cku;
  masked_ckx.setZero();
  masked_cku.setZero();
  for (int j = 0; j < num_constrain; ++j) {
    if (ck.back()(j, 0) > 0) {
      masked_ckx.row(j) = ck_x_u.first.back().row(j);
      masked_cku.row(j) = ck_x_u.second.back().row(j);
    }
    // else 留为全零
  }
  //开始BackUp
  V_x = V_x + masked_ckx.transpose() * pho.back() * ck.back() +
        masked_ckx.transpose() * lambda_ALM.back();
  V_xx = V_xx + masked_ckx.transpose() * pho.back() * masked_ckx;


  std::vector<CilqrCarModel::k> k_seq;
  k_seq.resize(steps_ - 1);
  std::vector<CilqrCarModel::K> K_seq;
  K_seq.resize(steps_ - 1);
  delt_v = 0;

  CilqrCarModel::Q_x Q_x;
  CilqrCarModel::Q_u Q_u;
  CilqrCarModel::Q_xx Q_xx;
  CilqrCarModel::Q_ux Q_ux;
  CilqrCarModel::Q_uu Q_uu;
  for (int i = (steps_ - 2); i >= 0; i--) {
     


    masked_ckx.setZero();
    masked_cku.setZero();
    for (int j = 0; j < num_constrain; ++j) {
      if (ck[i](j, 0) > 0) {
        masked_ckx.row(j) = ck_x_u.first[i].row(j);
        masked_cku.row(j) = ck_x_u.second[i].row(j);
      }
      // else 留为全零
    }
    // back ward
    auto& A = cilqr_model_->error_path_model_matrix()[i].A;
    auto& B = cilqr_model_->error_path_model_matrix()[i].B;
    auto AT=A.transpose();
    auto BT=B.transpose();
    auto masked_ckxT=masked_ckx.transpose();
    auto masked_ckuT=masked_cku.transpose();
    auto& ci = ck[i];
    auto& I = pho[i];
    auto& lambda_AL = lambda_ALM[i];
    auto masked_ckxT_I=masked_ckxT * I;
    auto masked_ckuT_I=masked_ckuT * I;
    auto V_xx_A=V_xx *A;



    auto 
    Q_x =
        cost_derivatives.l_x[i] +
        AT * V_x +
        masked_ckxT_I * ci + masked_ckxT * lambda_AL;
    Q_u =
        cost_derivatives.l_u[i] +
        BT * V_x +
        masked_ckuT_I * ci + masked_ckuT * lambda_AL;
   Q_xx =
        cost_derivatives.l_xx[i] +
        AT *  V_xx_A +
        masked_ckxT_I * masked_ckx;
    Q_ux =
        cost_derivatives.l_ux[i] +BT *  V_xx_A +
        masked_ckuT_I * masked_ckx;
    Q_uu =
        cost_derivatives.l_uu[i] +BT * V_xx *B +
        masked_ckuT_I * masked_cku;


    // 在计算逆之前，给 Hessian 矩阵加上 λI
    CilqrCarModel::Q_uu Q_uu1 = Q_uu + lambda * Eigen::Matrix2d::Identity();

    CilqrCarModel::Q_uu Q_uu_inv = fastInverse2x2(Q_uu1);
    k_seq[i] = -Q_uu_inv * Q_u;
    K_seq[i] = -Q_uu_inv * Q_ux;
    auto K_seqiT=K_seq[i].transpose();
    auto Q_uxT=Q_ux.transpose();
    // update V_x V_xx
    V_x = Q_x + K_seqiT * Q_uu * k_seq[i] +
          K_seqiT * Q_u + Q_uxT * k_seq[i];
    V_xx = Q_xx + K_seqiT * Q_uu * K_seq[i] +
           K_seqiT * Q_ux + Q_uxT * K_seq[i];

    // delt_v = delt_v + (0.5 * k_seq[i].transpose() * Q_uu1 * k_seq[i] +
    //                    k_seq[i].transpose() * Q_u)(0, 0);//计算理论的线性cost降低预期
    // oss_cilqr << "[BackwardPass-Step:" << i << "]"
    //   << " part1: " << t_before_Q - t_step_start << " s,"
    //   << " part2: " << t_before_inv - t_before_Q << " s,"
    //   << " part3: " << t_step_end - t_before_inv << " s,"
    //   << " total: " << t_step_end - t_step_start << " s\n";
  }
  // std::cout << "delt_v:" << delt_v << std::endl;
  // oss_cilqr 
  //   << "[BackwardPass] "
  //   << "GetCostDerivatives: " << t1 - t0 << " s, "
  //   << "GetConstrainDre: " << t2 - t1 << " s, "
  //   << "BackUp: " << t3 - t2 << " s, "
  //   << "update_seq " << t5_loop_end - t4_loop_start << " s, "
  //   << std::endl;
  return std::make_pair(K_seq, k_seq);
}

CilqrCarModel::PathBackwardPerturbationPara PathCilqrSolverAML::GetCostDerivatives(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq) {
  // 返回针对控制输入部分的cost以及约束的偏导数和hessian矩阵，整个序列的值，其中cost只用了二次代价函数，约束为最大最小约束；
  auto control_derivative =
      GetControlCostDerivatives(rough_state_seq, rough_control_seq);
  // 返回对状态量的cost以及约束的偏导数和hessian矩阵；

  auto state_derivative =
      GetStateCostDerivatives(rough_state_seq, rough_control_seq);

  CilqrCarModel::l_ux l_ux;
  l_ux.setZero();
  std::vector<CilqrCarModel::l_ux> l_ux_seq(steps_ - 1, l_ux);

  CilqrCarModel::PathBackwardPerturbationPara backward_perturbation_para{
      state_derivative.l_x, state_derivative.l_xx, control_derivative.l_u,
      control_derivative.l_uu, l_ux_seq};
  return backward_perturbation_para;
}

CilqrCarModel::PathBackwardPerturbationParaState
PathCilqrSolverAML::GetStateCostDerivatives(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq) {
  // const auto& path_cilqr_conf =
  //     config::PlanningConfig::Instance()->planning_research_config().path_cilqr;
  std::vector<CilqrCarModel::l_x> l_x_seq{};
  std::vector<CilqrCarModel::l_xx> l_xx_seq{};

  l_x_seq.clear();
  l_xx_seq.clear();
  l_x_seq.resize(steps_);
  l_xx_seq.resize(steps_);
  Eigen::Matrix<double, 5, 1> Pka;
  CilqrCarModel::l_x l_x;
  CilqrCarModel::l_xx l_xx;
  CilqrCarModel::X_state state_error_vector;

  for (size_t i = 0; i < steps_ - 1; i++) {
    double goal_x = path_goal_[i].xk.state_x;
    double goal_y = path_goal_[i].xk.state_y;
    double goal_v = path_goal_[i].xk.state_v;
    double goal_heading = path_goal_[i].xk.state_heading;
    double goal_kappa = path_goal_[i].xk.state_kappa;

    double state_x = rough_state_seq[i].state_x;
    double state_y = rough_state_seq[i].state_y;
    double state_v = rough_state_seq[i].state_v;
    double state_heading = rough_state_seq[i].state_heading;
    double state_kappa = rough_state_seq[i].state_kappa;

    state_error_vector(0, 0) = state_x - goal_x;
    state_error_vector(1, 0) = state_y - goal_y;
    state_error_vector(2, 0) = state_v - goal_v;
    state_error_vector(3, 0) = state_heading - goal_heading;
    state_error_vector(4, 0) = state_kappa - goal_kappa;

    // 1.最小二乘代价
    l_x = ref_x_weight * 2 * cilqr_model_->path_cost_para().state_cost_matrix *
          state_error_vector;
    l_xx = ref_x_weight * 2 * cilqr_model_->path_cost_para().state_cost_matrix;
    l_x_seq[i] = l_x;
    l_xx_seq[i] = l_xx;
  }

  int i = steps_ - 1;
  double goal_x = path_goal_[i].xk.state_x;
  double goal_y = path_goal_[i].xk.state_y;
  double goal_v = path_goal_[i].xk.state_v;
  double goal_heading = path_goal_[i].xk.state_heading;
  double goal_kappa = path_goal_[i].xk.state_kappa;
  double state_x = rough_state_seq[i].state_x;
  double state_y = rough_state_seq[i].state_y;
  double state_v = rough_state_seq[i].state_v;
  double state_heading = rough_state_seq[i].state_heading;
  double state_kappa = rough_state_seq[i].state_kappa;
  state_error_vector(0, 0) = state_x - goal_x;
  state_error_vector(1, 0) = state_y - goal_y;
  state_error_vector(2, 0) = state_v - goal_v;
  state_error_vector(3, 0) = state_heading - goal_heading;
  state_error_vector(4, 0) = state_kappa - goal_kappa;
  // 1.最小二乘代价
  l_x = final_x_weight * 2 * cilqr_model_->path_cost_para().state_cost_matrix *
        state_error_vector;
  l_xx = final_x_weight * 2 * cilqr_model_->path_cost_para().state_cost_matrix;

  l_x_seq[i] = l_x;
  l_xx_seq[i] = l_xx;
  CilqrCarModel::PathBackwardPerturbationParaState state_perturbation_result;
  state_perturbation_result.l_x = l_x_seq;
  state_perturbation_result.l_xx = l_xx_seq;

  return state_perturbation_result;
}
// // 计算路径CILQR的总代价函数，包括状态、控制输入以及约束（增广拉格朗日约束）相关项
double PathCilqrSolverAML::GetTotalCost(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,      // 输入：状态序列
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq,  // 输入：控制量序列
    std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,   // 输入/输出：增广拉格朗日乘子
    std::vector<Eigen::Matrix<double, num_constrain, num_constrain>> &pho, // 输入/输出：惩罚因子矩阵
    std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck)           // 输入/输出：约束项正部分
{
  double total_cost = 0.0;  // 总代价
  double state_cost = 0, constrain_ = 0; // 状态、控制、约束相关代价

  double total_constrain = 0, ter_constrain = 0, total_state = 0, ter_state = 0;
std::ostringstream oss;
// oss<<"Uki:"<<uki;
  // 主循环，对每一个时间步（除了最后一步）进行代价累加
  auto &Q=cilqr_model_->path_cost_para().state_cost_matrix;
  


  for (size_t i = 0; i < steps_ - 1; i++) {
    // 计算当前步状态与目标状态之间的差异
    Eigen::Matrix<double, 5, 1> state_diff;
    state_diff << rough_state_seq[i].state_x - path_goal_[i].xk.state_x,
              rough_state_seq[i].state_y - path_goal_[i].xk.state_y,
              rough_state_seq[i].state_v - path_goal_[i].xk.state_v,
              rough_state_seq[i].state_heading - path_goal_[i].xk.state_heading,
              rough_state_seq[i].state_kappa - path_goal_[i].xk.state_kappa;


    // 1. 状态项代价（权重 × 二次型）
    double c_state = ref_x_weight * state_diff.transpose() *
                    Q*state_diff;

    // 2. 控制项代价（两个控制输入分别加权）
    double c_ctrl =
        rough_control_seq[i].control_a*rough_control_seq[i].control_a * control_a_cost +
        rough_control_seq[i].control_dkappa*
        rough_control_seq[i].control_dkappa * control_dkappa_cost;

    // 3. 约束项
    Eigen::Matrix<double, num_constrain, 1> cki =
        GetConstrain(rough_state_seq[i], rough_control_seq[i], i);



    // 动态调整惩罚因子
    for (size_t j = 0; j < num_constrain; ++j) {
      if (cki(j, 0) >= 0 || lambda_ALM[i](j, 0) > 0) {
        pho[i](j, j) = uki;
      } else {
        pho[i](j, j) = 0;
      }
    }

    // 提取约束残差正部分
    Eigen::Matrix<double, num_constrain, 1> cki_pos = cki.cwiseMax(0.0);

    ck[i] = cki_pos;

    // 增广拉格朗日项
    auto constrain = lambda_ALM[i].transpose() * cki_pos +
                     cki_pos.transpose() * pho[i] * cki_pos;
    total_state += c_state + c_ctrl;
    total_constrain += constrain.value();

    // 累加总代价
    total_cost = total_cost + c_state + c_ctrl + constrain.value();
    // // 汇总到oss
    // oss << "Step: " << i
    //     << ": constrain.value()=" << constrain.value()<< std::endl;
  }

  // 末步（终点）状态只加终点状态cost和终点约束

  size_t i = steps_ - 1;
  Eigen::Matrix<double, 5, 1> state_diff;
    state_diff << rough_state_seq[i].state_x - path_goal_[i].xk.state_x,
              rough_state_seq[i].state_y - path_goal_[i].xk.state_y,
              rough_state_seq[i].state_v - path_goal_[i].xk.state_v,
              rough_state_seq[i].state_heading - path_goal_[i].xk.state_heading,
              rough_state_seq[i].state_kappa - path_goal_[i].xk.state_kappa;

  // 末步状态项
  double c_state = final_x_weight * state_diff.transpose() *
                   Q *state_diff;


  // 末步默认不加控制项
  CilqrCarModel::PathModelControl last;
  last.control_a = 0;
  last.control_dkappa = 0;
  auto cki = GetConstrain(rough_state_seq[i], last, i);
  ck[i] = cki;

  // 终点步也动态调整惩罚参数
  for (size_t j = 0; j < num_constrain; ++j) {
    if (cki(j, 0) >= 0 || lambda_ALM[i](j, 0) > 0) {
      pho[i](j, j) = uki;
    } else {
      pho[i](j, j) = 0;
    }
  }
  // 只加正部分
  Eigen::Matrix<double, num_constrain, 1> cki_pos = cki.cwiseMax(0.0);
  auto constrain = lambda_ALM[i].transpose() * cki_pos +
                   cki_pos.transpose() * pho[i] * cki_pos;

  ter_state = c_state;
  ter_constrain = constrain.value();
  total_cost += c_state + constrain.value();

  // oss << "FINAL: constrain.value()=" << constrain.value()<< std::endl;

  // // 打印累计信息
  // oss << "[Summary] total_constrain=" << total_constrain
  //     << ", ter_constrain=" << ter_constrain
  //     << ", total_state=" << total_state
  //     << ", ter_state=" << ter_state << std::endl;
  // LOG_INFO("Total_cost:{}",oss.str().c_str());
  // oss_cilqr<<"total_time_get_constrain: "<<t_constrain_sum<<std::endl;
  return total_cost;
}



CilqrCarModel::PathBackwardPerturbationParaControl
PathCilqrSolverAML::GetControlCostDerivatives(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq) {
  // const auto& path_cilqr_conf =
  //     config::PlanningConfig::Instance()->planning_research_config().path_cilqr;//cilqr的配置参数
  std::vector<CilqrCarModel::l_u> l_u_seq{};
  std::vector<CilqrCarModel::l_uu> l_uu_seq{};
  l_u_seq.resize(steps_ - 1);
  l_uu_seq.resize(steps_ - 1);
  CilqrCarModel::l_u l_u;
  CilqrCarModel::l_uu l_uu;
  double P1 = 1;
  for (size_t i = 0; i < steps_ - 1; i++) {
    // 1.最小二乘代价
    l_u << 2 * rough_control_seq[i].control_a * control_a_cost,
        2 * rough_control_seq[i].control_dkappa * control_dkappa_cost;
    l_uu << 2 * control_a_cost, 0, 0,
        2 * control_dkappa_cost; // 这里的cost应该是权重可以使用数值代替；

    l_u_seq[i] = l_u;
    l_uu_seq[i] = l_uu;
  }
  CilqrCarModel::PathBackwardPerturbationParaControl control_perturbation_result;
  control_perturbation_result.l_u = l_u_seq;
  control_perturbation_result.l_uu = l_uu_seq;
  return control_perturbation_result;
}
//根据坐标点x,y计算对应s的左右边界以及L和参考线的heading
inline void PathCilqrSolverAML::calc_s_l_heading_from_reference(
    double x, double y, const std::vector<CilqrCarModel::PieceBoundary> &ref_path,
    double &s_out, double &l_out, double &heading_out,double &left_bound,double &right_bound,
    const ReferenceLine& reference_line_) {
  // if (ref_path.size() < 2)
  //   return;
  // double min_dist = std::numeric_limits<double>::max();
  // size_t best_i = 0;
  // double best_t = 0.0;
  // for (size_t i = 0; i < ref_path.size() - 1; ++i) {
  //   const auto &p0 = ref_path[i];
  //   const auto &p1 = ref_path[i + 1];
  //   double dx = p1.x - p0.x;
  //   double dy = p1.y - p0.y;
  //   double seg_len2 = dx * dx + dy * dy;
  //   if (seg_len2 < 1e-8)
  //     continue;
  //   double t = ((x - p0.x) * dx + (y - p0.y) * dy) / seg_len2;
  //   t = std::max(0.0, std::min(1.0, t));
  //   double proj_x = p0.x + t * dx;
  //   double proj_y = p0.y + t * dy;
  //   double dist2 = (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y);
  //   if (dist2 < min_dist) {
  //     min_dist = dist2;
  //     s_out = p0.s + t * (p1.s - p0.s);
  //     // 法向量
  //     double nx = -dy, ny = dx;
  //     double norm = std::sqrt(nx * nx + ny * ny);
  //     if (norm < 1e-8)
  //       norm = 1.0;
  //     nx /= norm;
  //     ny /= norm;
  //     double vx = x - proj_x, vy = y - proj_y;
  //     l_out = vx * nx + vy * ny;
  //     best_i = i;
  //     best_t = t;
  //   }
  // }
  // // 计算heading
  // if (ref_path.size() < 2) {
  //   heading_out = 0.0;
  // } else {
  //   const auto &p0 = ref_path[best_i];
  //   const auto &p1 = ref_path[best_i + 1];
  //   double dx = p1.x - p0.x;
  //   double dy = p1.y - p0.y;
  //   heading_out = std::atan2(dy, dx); // heading = atan2(dy, dx)
  // }

    Vec2d xypt(x,y);
    ReferencePoint sl_pt;
    reference_line_.GetNearestRefPoint(xypt,&sl_pt);
    heading_out =sl_pt.heading();
    double dx = x - sl_pt.x();
    double dy = y - sl_pt.y();
    l_out=-sin(heading_out) * dx + cos(heading_out) * dy;
    s_out=sl_pt.s();
    left_bound=sl_pt.left_bound();
    right_bound=-sl_pt.right_bound();
    
}

// // 计算当前状态和控制下的所有约束残差（ck），包括加速度、曲率、角点边界等约束
Eigen::Matrix<double, num_constrain, 1>
PathCilqrSolverAML::GetConstrain(const CilqrCarModel::PathModelState &rough_state,
                              const CilqrCarModel::PathModelControl &rough_control,
                              size_t index) {
  Eigen::Matrix<double, num_constrain, 1> ck; // 最终返回的约束残差
  // 读取状态分量
  ck.setZero();
  double state_x = rough_state.state_x;
  double state_y = rough_state.state_y;
  double state_v = rough_state.state_v;
  double state_heading = rough_state.state_heading;
  double kappa = rough_state.state_kappa;
  double a = rough_control.control_a;
  double dk = rough_control.control_dkappa;


  // // 用于存储6个角点每个的[右边界剩余距离, 左边界剩余距离]
  // std::vector<double> corner_constrains(
  //     max_num);
  // // 遍历每4角点+两个左右中点，分别计算其对边界的约束剩余
  // // std::ostringstream oss;
  // // oss << std::fixed << std::setprecision(4);  // 可调整精度
  // // oss << "==== All corner constraints ====\n";
  // double cosheading=std::cos(state_heading);
  // double sinheading=std::sin(state_heading);
  // double state_s = 0, state_l = 0, heading = 0;
  // for (size_t i = 0; i < max_num; ++i) {
  //     auto &dxy= dxys[i];

  //     double state_x1 = state_x + cosheading * dxy[0] -
  //                       sinheading * dxy[1];
  //     double state_y1 = state_y + sinheading * dxy[0] +
  //                       cosheading * dxy[1];
  //     double left_bound=0,right_bound=0;
  //     calc_s_l_heading_from_reference(state_x1, state_y1, orin_boundaries_,
  //                                     state_s, state_l, heading,left_bound,right_bound,reference_line_);
  //     // 查找该s位置的左右边界极限
  //     // double left_bound = bound_left_l_spline_(state_s);
  //     // double right_bound = bound_right_l_spline_(state_s);
  //     // LOG_INFO("s={},left_bound1={}, right_bound1={}, left_bound(spline)={}, right_bound(spline)={}",
  //     //    state_s, left_bound1, right_bound1, left_bound, right_bound);
  //     // 右约束（距离右边界还有多少裕量）/左约束


  //     // corner_constrain[0] = (right_bound) - state_l + safe_bound;
  //     // corner_constrain[1] = state_l - left_bound + safe_bound;

  //     if(i<max_num/2){
  //       corner_constrains[i] =  state_l - left_bound + safe_bound;
  //     }else{
  //       corner_constrains[i] =(right_bound) - state_l + safe_bound;
  //     }

  //     // oss << "Corner[" << i << "]: "
  //     //     << "state_x=" << state_x << ", state_y=" << state_y
  //     //     << "; x1=" << state_x1 << ", y1=" << state_y1
  //     //     << "; S=" << state_s
  //     //     << ", L=" << state_l
  //     //     << ", LEFT=" << left_bound
  //     //     << ", RIGHT=" << right_bound
  //     //     << ", right_constrain=" << corner_constrain[0]
  //     //     << ", left_constrain=" << corner_constrain[1]
  //     //     << "\n";
  // }
  // // oss << "==== End corner constraints ====";

  // // // 一次性打印全部角点信息
  // // LOG_INFO("{}", oss.str());



  // 车辆动力学及物理约束
  double amax = a - max_acc;                // 加速度上限
  double amin = (min_acc) - a;             // 加速度下限
  double kappamax = kappa - max_kappa;      // 曲率上限
  double kappamin = (-max_kappa) - kappa;   // 曲率下限
  double dkappamax = dk - max_kappa;        // 曲率变化率上限
  double dkappamin = (-max_kappa) - dk;     // 曲率变化率下限

  // 区分是否为末步，末步不再约束加速度，但曲率类约束依然存在
  if (index != steps_ - 1) {
    ck << amax, amin, kappamax, kappamin, dkappamax, dkappamin;
  } else {
    // 末步，将加速度相关约束置为-1（等价于不约束），只保留曲率与角点
    ck << -1, -1, kappamax, kappamin, dkappamax, dkappamin;
  }
  return ck; // 输出所有约束残差组成的列向量
}


std::pair<std::vector<MatCX>, std::vector<MatCU>>
PathCilqrSolverAML::GetConstrainDre(
    const std::vector<CilqrCarModel::PathModelState> &rough_state_seq,
    const std::vector<CilqrCarModel::PathModelControl> &rough_control_seq) {
  std::vector<MatCX> cx(steps_);  // 直接分配好每个元素
  std::vector<MatCU> cu(steps_ - 1); // 最后一步无输入

  for (size_t i = 0; i < steps_ - 1; ++i) {
    MatCX& ckx = cx[i];
    MatCU& cku = cu[i];
    ckx.setZero();
    cku.setZero();
    // === 前4个约束 ===
    // 对状态
    ckx(2, 4) = 1;  // kappa
    ckx(3, 4) = -1; // -kappa
    // 对控制
    cku(0, 0) = 1;   // a
    cku(1, 0) = -1;  // -a
    cku(2, 1) = dt;  // dkappa, kappa_{k+1} = kappa + dt*dkappa
    cku(3, 1) = -dt; // -dkappa
    cku(4, 1) = 1;
    cku(5, 1) = -1;
    // const double &state_x = rough_state_seq[i].state_x;
    // const double &state_y = rough_state_seq[i].state_y;
    // const double &state_v = rough_state_seq[i].state_v;
    // const double &state_heading = rough_state_seq[i].state_heading;
    // const double &kappa = rough_state_seq[i].state_kappa;
    // const double &a = rough_control_seq[i].control_a;
    // double state_s = 0, state_l = 0, heading = 0;
    // // === 后8个约束 ===
    // // 对每个角点，填充边界约束的梯度
    // for (int corner = 0; corner < max_num; ++corner) {
    //   // 提取角点信息
    //   auto &dxy = dxys[corner]; // std::vector<double> {dx, dy}
    //   double dx_i = dxy[0], dy_i = dxy[1];
    //   double state_x1 = state_x + std::cos(state_heading) * dxy[0] -
    //                     std::sin(state_heading) * dxy[1];
    //   double state_y1 = state_y + std::sin(state_heading) * dxy[0] +
    //                     std::cos(state_heading) * dxy[1];
      
    //   double left_bound1=0,right_bound1=0;
    //   calc_s_l_heading_from_reference(state_x1, state_y1, orin_boundaries_,
    //                                   state_s, state_l, heading,left_bound1,right_bound1,reference_line_);

    //   // 这里假设calc_s_l_heading_from_reference已计算了theta_r
    //   double theta_r = heading; 
    //   double psi = state_heading;
    //   // 如果可直接获得theta_r，替换此处
    //   // 若没有，可传递额外参数或通过成员变量获得

    //   // 推导雅可比
    //   double dldx = -sin(theta_r);
    //   double dldy = cos(theta_r);
    //   double dldv = 0;
    //   double dldpsi = dx_i * sin(psi - theta_r) + dy_i * cos(psi - theta_r);
    //   double dldkappa = 0;

    //   // // 右侧约束梯度
    //   // int row_r = 6 + 2 * corner; // 第4+2*corner行
    //   // ckx(row_r, 0) = -dldx;
    //   // ckx(row_r, 1) = -dldy;
    //   // ckx(row_r, 2) = -dldv;
    //   // ckx(row_r, 3) = -dldpsi;
    //   // ckx(row_r, 4) = -dldkappa;

    //   // // 左侧约束梯度
    //   // int row_l = 6 + 2 * corner + 1; // 第4+2*corner+1行
    //   // ckx(row_l, 0) = dldx;
    //   // ckx(row_l, 1) = dldy;
    //   // ckx(row_l, 2) = dldv;
    //   // ckx(row_l, 3) = dldpsi;
    //   // ckx(row_l, 4) = dldkappa;
    //   if(corner<max_num/2){
    //     int row_l = 6 + corner; // 第4+2*corner+1行
    //     ckx(row_l, 0) = dldx;
    //     ckx(row_l, 1) = dldy;
    //     ckx(row_l, 2) = dldv;
    //     ckx(row_l, 3) = dldpsi;
    //     ckx(row_l, 4) = dldkappa;
    //   }else{
    //           // 右侧约束梯度
    //     int row_r = 6 + corner; // 第4+2*corner行
    //     ckx(row_r, 0) = -dldx;
    //     ckx(row_r, 1) = -dldy;
    //     ckx(row_r, 2) = -dldv;
    //     ckx(row_r, 3) = -dldpsi;
    //     ckx(row_r, 4) = -dldkappa;
    //   }
    //   // 对控制量
    //   // cu 后8行为 0
    // }
  }

  // 末步边界（可与倒数第二步相同，或自定义）
  size_t i = steps_ - 1;
  MatCX& ckx = cx[i];
  // 填法同上
  ckx(2, 4) = 1;  // kappa
  ckx(3, 4) = -1; // -kappa
  // double state_x = rough_state_seq[i].state_x;
  // double state_y = rough_state_seq[i].state_y;
  // double state_v = rough_state_seq[i].state_v;
  // double state_heading = rough_state_seq[i].state_heading;
  // double kappa = rough_state_seq[i].state_kappa;
  // double state_s = 0, state_l = 0, heading = 0;
  // // === 后8个约束 ===
  // // 对每个角点，填充边界约束的梯度
  // for (int corner = 0; corner < max_num; ++corner) {
  //   // 提取角点信息
  //   auto &dxy = dxys[corner]; // std::vector<double> {dx, dy}
  //   double dx_i = dxy[0], dy_i = dxy[1];
  //   double state_x1 = state_x + std::cos(state_heading) * dxy[0] -
  //                     std::sin(state_heading) * dxy[1];
  //   double state_y1 = state_y + std::sin(state_heading) * dxy[0] +
  //                     std::cos(state_heading) * dxy[1];
  //   double left_bound1=0,right_bound1=0;
  //     calc_s_l_heading_from_reference(state_x1, state_y1, orin_boundaries_,
  //                                     state_s, state_l, heading,left_bound1,right_bound1,reference_line_);
  //   // 这里假设calc_s_l_heading_from_reference已计算了theta_r
  //   double theta_r = heading; //参考轨迹heading
  //   double psi = state_heading;
  //   // 推导雅可比

  //   double dldx = -sin(theta_r);
  //   double dldy = cos(theta_r);
  //   double dldv = 0;
  //   double dldpsi = dx_i * sin(psi - theta_r) + dy_i * cos(psi - theta_r);
  //   double dldkappa = 0;

  //   // // 右侧约束梯度
  //   // int row_r = 6 + 2 * corner; // 第4+2*corner行
  //   // ckx(row_r, 0) = -dldx;
  //   // ckx(row_r, 1) = -dldy;
  //   // ckx(row_r, 2) = -dldv;
  //   // ckx(row_r, 3) = -dldpsi;
  //   // ckx(row_r, 4) = -dldkappa;

  //   // // 左侧约束梯度
  //   // int row_l = 6 + 2 * corner + 1; // 第4+2*corner+1行
  //   // ckx(row_l, 0) = dldx;
  //   // ckx(row_l, 1) = dldy;
  //   // ckx(row_l, 2) = dldv;
  //   // ckx(row_l, 3) = dldpsi;
  //   // ckx(row_l, 4) = dldkappa;
  //   if(corner<max_num/2){
  //       int row_l = 6 + corner; // 第4+2*corner+1行
  //       ckx(row_l, 0) = dldx;
  //       ckx(row_l, 1) = dldy;
  //       ckx(row_l, 2) = dldv;
  //       ckx(row_l, 3) = dldpsi;
  //       ckx(row_l, 4) = dldkappa;
  //     }else{
  //             // 右侧约束梯度
  //       int row_r = 6 + corner; // 第4+2*corner行
  //       ckx(row_r, 0) = -dldx;
  //       ckx(row_r, 1) = -dldy;
  //       ckx(row_r, 2) = -dldv;
  //       ckx(row_r, 3) = -dldpsi;
  //       ckx(row_r, 4) = -dldkappa;
  //     }
  //   // 对控制量
  //   // cu 后8行为 0
  // }
  return std::make_pair(cx, cu);
}

void PathCilqrSolverAML::UpdatePara(
    std::vector<Eigen::Matrix<double, num_constrain, 1>> &lambda_ALM,
    std::vector<Eigen::Matrix<double, num_constrain, num_constrain>> &pho,
    std::vector<Eigen::Matrix<double, num_constrain, 1>> &ck, int m) {
  for (size_t i = 0; i < lambda_ALM.size(); ++i) {
    Eigen::Matrix<double, num_constrain, 1> lambda_next =
        lambda_ALM[i] + pho[i] * ck[i];
    for (size_t j = 0; j < num_constrain; ++j) {
      lambda_next(j, 0) = std::max(0., lambda_next(j, 0));
    }
    lambda_ALM[i] = lambda_next;
  }
  // m=std::min(m,15);
  uki = uki * 2;
}

} // namespace planning
}  // namespace ceshi