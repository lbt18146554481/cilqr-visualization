#pragma once
#include "cilqr_model.h"
#include "cilqr_solver.h"
#include "planning/planning_map/planning_map.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/common/trajectory/publishable_trajectory.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/optimizers/path_constrained_iter_lqr_planner/path_cilqr_solver.h"
#include "src/planning/task/optimizers/third_order_spline_path_optimizer/third_order_spline_path_model.h"

namespace ceshi {
namespace planning {

class PathConstrainedIterLqrDeciderAML {
  // DECLARE_SINGLETON(PathConstrainedIterLqrDeciderAML);

 public:
  PathConstrainedIterLqrDeciderAML(std::string name);
  ~PathConstrainedIterLqrDeciderAML();
  ErrorCode Execute(const ThirdOrderSplinePath::State& init_state,
                    const InsidePlannerData& inside_data,
                    const std::vector<TrajectoryPoint>& rough_trajectory,
                    const std::vector<double>& knots_delta_s,
                    const std::vector<PieceBoundary>& piece_boundaries,
                    const std::vector<PieceBoundary>& orin_boundaries,
                    const ReferenceLine& reference_line);
  void SaveTaskResults(TaskInfo& task_info);
  void Reset();
  void SaveLOGResults();
  const std::vector<CilqrCarModel::PathModelState> GetOptSolution();
  const std::vector<TrajectoryPoint> GetComputedTrajectoryPoints();
  void TranslateSLtoCartesian(
      const std::vector<double>& knots_delta_s,
      const std::vector<ThirdOrderSplinePath::OptVariables>&
          opt_path_variables_fre,
      std::vector<CilqrCarModel::OptVariables>& opt_path_variables_car,
      const ReferenceLine& reference_line,
      const InsidePlannerData& inside_data);
  double calc_curvature(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3);
  void ConvertXYSeqToOptVariables(
      const std::vector<TrajectoryPoint>& rough_trajectory,
      std::vector<double>& time_seq,  // 必须和 xy_seq 同长度
      std::vector<CilqrCarModel::OptVariables>& opt_path_variables_car);
  PathModelOutput TranslateCartesiantoSL(
      const std::vector<double>& knots_delta_s,
      const PathCarModelOutput& path_car_model_output,
      const ReferenceLine& reference_line);

  bool get_xystate_seqs(std::vector<TrajectoryPoint>& rough_trajectory);

 private:
  bool Init();
  bool Process();
  void MemberReset();

 private:
  double dt = 0.2;
  int total_num_init = 30;
  double total_s = 0;
  std::vector<double> t_seq;
  const ReferenceLine* reference_line_;
  std::vector<CilqrCarModel::PieceBoundary> orin_boundaries_car{};
  std::vector<CilqrCarModel::PieceBoundary> orin_boundaries_car_sample;
  std::string name_{""};
  std::vector<ThirdOrderSplinePath::OptVariables> opt_path_variables_{};
  std::vector<CilqrCarModel::OptVariables> opt_path_variables_car{};
  std::vector<CilqrCarModel::OptVariables> path_goal_{};
  std::vector<CilqrCarModel::OptVariables> opt_path_variables_car_last{};
  std::vector<double> knots_delta_s_{};
  std::vector<PieceBoundary> piece_boundaries_{};
  std::vector<PieceBoundary> orin_boundaries_{};
  std::vector<PieceBoundary> orin_boundaries_sample{};
  ThirdOrderSplinePath::State init_state_;
  CilqrCarModel::PathModelState init_state_car;
  std::vector<CilqrCarModel::PathKappa> kappa_seq_{};
  tk::spline bound_left_l_spline_;
  tk::spline bound_right_l_spline_;
  std::size_t n_{0};
  bool task_failed{false};
  SLPoint frenet_init_point;
  PathCarModelOutput path_cilqr_{};
  std::vector<TrajectoryPoint> computed_trajectory_points_{};
  std::vector<CilqrCarModel::PathModelState> cilqr_path_{};
};

}  // namespace planning
}  // namespace ceshi