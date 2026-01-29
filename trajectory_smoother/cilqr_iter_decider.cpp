#include "cilqr_iter_decider.h"

#include <unordered_map>

#include "../../single_frame/mock_headers/src/planning/common/log.h"
#include "../../single_frame/mock_headers/src/planning/common/vehicle_param.h"
#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/util/speed_planner_common.h"

namespace ceshi {
namespace planning {
// void VisilqrOptPath(
//     double lon_sample_s,
//     const std::vector<CilqrCarModel::PathModelState>& opt_path_variables,
//     const std::vector<double>& knots_delta_s,
//     const ReferenceLineShrPtr& reference_line, std::string s) {
//   if (!FLAGS_planning_enable_vis_event) return;

//   auto event = vis::EventSender::Instance()->GetEvent(s);
//   event->set_type(visualizer::Event::k3D);
//   event->add_attribute(visualizer::Event::kOdom);

//   for (std::size_t i = 0; i < opt_path_variables.size(); ++i) {
//     SLPoint frenet_point(lon_sample_s, opt_path_variables[i].state_l);
//     Vec2d cartesian_point;
//     if (!reference_line->GetPointInCartesianFrame(frenet_point,
//                                                   &cartesian_point)) {
//       LOG_ERROR("Fail to convert sl point to cartesian point");
//       return;
//     }
//     auto sphere = event->mutable_sphere()->Add();
//     sphere->mutable_center()->set_x(cartesian_point.x());
//     sphere->mutable_center()->set_y(cartesian_point.y());
//     sphere->mutable_center()->set_z(0);
//     sphere->set_radius(0.2);
//     lon_sample_s += knots_delta_s[i];
//   }
// }

PathConstrainedIterLqrDeciderAML::PathConstrainedIterLqrDeciderAML(
    std::string name)
    : name_(name) {}
// //将上游得到的sl坐标系下的优化结果转换到欧式空间
// void PathConstrainedIterLqrDeciderAML::TranslateSLtoCartesian(
//     const std::vector<double>& knots_delta_s,
//     const std::vector<ThirdOrderSplinePath::OptVariables>&
//     opt_path_variables_fre, std::vector<CilqrCarModel::OptVariables>&
//     opt_path_variables_car, const ReferenceLineShrPtr& reference_line, const
//     InsidePlannerData &inside_data)
// {
//     // ========== 1. 当前帧轨迹的SL->欧式处理 ==========
//     std::vector<Vec2d> xy_seq;
//     t_seq.clear();
//     frenet_init_point = inside_data.init_sl_point;
//     double starts = frenet_init_point.s();
//     total_s = 0;
//     for (auto &delta_s : knots_delta_s) total_s += delta_s;
//     LOG_INFO("start_s:{},start_l:{}
//     ,total_s:{},opt_path_variables_fre[0].xk.state_l0:{}",
//     starts,frenet_init_point.l(),total_s,opt_path_variables_fre[0].xk.state_l0);
//     double init_v = total_s / total_num_init / dt;
//     int total_num = total_num_init + 3;
//     // 当前帧SL插值用的数据
//     std::vector<double> l_vec, s_vec;
//     l_vec.reserve(opt_path_variables_fre.size());
//     s_vec.reserve(opt_path_variables_fre.size());
//     s_vec.push_back(frenet_init_point.s());
//     l_vec.push_back(opt_path_variables_fre[0].xk.state_l0);
//     for (size_t i = 1; i < opt_path_variables_fre.size(); i++) {
//         double cur_s = s_vec.back() + knots_delta_s[i - 1];
//         double cur_l = opt_path_variables_fre[i].xk.state_l0;
//         l_vec.emplace_back(cur_l);
//         s_vec.emplace_back(cur_s);
//     }
//     double max_s = s_vec.back();
//     // 构建当前帧样条
//     tk::spline l_spline;
//     l_spline.set_points(s_vec, l_vec, tk::spline::spline_type::cspline);
//     // 构建时刻SL序列
//     std::vector<SLPoint> sl_seqs;
//     double start_t = 0., cur_s = starts, cur_l = l_vec[0];
//     sl_seqs.push_back({starts, l_vec[0]});
//     for (size_t i = 0; i < total_num; ++i) {
//         cur_s += init_v * dt;
//         if (cur_s >= max_s) break;
//         cur_l = l_spline(cur_s);
//         sl_seqs.push_back({cur_s, cur_l});
//     }
//     // SL→欧式坐标
//     Vec2d xy_pt;
//     for (size_t i = 0; i < sl_seqs.size(); ++i) {
//         if (!reference_line->GetPointInCartesianFrame(sl_seqs[i], &xy_pt)) {
//             LOG_ERROR("CILQR state translate failed_FreToCar.");
//         }
//         xy_seq.push_back(xy_pt);
//         t_seq.push_back(start_t);
//         start_t += dt;
//     }
//     // 状态转换
//     ConvertXYSeqToOptVariables(xy_seq, t_seq, opt_path_variables_car);
//     LOG_INFO("CILQR 当前帧 sample done, sl_seqs.size()={}, xy_seq.size()={},
//     t_seq.size()={},opt_path_variables_car:{}", sl_seqs.size(),
//     xy_seq.size(), t_seq.size(), opt_path_variables_car.size());
//     // ========== 2. 上一帧轨迹的SL->欧式处理（last_path判空保护） ==========
//      // ========== 2. 上一帧轨迹SL->欧式 ==========
//     auto &task_info = DataCenter::Instance()->task_info();
//     auto last_path =
//         task_info.last_frame()
//             ? task_info.last_frame()->outside_planner_data().path_data
//             : nullptr;
//     opt_path_variables_car_last.clear();
//     if (last_path && !last_path->frenet_path().points().empty()) {
//         std::vector<Vec2d> xy_seq_last;
//         std::vector<double> s_last_vec, x_last_vec,y_last_vec;
//         const auto &reference_points = last_path->path().path_points();
//         const auto &fre_points = last_path->frenet_path().points();
//         s_last_vec.reserve(reference_points.size());
//         x_last_vec.reserve(reference_points.size());
//         y_last_vec.reserve(reference_points.size());
//         for (int i =0;i<reference_points.size();++i) {
//             s_last_vec.push_back(fre_points[i].s());
//             x_last_vec.push_back(reference_points[i].x());
//             y_last_vec.push_back(reference_points[i].y());
//         }
//         // 记录原始采样点数量
//         size_t raw_size = s_last_vec.size();
//         // 补尾
//         double step = 0.3;
//         double last_s = s_last_vec.empty() ? 0.0 : s_last_vec.back();
//         while (last_s + step < max_s + 1e-6) {
//             last_s += step;
//             s_last_vec.push_back(last_s);
//             SLPoint cur={last_s,0.0};
//             Vec2d curxy;
//             reference_line_->GetPointInCartesianFrame(cur, &curxy);
//             x_last_vec.push_back(curxy.x());
//             y_last_vec.push_back(curxy.y());
//         }
//         std::ostringstream oss_last;
// oss_last << "上一帧采样点(s, x, y):\n";
// for (size_t i = 0; i < s_last_vec.size(); ++i) {
//     oss_last << "[" << i << "]: s = " << s_last_vec[i]
//              << ", x = " << x_last_vec[i]
//              << ", y = " << y_last_vec[i] << "\n";
// }
// LOG_INFO("{}", oss_last.str());
// std::ostringstream oss_tail;
// oss_tail << "补尾采样点(s, x, y):\n";
// for (size_t i = raw_size; i < s_last_vec.size(); ++i) {
//     oss_tail << "[" << i << "]: s = " << s_last_vec[i]
//              << ", x = " << x_last_vec[i]
//              << ", y = " << y_last_vec[i] << "\n";
// }
// LOG_INFO("{}", oss_tail.str());
//         // // oss打印
//         // std::ostringstream oss_last, oss_last_pad;
//         // oss_last << "上一帧采样点（原始）:\n";
//         // for (size_t i = 0; i < raw_size; ++i) {
//         //     oss_last << "[" << i << "]: s=" << s_last_vec[i]
//         //              << ", l=" << l_last_vec[i] << "\n";
//         // }
//         // oss_last_pad << "上一帧采样点（补全）:\n";
//         // for (size_t i = raw_size; i < s_last_vec.size(); ++i) {
//         //     oss_last_pad << "[" << i << "]: s=" << s_last_vec[i]
//         //                  << ", l=" << l_last_vec[i] << "\n";
//         // }
//         // LOG_INFO("上一帧采样点:\n{}\n补全点:\n{}",
//         //          oss_last.str().c_str(), oss_last_pad.str().c_str());
//         // 构建last样条
//         tk::spline sx_spline_last,sy_spline_last;
//         sx_spline_last.set_points(s_last_vec, x_last_vec,
//         tk::spline::spline_type::cspline);
//         sy_spline_last.set_points(s_last_vec, y_last_vec,
//         tk::spline::spline_type::cspline);
//         // 构建上一帧采样SXY
//         std::vector<SLPoint> sl_seqs_last;
//         double cur_s = frenet_init_point.s();
//         double cur_x = 0,cur_y=0;
//         for (size_t i = 0; i < total_num; ++i) {
//             if (cur_s >= max_s) break;
//             cur_x = sx_spline_last(cur_s);
//             cur_y = sy_spline_last(cur_s);
//             Vec2d xy_pt(cur_x,cur_y);
//             xy_seq_last.push_back(xy_pt);
//             cur_s += init_v * dt;
//         }
//         //test
//         {
//             Vec2d cur_xy=xy_seq_last[0];
//             SLPoint cur_sl;
//             reference_line_->GetPointInFrenetFrame(cur_xy, &cur_sl);
//             LOG_INFO("start_s:{},start_l{},xy_start_s:{},xy_start_l:{}",
//             frenet_init_point.s(),frenet_init_point.l(),cur_sl.s(),cur_sl.l());
//         }
//         ConvertXYSeqToOptVariables(xy_seq_last, t_seq,
//         opt_path_variables_car_last); LOG_INFO("CILQR 上一帧 sample done,
//         sl_seqs_last.size()={}, xy_seq_last.size()={}, t_seq.size()={},
//         opt_path_variables_car_last:{}",
//             sl_seqs_last.size(), xy_seq_last.size(), t_seq.size(),
//             opt_path_variables_car_last.size());
//     }
// }

PathModelOutput PathConstrainedIterLqrDeciderAML::TranslateCartesiantoSL(
    const std::vector<double>& knots_delta_s,
    const PathCarModelOutput& path_car_model_output,
    const ReferenceLine& reference_line) {
  PathModelOutput result;
  std::vector<PathModel::PathModelControl> controls_fre;
  std::vector<PathModel::PathModelState> states_fre;

  std::vector<double> l_vec, s_vec;
  double max_s = 0.0;
  const auto& states = path_car_model_output.second;

  for (size_t i = 0; i < states.size(); i++) {
    double x = states[i].state_x;
    double y = states[i].state_y;
    Vec2d cur(x, y);
    SLPoint sl_pt;
    reference_line.GetPointInFrenetFrame(cur, &sl_pt);
    s_vec.emplace_back(sl_pt.s());
    l_vec.emplace_back(sl_pt.l());
    if (sl_pt.s() > max_s) max_s = sl_pt.s();
  }

  // 用已有 s, l 点构建完整轨迹
  std::vector<double> full_s_seq, full_l_seq;
  double cur_s = frenet_init_point.s();
  double cur_l = frenet_init_point.l();
  full_s_seq.push_back(cur_s);
  full_l_seq.push_back(cur_l);

  for (size_t i = 0; i < knots_delta_s.size(); ++i) {
    cur_s += knots_delta_s[i];
    if (cur_s > max_s) {
      cur_s = max_s;
      cur_l = l_vec.back();
    } else {
      // 线性插值计算 l
      size_t j = 1;
      while (j < s_vec.size() && s_vec[j] < cur_s) ++j;
      if (j >= s_vec.size()) j = s_vec.size() - 1;
      double s0 = s_vec[j - 1], s1 = s_vec[j];
      double l0 = l_vec[j - 1], l1 = l_vec[j];
      double ratio = (cur_s - s0) / (s1 - s0 + 1e-6);
      cur_l = l0 + ratio * (l1 - l0);
    }
    full_s_seq.push_back(cur_s);
    full_l_seq.push_back(cur_l);
  }

  size_t N = full_s_seq.size();
  for (size_t i = 0; i < N; ++i) {
    double s = full_s_seq[i];
    double l = full_l_seq[i];

    double dl = 0.0, ddl = 0.0, dddl = 0.0;

    if (i + 1 < N) {
      double ds = full_s_seq[i + 1] - s;
      if (ds > 1e-6) {
        dl = (full_l_seq[i + 1] - l) / ds;
      }
    }

    // if (i + 2 < N) {
    //     double ds1 = full_s_seq[i + 1] - full_s_seq[i];
    //     double ds2 = full_s_seq[i + 2] - full_s_seq[i + 1];
    //     if (ds1 > 1e-6 && ds2 > 1e-6) {
    //         double dl1 = (full_l_seq[i + 1] - full_l_seq[i]) / ds1;
    //         double dl2 = (full_l_seq[i + 2] - full_l_seq[i + 1]) / ds2;
    //         ddl = (dl2 - dl1) / ((ds1 + ds2) * 0.5);
    //     }
    // }

    // if (i + 3 < N) {
    //     double ds1 = full_s_seq[i + 1] - full_s_seq[i];
    //     double ds2 = full_s_seq[i + 2] - full_s_seq[i + 1];
    //     double ds3 = full_s_seq[i + 3] - full_s_seq[i + 2];
    //     if (ds1 > 1e-6 && ds2 > 1e-6 && ds3 > 1e-6) {
    //         double dl0 = (full_l_seq[i + 1] - full_l_seq[i]) / ds1;
    //         double dl1 = (full_l_seq[i + 2] - full_l_seq[i + 1]) / ds2;
    //         double dl2 = (full_l_seq[i + 3] - full_l_seq[i + 2]) / ds3;
    //         double ddl1 = (dl1 - dl0) / ((ds1 + ds2) * 0.5);
    //         double ddl2 = (dl2 - dl1) / ((ds2 + ds3) * 0.5);
    //         dddl = (ddl2 - ddl1) / ((ds1 + ds2 + ds3) / 3.0);
    //     }
    // }

    PathModel::PathModelState state_fre;
    PathModel::PathModelControl control_fre;
    state_fre.state_l = l;
    state_fre.state_dl = dl;
    state_fre.state_ddl = ddl;
    control_fre.control_dddl = dddl;

    controls_fre.push_back(control_fre);
    states_fre.push_back(state_fre);
  }

  LOG_INFO("CILQR_optvar_num:{}", states_fre.size());
  result.first = controls_fre;
  result.second = states_fre;
  return result;
}

double PathConstrainedIterLqrDeciderAML::calc_curvature(const Vec2d& p1,
                                                        const Vec2d& p2,
                                                        const Vec2d& p3) {
  double a = std::hypot(p2.x() - p1.x(), p2.y() - p1.y());
  double b = std::hypot(p3.x() - p2.x(), p3.y() - p2.y());
  double c = std::hypot(p3.x() - p1.x(), p3.y() - p1.y());
  // 带方向的面积（叉积/2）
  double area2 = (p2.x() - p1.x()) * (p3.y() - p1.y()) -
                 (p2.y() - p1.y()) * (p3.x() - p1.x());  // 两倍有向面积
  // 防止除零
  if (a * b * c < 1e-8) return 0.0;
  return 2.0 * area2 / (a * b * c);  // 带正负号的曲率
}

void PathConstrainedIterLqrDeciderAML::ConvertXYSeqToOptVariables(
    const std::vector<TrajectoryPoint>& rough_trajectory,
    std::vector<double>& time_seq,  // 必须和 xy_seq 同长度
    std::vector<CilqrCarModel::OptVariables>& opt_path_variables_car) {
  std::vector<Vec2d> xy_seq;
  std::vector<double> heading_vec;
  time_seq.clear();
  xy_seq.clear();
  // TODO
  //将粗解轨迹中的信息提取：坐标和时间序列信息；
  for (const auto& rough_point : rough_trajectory) {
    Vec2d curxy(rough_point.x(), rough_point.y());
    xy_seq.push_back(curxy);
    time_seq.push_back(rough_point.relative_time());
  }

  using CilqrCarModel::OptVariables;
  size_t n = xy_seq.size();
  opt_path_variables_car.clear();
  if (n < 2) return;

  for (size_t i = 0; i < n; ++i) {
    OptVariables opt;
    // x, y()
    opt.xk.state_x = xy_seq[i].x();
    opt.xk.state_y = xy_seq[i].y();

    // heading
    if (i == n - 1) {
      opt.xk.state_heading = std::atan2(xy_seq[n - 1].y() - xy_seq[n - 2].y(),
                                        xy_seq[n - 1].x() - xy_seq[n - 2].x());
    } else {
      opt.xk.state_heading = std::atan2(xy_seq[i + 1].y() - xy_seq[i].y(),
                                        xy_seq[i + 1].x() - xy_seq[i].x());
    }

    // v

    if (i == n - 1) {
      opt.xk.state_v = std::hypot(xy_seq[n - 1].x() - xy_seq[n - 2].x(),
                                  xy_seq[n - 1].y() - xy_seq[n - 2].y()) /
                       (time_seq[n - 1] - time_seq[n - 2]);
    } else {
      opt.xk.state_v = std::hypot(xy_seq[i + 1].x() - xy_seq[i].x(),
                                  xy_seq[i + 1].y() - xy_seq[i].y()) /
                       (time_seq[i + 1] - time_seq[i]);
    }

    opt.xk.state_kappa = 0.0;
    // 先设为0，后面统一批量计算a, dkappa
    opt.uk.control_a = 0.0;
    opt.uk.control_dkappa = 0.0;
    opt_path_variables_car.push_back(opt);
  }
  // opt_path_variables_car[0].xk=init_state_car;
  // heading 连续化处理；
  std::vector<double> heading_seq(n);
  for (size_t i = 0; i < n; ++i) {
    heading_seq[i] = opt_path_variables_car[i].xk.state_heading;
  }
  // 连续化处理（unwrap）
  for (size_t i = 1; i < n; ++i) {
    double diff = heading_seq[i] - heading_seq[i - 1];
    while (diff > M_PI) {
      heading_seq[i] -= 2 * M_PI;
      diff = heading_seq[i] - heading_seq[i - 1];
    }
    while (diff < -M_PI) {
      heading_seq[i] += 2 * M_PI;
      diff = heading_seq[i] - heading_seq[i - 1];
    }
    opt_path_variables_car[i].xk.state_heading = heading_seq[i];
  }

  // 4. 用heading差分公式计算kappa
  for (size_t i = 0; i < n; ++i) {
    if (n < 3) {
      opt_path_variables_car[i].xk.state_kappa = 0.0;
      continue;
    }
    if (i == n - 1) {
      // 后向差分
      double dtheta = heading_seq[n - 1] - heading_seq[n - 2];
      opt_path_variables_car[i].xk.state_kappa =
          dtheta / opt_path_variables_car[i].xk.state_v /
          (time_seq[n - 1] - time_seq[n - 2]);
    } else {
      // 中心差分
      double dtheta = heading_seq[i + 1] - heading_seq[i];
      if (i == 0) {
        opt_path_variables_car[i].xk.state_kappa =
            std::clamp(dtheta / opt_path_variables_car[i].xk.state_v /
                           (time_seq[i + 1] - time_seq[i]),
                       -0.3, 0.3);

      } else {
        opt_path_variables_car[i].xk.state_kappa =
            dtheta / opt_path_variables_car[i].xk.state_v /
            (time_seq[i + 1] - time_seq[i]);
      }
    }
  }

  // a, dkappa（中心差分，首尾用前/后向差分）
  size_t n_var = opt_path_variables_car.size();
  for (size_t i = 0; i < n_var; ++i) {
    if (i == n_var - 1 && n_var > 2) {
      opt_path_variables_car[i].uk.control_a =
          (opt_path_variables_car[n_var - 1].xk.state_v -
           opt_path_variables_car[n_var - 2].xk.state_v) /
          (time_seq[n - 1] - time_seq[n - 2]);
      opt_path_variables_car[i].uk.control_dkappa =
          (opt_path_variables_car[n_var - 1].xk.state_kappa -
           opt_path_variables_car[n_var - 2].xk.state_kappa) /
          (time_seq[n - 1] - time_seq[n - 2]);
    } else if (n_var > 2) {
      opt_path_variables_car[i].uk.control_a =
          (opt_path_variables_car[i + 1].xk.state_v -
           opt_path_variables_car[i].xk.state_v) /
          (time_seq[i + 1] - time_seq[i]);
      opt_path_variables_car[i].uk.control_dkappa =
          (opt_path_variables_car[i + 1].xk.state_kappa -
           opt_path_variables_car[i].xk.state_kappa) /
          (time_seq[i + 1] - time_seq[i]);
    } else {
      opt_path_variables_car[i].uk.control_a = 0.0;
      opt_path_variables_car[i].uk.control_dkappa = 0.0;
    }
  }
}

PathConstrainedIterLqrDeciderAML::~PathConstrainedIterLqrDeciderAML() {}

void PathConstrainedIterLqrDeciderAML::SaveLOGResults() {}
void PathConstrainedIterLqrDeciderAML::SaveTaskResults(TaskInfo& task_info){};
void PathConstrainedIterLqrDeciderAML::Reset(){};

ErrorCode PathConstrainedIterLqrDeciderAML::Execute(
    const ThirdOrderSplinePath::State& init_state,
    const InsidePlannerData& inside_data,
    const std::vector<TrajectoryPoint>& rough_trajectory,
    const std::vector<double>& knots_delta_s,
    const std::vector<PieceBoundary>& piece_boundaries,
    const std::vector<PieceBoundary>& orin_boundaries,
    const ReferenceLine& reference_line) {
  LOG_INFO(">>>> start execute cilqr");

  //   const auto& path_cilqr =
  //       config::PlanningConfig::Instance()->planning_research_config().path_cilqr;
  frenet_init_point = inside_data.init_sl_point;
  knots_delta_s_ = knots_delta_s;
  piece_boundaries_ = piece_boundaries;
  orin_boundaries_ = orin_boundaries;
  init_state_ = init_state;
  reference_line_ = &reference_line;
  ReferencePoint ref_point;

  auto wheel_base = VehicleParam::Instance()->wheel_base();
  auto max_steer_angle_ = VehicleParam::Instance()->max_steer_angle();
  auto ratio = VehicleParam::Instance()->steer_ratio();

  //计算欧式坐标系下的状态
  init_state_car.state_x = inside_data.vel_x;
  init_state_car.state_y = inside_data.vel_y;
  init_state_car.state_v = inside_data.vel_v;
  init_state_car.state_heading = inside_data.vel_heading;
  init_state_car.state_kappa =
      std::tan(inside_data.vel_steer_angle / 100 * max_steer_angle_ / ratio) /
      wheel_base;

  opt_path_variables_car.clear();
  // TranslateSLtoCartesian(knots_delta_s,opt_path_variables,
  // opt_path_variables_car,reference_line,inside_data);
  ConvertXYSeqToOptVariables(rough_trajectory, t_seq, opt_path_variables_car);
  n_ = opt_path_variables_car.size();
  LOG_INFO("TranslateSLtoCartesian_success");

  if (!Init()) {
    LOG_ERROR("Path Constrained Iter Lqr Decider Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  LOG_INFO("CILQR_Init_Success");

  if (!Process()) {
    LOG_ERROR("Speed Iter Deduction Decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Process Finished.");
  SaveLOGResults();

  return ErrorCode::PLANNING_OK;
}

bool PathConstrainedIterLqrDeciderAML::Init() {
  //初始化原始边界；
  // path_goal_.clear();
  // for(auto &opt_path_variable_car :opt_path_variables_car){
  //     CilqrCarModel::OptVariables state;
  //     Vec2d cur{state.xk.state_x,state.xk.state_y};
  //     ReferencePoint ref;
  //     reference_line_->GetNearestRefPoint(cur,&ref);

  // }

  // auto const& points = reference_line_->ref_points();
  // double starts = frenet_init_point.s();
  // int n=(starts + total_s)/0.2+10;
  // orin_boundaries_car.resize(n);
  // for (auto const &point : points) {
  //     if (point.s() >= starts - 3 && point.s() <= starts + total_s + 3) {
  //         CilqrCarModel::PieceBoundary ori;
  //         ori.s = point.s();
  //         ori.x = point.x();
  //         ori.y = point.y();
  //         ori.left_bound = point.left_bound();
  //         ori.right_bound = -point.right_bound();
  //         orin_boundaries_car.push_back(ori);

  //     }
  // }

  //  // 构造左右边界样条
  //   std::vector<double> bound_s_vec, bound_left_vec, bound_right_vec;
  //   for (auto &b : orin_boundaries_car) {
  //     bound_s_vec.push_back(b.s);
  //     bound_left_vec.push_back(b.left_bound);
  //     bound_right_vec.push_back(b.right_bound);
  //   }
  //   bound_left_l_spline_.set_points(bound_s_vec, bound_left_vec,
  //                                   tk::spline::spline_type::linear);
  //   bound_right_l_spline_.set_points(bound_s_vec, bound_right_vec,
  //                                    tk::spline::spline_type::linear);
  // 重采样边界
  // orin_boundaries_sample.clear();
  // for (double s_val : s_seq) {
  //   CilqrCarModel::PieceBoundary pb;
  //   pb.s = s_val;
  //   pb.left_bound = bound_left_l_spline_(s_val);
  //   pb.right_bound = bound_right_l_spline_(s_val);
  //   orin_boundaries_sample.push_back(pb);
  // }
  return true;
}

void PathConstrainedIterLqrDeciderAML::MemberReset() {
  knots_delta_s_.clear();
  opt_path_variables_car.clear();
  opt_path_variables_.clear();
  piece_boundaries_.clear();
  orin_boundaries_.clear();
  orin_boundaries_car.clear();
}


bool PathConstrainedIterLqrDeciderAML::get_xystate_seqs(std::vector<TrajectoryPoint>& rough_trajectory) {
  std::vector<CilqrCarModel::PathModelState> cilqr_state = path_cilqr_.second;
  std::vector<double> t_vec, x_vec, y_vec, heading_vec, kappa_vec;
  

  if(cilqr_state.size()!=rough_trajectory.size()){
    LOG_INFO("cilqr size error,cilqr_state:{},rough_trajectory:{}",cilqr_state.size(),rough_trajectory.size());
    return false;
  }
  std::ostringstream oss;
  for(size_t i=0;i<cilqr_state.size();++i){
    rough_trajectory[i].set_theta(cilqr_state[i].state_heading);
    rough_trajectory[i].set_x(cilqr_state[i].state_x);
    rough_trajectory[i].set_y(cilqr_state[i].state_y);
    rough_trajectory[i].set_kappa(cilqr_state[i].state_kappa);
    rough_trajectory[i].set_velocity(cilqr_state[i].state_v);
        oss << "[" << i << "]"
        << " x=" << cilqr_state[i].state_x
        << ", y=" << cilqr_state[i].state_y
        << ", heading=" << cilqr_state[i].state_heading
        << ", kappa=" << cilqr_state[i].state_kappa
        << ", v=" << cilqr_state[i].state_v
        << "\n";
  }
// LOG_INFO("cilqr_state each point info:\n{}", oss.str());
  return true;
}

// bool PathConstrainedIterLqrDeciderAML::get_xystate_seqs() {
//   std::vector<CilqrCarModel::PathModelState> cilqr_state = path_cilqr_.second;
//   std::vector<double> t_vec, x_vec, y_vec, heading_vec, kappa_vec;
//   t_vec = t_seq;
//   x_vec.reserve(cilqr_state.size() + 1);
//   y_vec.reserve(cilqr_state.size() + 1);
//   heading_vec.reserve(cilqr_state.size() + 1);
//   kappa_vec.reserve(cilqr_state.size() + 1);
//   // 原始点
//   for (size_t i = 0; i < cilqr_state.size(); i++) {
//     x_vec.emplace_back(cilqr_state[i].state_x);
//     y_vec.emplace_back(cilqr_state[i].state_y);
//     heading_vec.emplace_back(cilqr_state[i].state_heading);
//     kappa_vec.emplace_back(cilqr_state[i].state_kappa);
//   }

//   // --- 在首端插入外推点（可选，可去掉，视你的应用场景） ---
//   if (cilqr_state.size() >= 2) {
//     double t0 = t_vec[0], t1 = t_vec[1];
//     double x0 = x_vec[0], x1 = x_vec[1];
//     double y0 = y_vec[0], y1 = y_vec[1];
//     double heading0 = heading_vec[0], heading1 = heading_vec[1];
//     double kappa0 = kappa_vec[0], kappa1 = kappa_vec[1];
//     double dt = t1 - t0;
//     double t_pre = t0 - dt;
//     double x_pre = x0 - (x1 - x0);
//     double y_pre = y0 - (y1 - y0);

//     // heading 外推（unwrap，保证连贯）
//     double delta_heading = heading1 - heading0;
//     while (delta_heading > M_PI) delta_heading -= 2 * M_PI;
//     while (delta_heading < -M_PI) delta_heading += 2 * M_PI;
//     double heading_pre = heading0 - delta_heading;
//     double kappa_pre = 2 * kappa0 - kappa1;

//     t_vec.insert(t_vec.begin(), t_pre);
//     x_vec.insert(x_vec.begin(), x_pre);
//     y_vec.insert(y_vec.begin(), y_pre);
//     heading_vec.insert(heading_vec.begin(), heading_pre);
//     kappa_vec.insert(kappa_vec.begin(), kappa_pre);
//   }

//   // 样条拟合
//   tk::spline x_spline, y_spline;
//   x_spline.set_points(t_vec, x_vec, tk::spline::spline_type::cspline);
//   y_spline.set_points(t_vec, y_vec, tk::spline::spline_type::cspline);

//   // 采样
//   double t_start = t_vec[1];  // 起点为原第一个点
//   double t_end = t_vec.back();
//   double dt_sample = 0.05;
//   std::vector<double> t_dense, x_dense, y_dense;
//   for (double t = t_start; t <= t_end + 1e-8; t += dt_sample) {
//     x_dense.push_back(x_spline(t));
//     y_dense.push_back(y_spline(t));
//     t_dense.push_back(t);
//   }
//   size_t n = x_dense.size();

//   // 1. v_dense
//   std::vector<double> v_dense(n, 0.0);
//   for (size_t i = 0; i < n; ++i) {
//     double dx, dy;
//     if (i < n - 1) {
//       dx = x_dense[i + 1] - x_dense[i];
//       dy = y_dense[i + 1] - y_dense[i];
//     } else {
//       dx = x_dense[i] - x_dense[i - 1];
//       dy = y_dense[i] - y_dense[i - 1];
//     }
//     v_dense[i] = std::hypot(dx, dy) / dt_sample;
//   }

//   // 2. 原始点区间常值 heading_dense
//   std::vector<double> heading_dense(n, 0.0);
//   size_t orig_idx = 0;
//   for (size_t i = 0; i < n; ++i) {
//     while (orig_idx + 1 < t_vec.size() && t_dense[i] >= t_vec[orig_idx + 1]) {
//       ++orig_idx;
//     }
//     heading_dense[i] = heading_vec[orig_idx];
//   }

//   // 2.1 heading unwrap，消除跳变
//   for (size_t i = 1; i < n; ++i) {
//     while (heading_dense[i] - heading_dense[i - 1] > M_PI)
//       heading_dense[i] -= 2 * M_PI;
//     while (heading_dense[i] - heading_dense[i - 1] < -M_PI)
//       heading_dense[i] += 2 * M_PI;
//   }

//   orig_idx = 0;
//   std::vector<double> kappa_dense(n, 0.0);
//   for (size_t i = 0; i < n; ++i) {
//     while (orig_idx + 1 < t_vec.size() && t_dense[i] >= t_vec[orig_idx + 1]) {
//       ++orig_idx;
//     }
//     kappa_dense[i] = kappa_vec[orig_idx];
//   }

//   // 4. 批量赋值到cilqr_path_
//   cilqr_path_.clear();
//   computed_trajectory_points_.clear();

//   // 获取控制序列用于加速度信息
//   const auto& control_seq = path_cilqr_.first;

//   double s = 0.0;

//   for (size_t i = 0; i < n; ++i) {
//     CilqrCarModel::PathModelState state;
//     state.state_x = x_dense[i];
//     state.state_y = y_dense[i];
//     state.state_v = v_dense[i];
//     state.state_heading = heading_dense[i];
//     state.state_kappa = kappa_dense[i];
//     cilqr_path_.push_back(state);

//     // 填充computed_trajectory_points_
//     TrajectoryPoint traj_point;

//     // 设置路径点信息
//     PathPoint path_point;
//     path_point.set_x(x_dense[i]);
//     path_point.set_y(y_dense[i]);
//     path_point.set_theta(heading_dense[i]);
//     path_point.set_kappa(kappa_dense[i]);
//     if (i > 0) {
//       double dx = x_dense[i] - x_dense[i - 1];
//       double dy = y_dense[i] - y_dense[i - 1];
//       s += std::hypot(dx, dy);
//     }
//     path_point.set_s(s);
//     path_point.set_dkappa(0.0);  // 如果需要dkappa，可以后续计算

//     traj_point.set_path_point(path_point);
//     traj_point.set_velocity(v_dense[i]);
//     traj_point.set_relative_time(t_dense[i]);

//     // 设置加速度 - 从控制序列获取
//     if (i < control_seq.size()) {
//       traj_point.set_acceleration(control_seq[i].control_a);
//     } else {
//       // 如果控制序列长度不够，使用0或者前一个值
//       traj_point.set_acceleration(0.0);
//     }

//     // 设置jerk（如果需要）
//     traj_point.set_jerk(0.0);

//     computed_trajectory_points_.push_back(traj_point);
//   }
//   return true;
// }

// bool PathConstrainedIterLqrDeciderAML::get_xystate_seqs() {
//     std::vector<CilqrCarModel::PathModelState> cilqr_state =
//     path_cilqr_.second; std::vector<double> t_vec, x_vec, y_vec; t_vec =
//     t_seq; x_vec.reserve(cilqr_state.size() + 1); // 预留多1个点
//     y_vec.reserve(cilqr_state.size() + 1);

//     for (size_t i = 0; i < cilqr_state.size(); i++) {
//         x_vec.emplace_back(cilqr_state[i].state_x);
//         y_vec.emplace_back(cilqr_state[i].state_y);
//     }

//     // --- 在首端插入外推点 ---
//     if (cilqr_state.size() >= 2) {
//         double t0 = t_vec[0], t1 = t_vec[1];
//         double x0 = x_vec[0], x1 = x_vec[1];
//         double y0 = y_vec[0], y1 = y_vec[1];

//         double dt = t1 - t0;
//         double t_pre = t0 - dt;
//         double x_pre = x0 - (x1 - x0);
//         double y_pre = y0 - (y1 - y0);

//         // 插入到首位
//         t_vec.insert(t_vec.begin(), t_pre);
//         x_vec.insert(x_vec.begin(), x_pre);
//         y_vec.insert(y_vec.begin(), y_pre);
//     }

//     // 样条拟合
//     tk::spline x_spline, y_spline;
//     x_spline.set_points(t_vec, x_vec, tk::spline::spline_type::cspline);
//     y_spline.set_points(t_vec, y_vec, tk::spline::spline_type::cspline);

//     // 采样
//     double t_start = t_vec[1]; // 注意！现在起点是原来的第1个点
//     double t_end = t_vec.back();
//     double dt_sample = 0.05;
//     std::vector<double> t_dense, x_dense, y_dense;
//     for (double t = t_start; t <= t_end + 1e-8; t += dt_sample) {
//         x_dense.push_back(x_spline(t));
//         y_dense.push_back(y_spline(t));
//         t_dense.push_back(t);
//     }
//     size_t n = x_dense.size();

//     // 后续 v/heading/kappa 计算保持原逻辑即可

//     // 1. 批量计算 v
//     std::vector<double> v_dense(n, 0.0);
//     for (size_t i = 0; i < n; ++i) {
//         double dx, dy;
//         if (i < n - 1) {
//             dx = x_dense[i + 1] - x_dense[i];
//             dy = y_dense[i + 1] - y_dense[i];
//         } else {
//             dx = x_dense[i] - x_dense[i - 1];
//             dy = y_dense[i] - y_dense[i - 1];
//         }
//         v_dense[i] = std::hypot(dx, dy) / dt_sample;
//     }

//     // 2. 批量计算 heading
//     std::vector<double> heading_dense(n, 0.0);
//     for (size_t i = 0; i < n; ++i) {
//         double dx, dy;
//         if (i < n - 1) {
//             dx = x_dense[i + 1] - x_dense[i];
//             dy = y_dense[i + 1] - y_dense[i];
//         } else {
//             dx = x_dense[i] - x_dense[i - 1];
//             dy = y_dense[i] - y_dense[i - 1];
//         }
//         heading_dense[i] = std::atan2(dy, dx);
//     }
//     for (size_t i = 1; i < n; ++i) {
//         while (heading_dense[i] - heading_dense[i - 1] > M_PI)
//         heading_dense[i] -= 2 * M_PI; while (heading_dense[i] -
//         heading_dense[i - 1] < -M_PI) heading_dense[i] += 2 * M_PI;
//     }

//     // 3. 批量计算 kappa
//     std::vector<double> kappa_dense(n, 0.0);
//     for (size_t i = 0; i < n; ++i) {
//         if (i < n - 1) {
//             double dtheta = heading_dense[i + 1] - heading_dense[i];
//             while (dtheta > M_PI) dtheta -= 2 * M_PI;
//             while (dtheta < -M_PI) dtheta += 2 * M_PI;
//             kappa_dense[i] = dtheta / v_dense[i] / dt_sample;
//         } else {
//             kappa_dense[i] = 0.0;
//         }
//     }

//     // 4. 批量赋值到cilqr_path_
//     cilqr_path_.clear();
//     for (size_t i = 0; i < n; ++i) {
//         CilqrCarModel::PathModelState state;
//         state.state_x = x_dense[i];
//         state.state_y = y_dense[i];
//         state.state_v = v_dense[i];
//         state.state_heading = heading_dense[i];
//         state.state_kappa = kappa_dense[i];
//         cilqr_path_.push_back(state);
//     }
//     return true;
// }

bool PathConstrainedIterLqrDeciderAML::Process() {
  std::shared_ptr<PathCilqrSolverAML> cilqr_problem{nullptr};

  cilqr_problem = std::make_shared<planning::PathCilqrSolverAML>(
      init_state_car, opt_path_variables_car, opt_path_variables_car_last,
      t_seq, orin_boundaries_car_sample, n_, bound_left_l_spline_,
      bound_right_l_spline_, orin_boundaries_car,
      *reference_line_);

  path_cilqr_ = cilqr_problem->Optimize();

  if (cilqr_problem->IsIllness()) {
    LOG_ERROR("CILQR_Solve_IsIllness");
    return false;
  }
  // TODO :
  // 输出转换
  // if (!get_xystate_seqs()) {
  //   return false;
  //   LOG_ERROR("get_xystate_seqs_false");
  // }

  //   VisilqrOptPath(orin_boundaries_sample.front().s, output.second,
  //                  knots_delta_s_, reference_line_, name_);

  //     auto output_fre=TranslateCartesiantoSL(knots_delta_s_,output,
  //     reference_line_);
  //   LOG_INFO("CILQR_TransCartoFre_success");
  //   path_cilqr_.clear();
  //   for (std::size_t i = 0; i < output_fre.first.size()-1; ++i) {
  //     ThirdOrderSplinePath::OptVariables pt;
  //     pt.xk.state_l0 = output_fre.second[i].state_l;
  //     pt.xk.state_dl = output_fre.second[i].state_dl;
  //     pt.xk.state_ddl = output_fre.second[i].state_ddl;
  //     pt.uk.control_dddl = output_fre.first[i].control_dddl;
  //     path_cilqr_.emplace_back(pt);
  //   }

  //   ThirdOrderSplinePath::OptVariables pt;
  //   pt.xk.state_l0 = output_fre.second.back().state_l;
  //   pt.xk.state_dl = output_fre.second.back().state_dl;
  //   pt.xk.state_ddl = output_fre.second.back().state_ddl;
  //   pt.uk.control_dddl = output_fre.first.back().control_dddl;
  //   path_cilqr_.emplace_back(pt);

  MemberReset();
  return true;
};

const std::vector<CilqrCarModel::PathModelState>
PathConstrainedIterLqrDeciderAML::GetOptSolution() {
  return cilqr_path_;
}

const std::vector<TrajectoryPoint>
PathConstrainedIterLqrDeciderAML::GetComputedTrajectoryPoints() {
  return computed_trajectory_points_;
}

}  // namespace planning
}  // namespace ceshi
