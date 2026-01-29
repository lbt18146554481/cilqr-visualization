/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-10-30 00:05:14
 * @LastEditTime: 2025-03-06 22:07:49
 * @FilePath: /toy-example-of-iLQR/src/motion_planning.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "cubic_spline.hpp"
#include "global_config.hpp"
#include "matplotlibcpp.h"
#include "utils.hpp"
#include "common/algorithm_config.hpp"
#include "common/types.hpp"
#include "../../trajectory_smoother/cilqr_iter_decider.h"
#include "../../single_frame/mock_headers/src/planning/common/path/path_point.h"
#include "../../single_frame/mock_headers/src/planning/common/data_center/inside_planner_data.h"
#include "../../single_frame/mock_headers/src/planning/task/optimizers/third_order_spline_path_optimizer/third_order_spline_path_model.h"
#include "../../single_frame/mock_headers/src/planning/common/vehicle_param.h"
#include "../../single_frame/mock_headers/src/planning/common/log.h"

#include <fmt/core.h>
#include <getopt.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <limits>

namespace plt = matplotlibcpp;

// 辅助函数：执行 Python 代码
inline void exec_python(const std::string& code) {
    matplotlibcpp::detail::_interpreter::get();
    int ret = PyRun_SimpleString(code.c_str());
    if (ret != 0) {
        PyErr_Print();
        SPDLOG_WARN("Python execution failed for: {}", code);
    }
}

int main(int argc, char** argv) {
    int opt;
    const char* optstring = "c:";
    std::string config_path;

    while ((opt = getopt(argc, argv, optstring)) != -1) {
        switch (opt) {
            case 'c':
                config_path = optarg;
                break;
            default:
                fmt::print("Usage: %s [-c]\n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    if (config_path.empty()) {
        fmt::print("Usage: %s [-c]\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    spdlog::set_level(spdlog::level::debug);
    SPDLOG_INFO("config path: {}", config_path);
    GlobalConfig* config = GlobalConfig::get_instance(config_path);

    double delta_t = config->get_config<double>("delta_t");
    double max_simulation_time = config->get_config<double>("max_simulation_time");
    double target_velocity = config->get_config<double>("vehicle/target_velocity");
    std::vector<double> reference_x =
        config->get_config<std::vector<double>>("laneline/reference/x");
    std::vector<double> reference_y =
        config->get_config<std::vector<double>>("laneline/reference/y");
    std::vector<double> border_widths = config->get_config<std::vector<double>>("laneline/border");
    std::vector<double> center_line_widths =
        config->get_config<std::vector<double>>("laneline/center_line");
    std::vector<std::vector<double>> initial_conditions =
        config->get_config<std::vector<std::vector<double>>>("initial_condition");
    double wheelbase = config->has_key("vehicle/wheelbase")
                       ? config->get_config<double>("vehicle/wheelbase")
                       : config->get_config<double>("vehicle/length");
    std::string reference_point_string = config->get_config<std::string>("vehicle/reference_point");
    ReferencePoint reference_point = ReferencePoint::GravityCenter;
    if (reference_point_string == "rear_center") {
        reference_point = ReferencePoint::RearCenter;
    }
    double VEHICLE_WIDTH = config->get_config<double>("vehicle/width");
    double VEHICLE_HEIGHT = config->get_config<double>("vehicle/length");
    double ACC_MAX = config->get_config<double>("vehicle/acc_max");
    double d_safe = config->get_config<double>("vehicle/d_safe");
    Eigen::Vector3d obs_attr = {VEHICLE_WIDTH, VEHICLE_HEIGHT, d_safe};
    Eigen::Vector2d vehicle_para = {VEHICLE_HEIGHT, VEHICLE_WIDTH};
    size_t vehicle_num = initial_conditions.size();

    bool show_reference_line = config->get_config<bool>("visualization/show_reference_line");
    bool show_obstacle_boundary = config->get_config<bool>("visualization/show_obstacle_boundary");
    std::vector<double> visual_x_limit = {0, 0};
    std::vector<double> visual_y_limit = {0, 0};
    if (config->has_key("visualization/x_lim")) {
        visual_x_limit = config->get_config<std::vector<double>>("visualization/x_lim");
    }
    if (config->has_key("visualization/y_lim")) {
        visual_y_limit = config->get_config<std::vector<double>>("visualization/y_lim");
    }

    std::vector<ReferenceLine> borders;
    std::vector<ReferenceLine> center_lines;
    for (double w : border_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        borders.emplace_back(reference);
    }
    for (double w : center_line_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        center_lines.emplace_back(reference);
    }
    std::sort(border_widths.begin(), border_widths.end(), std::greater<double>());
    Eigen::Vector2d road_borders;
    road_borders << border_widths[0], border_widths.back();

    Outlook outlook_ego;
    Outlook outlook_agent;
    Outlook outlook_steering;
    double steer_size = 5;
    std::filesystem::path source_file_path(__FILE__);
    std::filesystem::path project_path = source_file_path.parent_path().parent_path();
    std::string vehicle_pic_path_ego =
        (project_path / "images" / "materials" / "car_cyan.mat.txt").string();
    std::string vehicle_pic_path_agent =
        (project_path / "images" / "materials" / "car_white.mat.txt").string();
    std::string steering_pic_path =
        (project_path / "images" / "materials" / "steering_wheel.mat.txt").string();
    bool ego_loaded = utils::imread(vehicle_pic_path_ego, outlook_ego);
    bool agent_loaded = utils::imread(vehicle_pic_path_agent, outlook_agent);
    bool steering_loaded = utils::imread(steering_pic_path, outlook_steering);
    
    if (!ego_loaded) {
        SPDLOG_WARN("Failed to load ego vehicle image: {}", vehicle_pic_path_ego);
    }
    if (!agent_loaded) {
        SPDLOG_WARN("Failed to load agent vehicle image: {}", vehicle_pic_path_agent);
    }
    if (!steering_loaded) {
        SPDLOG_WARN("Failed to load steering wheel image: {}", steering_pic_path);
    }

    std::vector<RoutingLine> routing_lines(vehicle_num);
    for (size_t idx = 0; idx < vehicle_num; ++idx) {
        size_t line_num = 0;
        double start_s = center_lines[line_num].length();
        double min_diff = -1.0;
        for (size_t l = 0; l < center_lines.size(); ++l) {
            for (size_t i = 1; i < center_lines[l].size(); ++i) {
                double last_diff = hypot(center_lines[l].x[i - 1] - initial_conditions[idx][0],
                                         center_lines[l].y[i - 1] - initial_conditions[idx][1]);
                double cur_diff = hypot(center_lines[l].x[i] - initial_conditions[idx][0],
                                        center_lines[l].y[i] - initial_conditions[idx][1]);
                if (cur_diff > last_diff) {
                    if (min_diff < 0 || last_diff < min_diff) {
                        min_diff = last_diff;
                        line_num = l;
                        start_s = center_lines[l].longitude[i - 1];
                    }
                    break;
                }
            }
        }
        SPDLOG_DEBUG("idx: {}, line_num: {}, start_s: {}", idx, line_num, start_s);

        for (double t = 0.0; t < max_simulation_time + 10; t += delta_t) {
            double cur_s = 0.;
            Eigen::Vector3d pos;
            if (initial_conditions[idx][3] <= M_PI_2) {
                cur_s = start_s + t * initial_conditions[idx][2];
                cur_s = std::min(cur_s, center_lines[line_num].longitude.back());
                pos = center_lines[line_num].calc_position(cur_s);
            } else {
                cur_s = start_s - t * initial_conditions[idx][2];
                cur_s = std::max(cur_s, center_lines[line_num].longitude.front());
                pos = center_lines[line_num].calc_position(cur_s);
                pos.z() = fmod(pos.z() + M_PI, 2 * M_PI);
            }

            if (idx == 0 || Random::uniform(0.0, 1.0) < 0.5) {
                routing_lines[idx].x.push_back(pos.x());
                routing_lines[idx].y.push_back(pos.y());
                routing_lines[idx].yaw.push_back(pos.z());
            } else {
                routing_lines[idx].x.push_back(pos.x() + Random::normal(0.0, 0.02));
                routing_lines[idx].y.push_back(pos.y() + Random::normal(0.0, 0.02));
                routing_lines[idx].yaw.push_back(pos.z());
            }
        }
    }
    std::vector<RoutingLine> obs_prediction(routing_lines.begin() + 1, routing_lines.end());

    SPDLOG_INFO("Using Path Model (5D) CILQR Solver with trajectory_smoother");

    cilqr::AlgorithmConfig algo_config = utils::create_algo_config(config);
    
    // 保留原有的cilqr对象用于备用（如果需要）
    // ceshi::planning::PathConstrainedIterLqrDeciderAML cilqr("motion planning cilqr", algo_config);
    // SPDLOG_INFO("PathConstrainedIterLqrDeciderAML created successfully");

    Vector5d ego_state_5d;
    double initial_s = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < center_lines[0].size(); ++i) {
        double dist = hypot(center_lines[0].x[i] - initial_conditions[0][0],
                           center_lines[0].y[i] - initial_conditions[0][1]);
        if (dist < min_dist) {
            min_dist = dist;
            initial_s = center_lines[0].longitude[i];
        }
    }
    double initial_kappa = utils::calc_kappa_from_reference(center_lines[0], initial_s);
    SPDLOG_DEBUG("Initial kappa calculated: {}", initial_kappa);
    ego_state_5d << initial_conditions[0][0], initial_conditions[0][1],
                    initial_conditions[0][2], initial_conditions[0][3], initial_kappa;
    SPDLOG_DEBUG("Initial 5D state: x={}, y={}, v={}, heading={}, kappa={}", 
                 ego_state_5d[0], ego_state_5d[1], ego_state_5d[2], 
                 ego_state_5d[3], ego_state_5d[4]);

    std::vector<double> time_history;
    std::vector<double> x_history;
    std::vector<double> y_history;
    std::vector<double> velocity_history;
    std::vector<double> acceleration_history;
    std::vector<double> jerk_history;
    std::vector<double> kappa_history;
    double last_acceleration = 0.0;

    plt::figure_size(2000, 800);
    
    exec_python("import matplotlib.pyplot as plt");
    exec_python("fig = plt.gcf()");
    exec_python("fig.clear()");
    
    exec_python("ax_main = plt.subplot2grid((5, 4), (0, 0), 5, 2)");
    exec_python("ax_path = plt.subplot2grid((5, 4), (0, 2), 1, 1)");
    exec_python("ax_vel = plt.subplot2grid((5, 4), (1, 2), 1, 1)");
    exec_python("ax_acc = plt.subplot2grid((5, 4), (2, 2), 1, 1)");
    exec_python("ax_jerk = plt.subplot2grid((5, 4), (3, 2), 1, 1)");
    exec_python("ax_kappa = plt.subplot2grid((5, 4), (4, 2), 1, 1)");
    exec_python("ax_path_pred = plt.subplot2grid((5, 4), (0, 3), 1, 1)");
    exec_python("ax_vel_pred = plt.subplot2grid((5, 4), (1, 3), 1, 1)");
    exec_python("ax_acc_pred = plt.subplot2grid((5, 4), (2, 3), 1, 1)");
    exec_python("ax_jerk_pred = plt.subplot2grid((5, 4), (3, 3), 1, 1)");
    exec_python("ax_kappa_pred = plt.subplot2grid((5, 4), (4, 3), 1, 1)");
    exec_python("plt.subplots_adjust(left=0.03, right=0.97, top=0.95, bottom=0.05, wspace=0.25, hspace=0.3)");

    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        size_t index = t / delta_t;
        
        exec_python("plt.sca(ax_main)");
        plt::cla();
        for (size_t i = 0; i < borders.size(); ++i) {
            if (i == 0 || i == borders.size() - 1) {
                plt::plot(borders[i].x, borders[i].y, {{"linewidth", "2"}, {"color", "k"}});
            } else {
                plt::plot(borders[i].x, borders[i].y, "-k");
            }
        }
        for (size_t i = 0; i < center_lines.size(); ++i) {
            plt::plot(center_lines[i].x, center_lines[i].y, "--k");
        }

        Eigen::MatrixX2d new_u;
        MatrixX5d new_x_5d;
        Vector5d ego_state_current_5d;
        
       
        cilqr::ReferenceLine algo_ref = utils::convert_to_algo_ref(center_lines[0]);
        std::vector<cilqr::RoutingLine> algo_obs = utils::convert_to_algo_routing(
            utils::get_sub_routing_lines(obs_prediction, index));
        
        
        // 1. 创建 InsidePlannerData
        ceshi::planning::InsidePlannerData inside_data;
        inside_data.vel_x = ego_state_5d[0];
        inside_data.vel_y = ego_state_5d[1];
        inside_data.vel_v = ego_state_5d[2];
        inside_data.vel_heading = ego_state_5d[3];
        inside_data.vel_steer_angle = 0.0;
        inside_data.init_sl_point.set_s(0.0);
        inside_data.init_sl_point.set_l(0.0);
        
        // 2. 创建 raw_trajectory_points（从参考线生成初始轨迹）
        // 首先找到参考线上最接近当前车辆位置的点
        size_t start_idx = 0;
        double min_dist_to_ego = std::numeric_limits<double>::max();
        for (size_t i = 0; i < algo_ref.size(); ++i) {
            double dist = std::hypot(algo_ref.x[i] - ego_state_5d[0], algo_ref.y[i] - ego_state_5d[1]);
            if (dist < min_dist_to_ego) {
                min_dist_to_ego = dist;
                start_idx = i;
            }
        }
        
        SPDLOG_INFO("Found closest point on reference line: start_idx={}, min_dist={:.3f}", start_idx, min_dist_to_ego);
        
        std::vector<ceshi::planning::TrajectoryPoint> raw_trajectory_points;
        // 从当前位置开始生成轨迹，但至少保留一定数量的点
        size_t num_points = std::min(static_cast<size_t>(50), algo_ref.size() - start_idx);
        raw_trajectory_points.reserve(num_points);
        
        // 第一个点使用当前车辆状态
        ceshi::planning::TrajectoryPoint first_point;
        first_point.set_x(ego_state_5d[0]);
        first_point.set_y(ego_state_5d[1]);
        first_point.set_theta(ego_state_5d[3]);
        first_point.set_kappa(ego_state_5d[4]);
        first_point.set_velocity(ego_state_5d[2]);
        first_point.set_relative_time(0.0);
        first_point.set_acceleration(0.0);
        raw_trajectory_points.push_back(first_point);
        
        size_t ref_start_idx = (min_dist_to_ego < 0.1 && start_idx + 1 < algo_ref.size()) ? start_idx + 1 : start_idx;
        for (size_t i = ref_start_idx; i < algo_ref.size() && raw_trajectory_points.size() < num_points; ++i) {
            ceshi::planning::TrajectoryPoint point;
            point.set_x(algo_ref.x[i]);
            point.set_y(algo_ref.y[i]);
            point.set_theta(algo_ref.yaw[i]);
            
            // 计算曲率
            double kappa = 0.0;
            if (i > 0 && i < algo_ref.size() - 1) {
                double dx1 = algo_ref.x[i] - algo_ref.x[i-1];
                double dy1 = algo_ref.y[i] - algo_ref.y[i-1];
                double dx2 = algo_ref.x[i+1] - algo_ref.x[i];
                double dy2 = algo_ref.y[i+1] - algo_ref.y[i];
                double dist1 = std::hypot(dx1, dy1);
                double dist2 = std::hypot(dx2, dy2);
                if (dist1 > 1e-6 && dist2 > 1e-6) {
                    double heading1 = std::atan2(dy1, dx1);
                    double heading2 = std::atan2(dy2, dx2);
                    double dheading = heading2 - heading1;
                    while (dheading > M_PI) dheading -= 2 * M_PI;
                    while (dheading < -M_PI) dheading += 2 * M_PI;
                    double avg_dist = (dist1 + dist2) / 2.0;
                    if (avg_dist > 1e-6) {
                        kappa = dheading / avg_dist;
                    }
                }
            }
            point.set_kappa(kappa);
            point.set_velocity(target_velocity);
            
            // 计算相对时间（从第一个点开始累积）
            double dt = 0.1; // 默认时间步长
            if (i > ref_start_idx && algo_ref.longitude.size() > i && i > 0) {
                // 使用参考线的纵向距离计算时间
                double ds = algo_ref.longitude[i] - algo_ref.longitude[i-1];
                if (target_velocity > 1e-6 && ds > 1e-6) {
                    dt = ds / target_velocity;
                }
            } else if (i == ref_start_idx && raw_trajectory_points.size() > 0) {
                // 第一个参考点的时间基于从车辆位置到该点的距离
                double ds = std::hypot(algo_ref.x[i] - ego_state_5d[0], algo_ref.y[i] - ego_state_5d[1]);
                if (target_velocity > 1e-6 && ds > 1e-6) {
                    dt = ds / target_velocity;
                }
            }
            double relative_time = raw_trajectory_points.back().relative_time() + dt;
            point.set_relative_time(relative_time);
            point.set_acceleration(0.0);
            
            raw_trajectory_points.push_back(point);
        }
        
        SPDLOG_INFO("Generated {} raw_trajectory_points, first point: [{:.3f}, {:.3f}], second point: [{:.3f}, {:.3f}]", 
                    raw_trajectory_points.size(),
                    raw_trajectory_points[0].x(), raw_trajectory_points[0].y(),
                    raw_trajectory_points.size() > 1 ? raw_trajectory_points[1].x() : 0.0,
                    raw_trajectory_points.size() > 1 ? raw_trajectory_points[1].y() : 0.0);
        
        std::vector<ceshi::planning::ReferencePoint> ref_points;
        ref_points.reserve(algo_ref.size());
        
        for (size_t i = 0; i < algo_ref.size(); ++i) {
            ceshi::planning::ReferencePoint ref_pt(
                algo_ref.x[i], algo_ref.y[i], algo_ref.yaw[i],
                0.0, 0.0, 0.0, algo_ref.longitude[i]
            );
            ref_pt.set_left_bound(road_borders[0]);
            ref_pt.set_right_bound(road_borders[1]);
            ref_points.push_back(ref_pt);
        }
        ceshi::planning::ReferenceLine reference_line(ref_points);
        
        ceshi::planning::ThirdOrderSplinePath::State init_state;
        init_state.state_l0 = 0.0;
        init_state.state_dl = 0.0;
        init_state.state_ddl = 0.0;
        
        std::vector<double> knots_delta_s;
        std::vector<ceshi::planning::PieceBoundary> piece_boundaries;
        std::vector<ceshi::planning::PieceBoundary> orin_boundaries;
        
        if (algo_ref.longitude.size() > 1) {
            knots_delta_s.reserve(algo_ref.longitude.size() - 1);
            for (size_t i = 1; i < algo_ref.longitude.size(); ++i) {
                knots_delta_s.push_back(algo_ref.longitude[i] - algo_ref.longitude[i-1]);
            }
        }
        
        std::vector<ceshi::planning::TrajectoryPoint> computed_trajectory_points;
        
        if (inside_data.vel_v >= 0.5) {
            SPDLOG_INFO("raw_trajectory_points size:{}", raw_trajectory_points.size());
            
            ceshi::planning::PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
            ceshi::planning::ErrorCode error_code = cilqr.Execute(init_state, inside_data, raw_trajectory_points, 
                                                 knots_delta_s, piece_boundaries, orin_boundaries, 
                                                 reference_line);
            
            if (error_code == ceshi::planning::ErrorCode::PLANNING_OK) {
                std::vector<ceshi::planning::TrajectoryPoint> rough_trajectory = raw_trajectory_points;
                bool success = cilqr.get_xystate_seqs(rough_trajectory);
                if (success) {
                    computed_trajectory_points = rough_trajectory;
                    SPDLOG_INFO("zzz_lqr:输出结果是lqr优化的结果");
                } else {
                    SPDLOG_WARN("get_xystate_seqs failed, using raw trajectory");
                    computed_trajectory_points = raw_trajectory_points;
                }
            } else {
                SPDLOG_WARN("CILQR Execute failed, using raw trajectory");
                computed_trajectory_points = raw_trajectory_points;
            }
        } else {
            computed_trajectory_points = raw_trajectory_points;
            SPDLOG_INFO("zzz_lqr:输出结果是直接使用粗解 (vel_v={:.3f} < 0.5)", inside_data.vel_v);
        }
        
        // 6. 计算 s 值（累积距离）
        std::vector<ceshi::planning::TrajectoryPoint> final_trajectory_points;
        if (!computed_trajectory_points.empty()) {
            final_trajectory_points.reserve(computed_trajectory_points.size());
            double accumulated_s = 0.0;
            
            ceshi::planning::TrajectoryPoint first_point = computed_trajectory_points[0];
            first_point.mutable_path_point()->set_s(accumulated_s);
            final_trajectory_points.push_back(first_point);
            
            for (size_t i = 1; i < computed_trajectory_points.size(); ++i) {
                const auto& prev_point = final_trajectory_points.back().path_point();
                const auto& raw_curr_point = computed_trajectory_points[i];
                
                double dist = std::sqrt(std::pow(raw_curr_point.path_point().x() - prev_point.x(), 2) +
                                        std::pow(raw_curr_point.path_point().y() - prev_point.y(), 2));
                accumulated_s += dist;
                
                ceshi::planning::TrajectoryPoint new_point = raw_curr_point;
                new_point.mutable_path_point()->set_s(accumulated_s);
                final_trajectory_points.push_back(new_point);
            }
        }
        
        std::vector<ceshi::planning::TrajectoryPoint>& optimized_trajectory = 
            final_trajectory_points.empty() ? computed_trajectory_points : final_trajectory_points;
        
        // 7. 将TrajectoryPoint转换为MatrixX5d和MatrixX2d格式
        size_t traj_size = optimized_trajectory.size();
        new_x_5d = MatrixX5d::Zero(traj_size, 5);
        new_u = Eigen::MatrixX2d::Zero(traj_size > 0 ? traj_size - 1 : 0, 2);
        
        for (size_t i = 0; i < traj_size; ++i) {
            new_x_5d(i, 0) = optimized_trajectory[i].x();
            new_x_5d(i, 1) = optimized_trajectory[i].y();
            new_x_5d(i, 2) = optimized_trajectory[i].velocity();
            new_x_5d(i, 3) = optimized_trajectory[i].theta();
            new_x_5d(i, 4) = optimized_trajectory[i].kappa();
            
            // 计算控制输入（加速度和曲率变化率）
            if (i > 0 && i - 1 < new_u.rows()) {
                double dt = optimized_trajectory[i].relative_time() - optimized_trajectory[i-1].relative_time();
                if (dt > 1e-6) {
                    new_u(i-1, 0) = optimized_trajectory[i].acceleration();
                    new_u(i-1, 1) = (optimized_trajectory[i].kappa() - optimized_trajectory[i-1].kappa()) / dt;
                } else {
                    new_u(i-1, 0) = 0.0;
                    new_u(i-1, 1) = 0.0;
                }
            }
        }
        // 找到轨迹中 relative_time 最接近 delta_t 的点来更新状态
        // 这样可以确保车辆按照正确的时间步长移动
        if (new_x_5d.rows() > 1) {
            Vector5d old_state = ego_state_5d;
            
            // 找到 relative_time 最接近 delta_t 的点
            size_t best_idx = 1; // 默认使用第二个点
            double min_time_diff = std::abs(optimized_trajectory[1].relative_time() - optimized_trajectory[0].relative_time() - delta_t);
            
            for (size_t i = 1; i < optimized_trajectory.size(); ++i) {
                double time_diff = std::abs(optimized_trajectory[i].relative_time() - optimized_trajectory[0].relative_time() - delta_t);
                if (time_diff < min_time_diff) {
                    min_time_diff = time_diff;
                    best_idx = i;
                }
            }
            
            // 使用找到的点更新状态
            ego_state_current_5d = new_x_5d.row(best_idx).transpose();
            ego_state_5d = ego_state_current_5d;
            
            double actual_dt = optimized_trajectory[best_idx].relative_time() - optimized_trajectory[0].relative_time();
            double dist_moved = std::hypot(ego_state_5d[0] - old_state[0], ego_state_5d[1] - old_state[1]);
            double expected_dist = old_state[2] * actual_dt; // 基于速度和时间计算期望距离
            
            SPDLOG_INFO("Using trajectory point[{}] with relative_time={:.3f} (delta_t={:.3f}), dist_moved={:.3f}, expected_dist={:.3f}, velocity={:.3f}", 
                        best_idx, actual_dt, delta_t, dist_moved, expected_dist, ego_state_5d[2]);
        } else if (new_x_5d.rows() == 1) {
            // 如果只有一个点，使用它
            ego_state_current_5d = new_x_5d.row(0).transpose();
            ego_state_5d = ego_state_current_5d;
            SPDLOG_WARN("Only one trajectory point available, vehicle may not move");
        } else {
            // 如果轨迹为空，保持当前状态不变
            ego_state_current_5d = ego_state_5d;
            SPDLOG_WARN("Empty trajectory, vehicle state unchanged");
        }
        
        Eigen::MatrixX4d new_x_4d = Eigen::MatrixX4d::Zero(new_x_5d.rows(), 4);
        new_x_4d << new_x_5d.block(0, 0, new_x_5d.rows(), 2),
                   new_x_5d.col(2),
                   new_x_5d.col(3);
        Eigen::Vector4d ego_state_current;
        ego_state_current << ego_state_current_5d[0], ego_state_current_5d[1],
                            ego_state_current_5d[2], ego_state_current_5d[3];

        Eigen::MatrixX4d boarder = utils::get_boundary(new_x_4d, VEHICLE_WIDTH * 0.7);
        std::vector<std::vector<double>> closed_curve = utils::get_closed_curve(boarder);
        plt::plot(closed_curve[0], closed_curve[1]);

        Eigen::Vector4d ego_viz;
        ego_viz << ego_state_current_5d[0], ego_state_current_5d[1],
                  ego_state_current_5d[2], ego_state_current_5d[3];
        utils::plot_vehicle(outlook_ego, ego_viz, vehicle_para, reference_point, VEHICLE_HEIGHT);
        
        for (size_t idx = 1; idx < vehicle_num; ++idx) {
            utils::plot_vehicle(outlook_agent, routing_lines[idx][index], vehicle_para,
                                reference_point, 0);
        }

        if (show_obstacle_boundary) {
            Eigen::Matrix3Xd cur_obstacle_states =
                utils::get_cur_obstacle_states(routing_lines, index);
            Eigen::Vector4d ego_for_boundary{ego_state_current_5d[0], ego_state_current_5d[1],
                                             ego_state_current_5d[2], ego_state_current_5d[3]};
            utils::plot_obstacle_boundary(ego_for_boundary, cur_obstacle_states, obs_attr,
                                          VEHICLE_HEIGHT, reference_point);
        }

        if (show_reference_line) {
            plt::plot(center_lines[0].x, center_lines[0].y, "-r");
        }

        double ego_x = ego_state_current_5d[0];
        double ego_y = ego_state_current_5d[1];
        double visual_x_min = ego_x - 10;
        double visual_y_min = ego_y - 5;
        double visual_x_max = ego_x + 30;
        double visual_y_max = ego_y + 15;
        if (hypot(visual_x_limit[0], visual_x_limit[1]) > 1e-3) {
            visual_x_min = visual_x_limit[0];
            visual_x_max = visual_x_limit[1];
        }
        if (hypot(visual_y_limit[0], visual_y_limit[1]) > 1e-3) {
            visual_y_min = visual_y_limit[0];
            visual_y_max = visual_y_limit[1];
        }

        double control_display = new_u.row(0)[1];
        std::vector<double> outlook_steer_pos = {visual_x_min + steer_size / 1.5,
                                                 visual_y_max - steer_size / 1.5,
                                                 control_display};
        utils::imshow(outlook_steering, outlook_steer_pos, {steer_size, steer_size});
        double acc = new_u.row(0)[0] > 0 ? new_u.row(0)[0] : 0;
        double brake = new_u.row(0)[0] > 0 ? 0 : -new_u.row(0)[0];
        double bar_bottom = visual_y_max - steer_size;
        double bar_left = visual_x_min + steer_size * 1.3;
        std::vector<double> acc_bar_x = {bar_left, bar_left + 1, bar_left + 1, bar_left};
        std::vector<double> acc_bar_y = {bar_bottom, bar_bottom,
                                         bar_bottom + steer_size * (acc / ACC_MAX),
                                         bar_bottom + steer_size * (acc / ACC_MAX)};
        std::vector<double> brake_bar_x = {bar_left + 2, bar_left + 3, bar_left + 3, bar_left + 2};
        std::vector<double> brake_bar_y = {bar_bottom, bar_bottom,
                                           bar_bottom + steer_size * (brake / ACC_MAX),
                                           bar_bottom + steer_size * (brake / ACC_MAX)};
        plt::fill(acc_bar_x, acc_bar_y, {{"color", "red"}});
        plt::fill(brake_bar_x, brake_bar_y, {{"color", "gray"}});
        double text_left = bar_left + 4.5;
        double text_top = visual_y_max - 1.5;
        
        plt::text(text_left, text_top, fmt::format("x = {:.2f} m", ego_state_current_5d[0]),
                  {{"color", "black"}});
        plt::text(text_left, text_top - 1.5, fmt::format("y = {:.2f} m", ego_state_current_5d[1]),
                  {{"color", "black"}});
        plt::text(text_left, text_top - 3, fmt::format("v = {:.2f} m/s", ego_state_current_5d[2]),
                  {{"color", "black"}});
        plt::text(text_left, text_top - 4.5, fmt::format("heading = {:.2f} rad", ego_state_current_5d[3]),
                  {{"color", "black"}});
        plt::text(text_left, text_top - 6, fmt::format("kappa = {:.3f}", ego_state_current_5d[4]),
                  {{"color", "black"}});
        plt::text(text_left + 10, text_top, fmt::format("acc = {:.2f}", new_u.row(0)[0]),
                  {{"color", "black"}});
        plt::text(text_left + 10, text_top - 1.5, fmt::format("dkappa = {:.3f}", new_u.row(0)[1]),
                  {{"color", "black"}});

        plt::xlim(visual_x_min, visual_x_max);
        plt::ylim(visual_y_min, visual_y_max);

        time_history.push_back(t);
        x_history.push_back(ego_state_current_5d[0]);
        y_history.push_back(ego_state_current_5d[1]);
        velocity_history.push_back(ego_state_current_5d[2]);
        double current_acceleration = new_u.row(0)[0];
        acceleration_history.push_back(current_acceleration);
        
        double current_jerk = 0.0;
        if (time_history.size() > 1) {
            current_jerk = (current_acceleration - last_acceleration) / delta_t;
        }
        jerk_history.push_back(current_jerk);
        last_acceleration = current_acceleration;
        
        kappa_history.push_back(ego_state_current_5d[4]);

        if (time_history.size() > 1) {
            exec_python("plt.sca(ax_path)");
            plt::cla();
            plt::plot(x_history, y_history, {{"color", "blue"}, {"linewidth", "1.5"}});
            std::vector<double> current_x = {x_history.back()};
            std::vector<double> current_y = {y_history.back()};
            plt::plot(current_x, current_y, {{"marker", "o"}, {"color", "red"}, {"markersize", "6"}, {"linestyle", "none"}});
            
            for (size_t idx = 1; idx < vehicle_num; ++idx) {
                if (index < routing_lines[idx].x.size() && index < routing_lines[idx].y.size()) {
                    std::vector<double> obs_x_history(routing_lines[idx].x.begin(), routing_lines[idx].x.begin() + index + 1);
                    std::vector<double> obs_y_history(routing_lines[idx].y.begin(), routing_lines[idx].y.begin() + index + 1);
                    plt::plot(obs_x_history, obs_y_history, {{"color", "gray"}, {"linewidth", "1.0"}, {"linestyle", "--"}});
                    std::vector<double> obs_current_x = {routing_lines[idx].x[index]};
                    std::vector<double> obs_current_y = {routing_lines[idx].y[index]};
                    plt::plot(obs_current_x, obs_current_y, {{"marker", "s"}, {"color", "orange"}, {"markersize", "5"}, {"linestyle", "none"}});
                }
            }
            
            double y_min = road_borders[1] - 1.0;
            double y_max = road_borders[0] + 1.0;
            plt::ylim(y_min, y_max);
            if (x_history.size() > 0) {
                double x_min = *std::min_element(x_history.begin(), x_history.end()) - 5.0;
                double x_max = *std::max_element(x_history.begin(), x_history.end()) + 5.0;
                for (size_t idx = 1; idx < vehicle_num; ++idx) {
                    if (index < routing_lines[idx].x.size()) {
                        double obs_x = routing_lines[idx].x[index];
                        x_min = std::min(x_min, obs_x - 5.0);
                        x_max = std::max(x_max, obs_x + 5.0);
                    }
                }
                plt::xlim(x_min, x_max);
            }
            plt::xlabel("X (m)", {{"fontsize", "7"}});
            plt::ylabel("Y (m)", {{"fontsize", "7"}});
            plt::grid(true);

            exec_python("plt.sca(ax_vel)");
            plt::cla();
            plt::plot(time_history, velocity_history, {{"color", "purple"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("v (m/s)", {{"fontsize", "7"}});
            plt::grid(true);

            exec_python("plt.sca(ax_acc)");
            plt::cla();
            plt::plot(time_history, acceleration_history, {{"color", "brown"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("a (m/s²)", {{"fontsize", "7"}});
            plt::grid(true);

            exec_python("plt.sca(ax_jerk)");
            plt::cla();
            plt::plot(time_history, jerk_history, {{"color", "orange"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("jerk (m/s³)", {{"fontsize", "7"}});
            plt::grid(true);

            exec_python("plt.sca(ax_kappa)");
            plt::cla();
            plt::plot(time_history, kappa_history, {{"color", "red"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("κ (1/m)", {{"fontsize", "7"}});
            plt::grid(true);
        }

        if (new_x_5d.rows() > 1 && new_u.rows() > 0) {
            std::vector<double> pred_time;
            std::vector<double> pred_x;
            std::vector<double> pred_y;
            std::vector<double> pred_velocity;
            std::vector<double> pred_acceleration;
            std::vector<double> pred_jerk;
            std::vector<double> pred_kappa;

            for (int i = 1; i < new_x_5d.rows(); ++i) {
                pred_time.push_back(t + (i - 1) * delta_t);
                pred_x.push_back(new_x_5d(i, 0));
                pred_y.push_back(new_x_5d(i, 1));
                pred_velocity.push_back(new_x_5d(i, 2));
                pred_kappa.push_back(new_x_5d(i, 4));
                
                if (i - 1 < new_u.rows()) {
                    pred_acceleration.push_back(new_u(i - 1, 0));
                }
            }

            if (pred_acceleration.size() > 1) {
                pred_jerk.push_back(0.0);
                for (size_t i = 1; i < pred_acceleration.size(); ++i) {
                    double jerk = (pred_acceleration[i] - pred_acceleration[i-1]) / delta_t;
                    pred_jerk.push_back(jerk);
                }
            } else if (pred_acceleration.size() == 1) {
                pred_jerk.push_back(0.0);
            }

            if (pred_time.size() > 0) {
                exec_python("plt.sca(ax_path_pred)");
                plt::cla();
                plt::plot(pred_x, pred_y, {{"color", "green"}, {"linewidth", "1.5"}, {"linestyle", "--"}});
                std::vector<double> start_x = {ego_state_current_5d[0]};
                std::vector<double> start_y = {ego_state_current_5d[1]};
                plt::plot(start_x, start_y, {{"marker", "o"}, {"color", "red"}, {"markersize", "6"}, {"linestyle", "none"}});
                double y_min = road_borders[1] - 1.0;
                double y_max = road_borders[0] + 1.0;
                plt::ylim(y_min, y_max);
                if (pred_x.size() > 0) {
                    double x_min = *std::min_element(pred_x.begin(), pred_x.end()) - 5.0;
                    double x_max = *std::max_element(pred_x.begin(), pred_x.end()) + 5.0;
                    x_min = std::min(x_min, ego_state_current_5d[0] - 5.0);
                    x_max = std::max(x_max, ego_state_current_5d[0] + 5.0);
                    plt::xlim(x_min, x_max);
                }
                plt::xlabel("X (m)", {{"fontsize", "7"}});
                plt::ylabel("Y (m)", {{"fontsize", "7"}});
                plt::grid(true);

                exec_python("plt.sca(ax_vel_pred)");
                plt::cla();
                plt::plot(pred_time, pred_velocity, {{"color", "green"}, {"linewidth", "1.5"}, {"linestyle", "--"}});
                plt::xlabel("Time (s)", {{"fontsize", "7"}});
                plt::ylabel("v (m/s)", {{"fontsize", "7"}});
                plt::grid(true);

                exec_python("plt.sca(ax_acc_pred)");
                plt::cla();
                if (pred_time.size() == pred_acceleration.size()) {
                    plt::plot(pred_time, pred_acceleration, {{"color", "green"}, {"linewidth", "1.5"}, {"linestyle", "--"}});
                }
                plt::xlabel("Time (s)", {{"fontsize", "7"}});
                plt::ylabel("a (m/s²)", {{"fontsize", "7"}});
                plt::grid(true);

                exec_python("plt.sca(ax_jerk_pred)");
                plt::cla();
                if (pred_time.size() == pred_jerk.size() && pred_jerk.size() > 0) {
                    plt::plot(pred_time, pred_jerk, {{"color", "green"}, {"linewidth", "1.5"}, {"linestyle", "--"}});
                }
                plt::xlabel("Time (s)", {{"fontsize", "7"}});
                plt::ylabel("jerk (m/s³)", {{"fontsize", "7"}});
                plt::grid(true);

                exec_python("plt.sca(ax_kappa_pred)");
                plt::cla();
                if (pred_time.size() == pred_kappa.size()) {
                    plt::plot(pred_time, pred_kappa, {{"color", "green"}, {"linewidth", "1.5"}, {"linestyle", "--"}});
                }
                plt::xlabel("Time (s)", {{"fontsize", "7"}});
                plt::ylabel("κ (1/m)", {{"fontsize", "7"}});
                plt::grid(true);
            }
        }

        plt::pause(delta_t);
    }

    config->destroy_instance();
    plt::show();

    return 0;
}
