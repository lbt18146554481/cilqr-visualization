/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-10-30 00:05:14
 * @LastEditTime: 2025-03-06 22:07:49
 * @FilePath: /toy-example-of-iLQR/src/motion_planning.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "cilqr_adapter.hpp"
#include "cubic_spline.hpp"
#include "global_config.hpp"
#include "matplotlibcpp.h"

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
    // 路径模型可能没有wheelbase配置，使用length代替
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
            // The current laneline does not have the attribute of driving direction,
            // and it is simply deduced by the yaw in the initial condition.
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

            // randomly add noise to other cars
            // TODO: the current planning results are very sensitive to noise and
            //       initial conditions, which need to be optimized.
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

    // 只使用5D路径模型
    SPDLOG_INFO("Using Path Model (5D) CILQR Solver");

    // 初始化适配器（连接算法层和可视化层）
    std::unique_ptr<CILQRAdapter> cilqr_adapter = nullptr;
    try {
        cilqr_adapter = std::make_unique<CILQRAdapter>(config);
        SPDLOG_INFO("CILQRAdapter created successfully");
    } catch (const std::exception& e) {
        SPDLOG_ERROR("Failed to create CILQRAdapter: {}", e.what());
        return -1;
    } catch (...) {
        SPDLOG_ERROR("Failed to create CILQRAdapter: unknown exception");
        return -1;
    }

    // 初始化5D状态（路径模型）
    Vector5d ego_state_5d;
    // 计算初始kappa（从参考线）
    double initial_s = 0.0;
    // 找到最近的参考点
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

    // 历史数据存储（用于右侧实时图表）
    std::vector<double> time_history;
    std::vector<double> x_history;
    std::vector<double> y_history;
    std::vector<double> velocity_history;
    std::vector<double> acceleration_history;
    std::vector<double> jerk_history;
    std::vector<double> kappa_history;
    double last_acceleration = 0.0;  // 用于计算jerk

    // 设置图形大小（更大的窗口以容纳左右两个区域）
    // figure_size 会自动创建 figure，不需要再调用 figure()
    plt::figure_size(1600, 800);
    
    // 使用 Python 直接执行 subplot 命令来设置布局（只在初始化时调用一次）
    // 使用 subplot2grid 创建更灵活的布局：
    // - 左侧大图：占据前2列的所有行（占据大部分空间）
    // - 右侧小图：占据第3列，分成5个小图
    exec_python("import matplotlib.pyplot as plt");
    exec_python("fig = plt.gcf()");
    exec_python("fig.clear()");  // 清除图形
    
    // 预先创建所有 subplot（只调用一次，避免在循环中重复调用）
    // 调整布局：左侧动画往左移动一点（减少右侧空间，增加左侧空间）
    exec_python("ax_main = plt.subplot2grid((5, 3), (0, 0), 5, 2)");
    exec_python("ax_path = plt.subplot2grid((5, 3), (0, 2), 1, 1)");
    exec_python("ax_vel = plt.subplot2grid((5, 3), (1, 2), 1, 1)");
    exec_python("ax_acc = plt.subplot2grid((5, 3), (2, 2), 1, 1)");
    exec_python("ax_jerk = plt.subplot2grid((5, 3), (3, 2), 1, 1)");
    exec_python("ax_kappa = plt.subplot2grid((5, 3), (4, 2), 1, 1)");
    // 调整子图间距，让左侧动画往左移动
    exec_python("plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, wspace=0.3, hspace=0.3)");

    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        size_t index = t / delta_t;
        
        // 左侧：原有动画（使用预先创建的 subplot，避免重复调用 Python）
        exec_python("plt.sca(ax_main)");  // 切换到主图（比 subplot2grid 快）
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
        
        // 使用适配器调用5D算法
        auto [u, x] = cilqr_adapter->solve(ego_state_5d, center_lines[0], target_velocity,
                                          utils::get_sub_routing_lines(obs_prediction, index), road_borders);
        new_u = u;
        new_x_5d = x;
        ego_state_current_5d = new_x_5d.row(1).transpose();
        ego_state_5d = ego_state_current_5d;
        
        // 转换为4维用于可视化
        Eigen::MatrixX4d new_x_4d = Eigen::MatrixX4d::Zero(new_x_5d.rows(), 4);
        new_x_4d << new_x_5d.block(0, 0, new_x_5d.rows(), 2),
                   new_x_5d.col(2),
                   new_x_5d.col(3);
        Eigen::Vector4d ego_state_current;
        ego_state_current << ego_state_current_5d[0], ego_state_current_5d[1],
                            ego_state_current_5d[2], ego_state_current_5d[3];

        Eigen::MatrixX4d boarder = utils::get_boundary(new_x_4d, VEHICLE_WIDTH * 0.7);
        std::vector<std::vector<double>> closed_curve = utils::get_closed_curve(boarder);
        // plt::fill(closed_curve[0], closed_curve[1], {{"color", "cyan"}, {"alpha", "0.7"}});// 2选1-1画自车轨迹(绿色条)
        plt::plot(closed_curve[0], closed_curve[1]); // 2选1-2画自车轨迹(线段形式)

        // 画自车（使用当前状态）
        // 对于5维状态，只使用前4维进行可视化
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

        // default figure x-y limit
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

        // 控制量显示（路径模型使用dkappa）
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
        
        // 状态显示（5D路径模型）
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
        // 移除标题

        // 收集历史数据
        time_history.push_back(t);
        x_history.push_back(ego_state_current_5d[0]);
        y_history.push_back(ego_state_current_5d[1]);
        velocity_history.push_back(ego_state_current_5d[2]);
        double current_acceleration = new_u.row(0)[0];
        acceleration_history.push_back(current_acceleration);
        
        // 计算jerk（加速度的变化率）
        double current_jerk = 0.0;
        if (time_history.size() > 1) {
            current_jerk = (current_acceleration - last_acceleration) / delta_t;
        }
        jerk_history.push_back(current_jerk);
        last_acceleration = current_acceleration;
        
        kappa_history.push_back(ego_state_current_5d[4]);

        // 右侧：实时图表（使用预先创建的 subplot，避免重复调用 Python）
        if (time_history.size() > 1) {
            // 1. 路径图（x-y轨迹）
            exec_python("plt.sca(ax_path)");  // 切换到路径图（比 subplot2grid 快）
            plt::cla();
            
            // 绘制自车轨迹（蓝色）
            plt::plot(x_history, y_history, {{"color", "blue"}, {"linewidth", "1.5"}});
            std::vector<double> current_x = {x_history.back()};
            std::vector<double> current_y = {y_history.back()};
            plt::plot(current_x, current_y, {{"marker", "o"}, {"color", "red"}, {"markersize", "6"}, {"linestyle", "none"}});
            
            // 绘制障碍物轨迹和当前位置
            for (size_t idx = 1; idx < vehicle_num; ++idx) {
                if (index < routing_lines[idx].x.size() && index < routing_lines[idx].y.size()) {
                    // 绘制障碍物的历史轨迹（从开始到当前时间）
                    std::vector<double> obs_x_history(routing_lines[idx].x.begin(), routing_lines[idx].x.begin() + index + 1);
                    std::vector<double> obs_y_history(routing_lines[idx].y.begin(), routing_lines[idx].y.begin() + index + 1);
                    plt::plot(obs_x_history, obs_y_history, {{"color", "gray"}, {"linewidth", "1.0"}, {"linestyle", "--"}});
                    
                    // 绘制障碍物当前位置（用三角形标记）
                    std::vector<double> obs_current_x = {routing_lines[idx].x[index]};
                    std::vector<double> obs_current_y = {routing_lines[idx].y[index]};
                    plt::plot(obs_current_x, obs_current_y, {{"marker", "s"}, {"color", "orange"}, {"markersize", "5"}, {"linestyle", "none"}});
                }
            }
            
            // 设置 path 图表的上下界为车道界限位置
            // road_borders[0] 是左边界，road_borders[1] 是右边界
            double y_min = road_borders[1] - 1.0;  // 右边界下方留一点空间
            double y_max = road_borders[0] + 1.0;  // 左边界上方留一点空间
            plt::ylim(y_min, y_max);
            // X 轴范围根据数据自动调整，但可以设置一个合理的范围
            if (x_history.size() > 0) {
                double x_min = *std::min_element(x_history.begin(), x_history.end()) - 5.0;
                double x_max = *std::max_element(x_history.begin(), x_history.end()) + 5.0;
                // 考虑障碍物的 x 范围
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

            // 2. 速度图
            exec_python("plt.sca(ax_vel)");
            plt::cla();
            plt::plot(time_history, velocity_history, {{"color", "purple"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("v (m/s)", {{"fontsize", "7"}});
            plt::grid(true);

            // 3. 加速度图
            exec_python("plt.sca(ax_acc)");
            plt::cla();
            plt::plot(time_history, acceleration_history, {{"color", "brown"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("a (m/s²)", {{"fontsize", "7"}});
            plt::grid(true);

            // 4. Jerk图
            exec_python("plt.sca(ax_jerk)");
            plt::cla();
            plt::plot(time_history, jerk_history, {{"color", "orange"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("jerk (m/s³)", {{"fontsize", "7"}});
            plt::grid(true);

            // 5. 曲率图
            exec_python("plt.sca(ax_kappa)");
            plt::cla();
            plt::plot(time_history, kappa_history, {{"color", "red"}, {"linewidth", "1.5"}});
            plt::xlabel("Time (s)", {{"fontsize", "7"}});
            plt::ylabel("κ (1/m)", {{"fontsize", "7"}});
            plt::grid(true);
        }

        plt::pause(delta_t);
    }

    config->destroy_instance();
    plt::show();

    return 0;
}
