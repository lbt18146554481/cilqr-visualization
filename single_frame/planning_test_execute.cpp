#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <json/json.h>

#include "config/planning_config_manager.h"
#include "src/common/reference_line/reference_line.h"
#include "src/common/trajectory/trajectory_point.h"
#include "src/common/path/path_corridor.h"
// 使用适配器替代外部依赖
#include "cilqr_adapter_simple.h"
#include "src/common/path/path_data.h"
#include "src/common/path/path_optimizer_result.h"
#include "src/proc_ppl/optimizer/path_corridor_constructor/heuristic_trajectory_generator.h"
#include "neolix_common/util/time_logger.h"

using namespace neolix::planning;

double g_init_x = 0.0;
double g_init_y = 0.0;
double g_init_theta = 0.0;
double g_init_kappa = 0.0;
double g_init_v = 10.0;
double g_init_a = 0.0;
std::string g_ref_mode = "straight";
double g_max_v = 15.0;  // 速度限制，从 speed_conf.max_v 读取

bool LoadConfig(const std::string& config_file) {
    std::ifstream ifs(config_file);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open config file: " << config_file << std::endl;
        return false;
    }
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(ifs, root)) {
        std::cerr << "Failed to parse config file" << std::endl;
        return false;
    }
    
    if (root.isMember("init_x")) {
        g_init_x = root["init_x"].asDouble();
        std::cout << "Load init_x: " << g_init_x << std::endl;
    }
    if (root.isMember("init_y")) {
        g_init_y = root["init_y"].asDouble();
        std::cout << "Load init_y: " << g_init_y << std::endl;
    }
    if (root.isMember("init_theta")) {
        g_init_theta = root["init_theta"].asDouble();
        std::cout << "Load init_theta: " << g_init_theta << std::endl;
    }
    if (root.isMember("init_kappa")) {
        g_init_kappa = root["init_kappa"].asDouble();
        std::cout << "Load init_kappa: " << g_init_kappa << std::endl;
    }
    if (root.isMember("init_v")) {
        g_init_v = root["init_v"].asDouble();
        std::cout << "Load init_v: " << g_init_v << std::endl;
    }
    if (root.isMember("init_a")) {
        g_init_a = root["init_a"].asDouble();
        std::cout << "Load init_a: " << g_init_a << std::endl;
    }
    if (root.isMember("ref_mode")) {
        g_ref_mode = root["ref_mode"].asString();
        std::cout << "Load ref_mode: " << g_ref_mode << std::endl;
    }
    
    // 读取速度限制
    if (root.isMember("speed_conf") && root["speed_conf"].isMember("max_v")) {
        g_max_v = root["speed_conf"]["max_v"].asDouble();
        std::cout << "Load max_v: " << g_max_v << std::endl;
    }

    auto* config = config::PlanningConfigManager::Instance()->mutable_planning_config();
    
    Json::Value cilqr_conf = root;
    if (root.isMember("cilqr_trajectory_conf")) {
        cilqr_conf = root["cilqr_trajectory_conf"];
    }

    if (cilqr_conf.isMember("position_weight")) {
        config->cilqr_trajectory_conf.position_weight = cilqr_conf["position_weight"].asDouble();
        std::cout << "Load position_weight: " << config->cilqr_trajectory_conf.position_weight << std::endl;
    }
    if (cilqr_conf.isMember("speed_weight")) {
        config->cilqr_trajectory_conf.speed_weight = cilqr_conf["speed_weight"].asDouble();
        std::cout << "Load speed_weight: " << config->cilqr_trajectory_conf.speed_weight << std::endl;
    }
    if (cilqr_conf.isMember("acc_weight")) {
        config->cilqr_trajectory_conf.acc_weight = cilqr_conf["acc_weight"].asDouble();
        std::cout << "Load acc_weight: " << config->cilqr_trajectory_conf.acc_weight << std::endl;
    }
    if (cilqr_conf.isMember("jerk_weight")) {
        config->cilqr_trajectory_conf.jerk_weight = cilqr_conf["jerk_weight"].asDouble();
        std::cout << "Load jerk_weight: " << config->cilqr_trajectory_conf.jerk_weight << std::endl;
    }
    if (cilqr_conf.isMember("kappa_weight")) {
        config->cilqr_trajectory_conf.kappa_weight = cilqr_conf["kappa_weight"].asDouble();
        std::cout << "Load kappa_weight: " << config->cilqr_trajectory_conf.kappa_weight << std::endl;
    }
    if (cilqr_conf.isMember("dkappa_weight")) {
        config->cilqr_trajectory_conf.dkappa_weight = cilqr_conf["dkappa_weight"].asDouble();
        std::cout << "Load dkappa_weight: " << config->cilqr_trajectory_conf.dkappa_weight << std::endl;
    }
    if (cilqr_conf.isMember("heading_weight")) {
        config->cilqr_trajectory_conf.heading_weight = cilqr_conf["heading_weight"].asDouble();
        std::cout << "Load heading_weight: " << config->cilqr_trajectory_conf.heading_weight << std::endl;
    }
    return true;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [config_json_file]" << std::endl;
        return 0;
    }
    std::string config_file = argv[1];

    // Init
    // INIT_NEOLOG_NAME("pnc_test.log");  // Commented out - not available in mock headers
    config::PlanningConfigManager::Instance()->Init();
    
    // Override config from JSON
    if (!LoadConfig(config_file)) {
        return -1;
    }

    // 1. Create ReferenceLine (使用初始状态作为起点)
    std::vector<ReferencePoint> ref_points;
    double start_x = g_init_x;
    double start_y = g_init_y;
    double start_theta = g_init_theta;
    double start_s = 0.0;

    for (int i = 0; i < 200; ++i) {
        double s = i * 1.0;
        double x = 0.0, y = 0.0, theta = 0.0, kappa = 0.0;
        
        if (s < start_s) {
            x = start_x + s * std::cos(start_theta);
            y = start_y + s * std::sin(start_theta);
            theta = start_theta;
            kappa = 0.0;
        } else {
            double ds = s - start_s;
            double pi = M_PI;
            
            // Straight scenario
            if (g_ref_mode == "straight") {
                x = start_x + ds * std::cos(start_theta);
                y = start_y + ds * std::sin(start_theta);
                theta = start_theta;
                kappa = 0.0;
            } 
            // Left turn scenario
            else if (g_ref_mode == "left_turn") {
                double straight_len = 10.0;
                if (ds < straight_len) {
                    x = start_x + ds * std::cos(start_theta);
                    y = start_y + ds * std::sin(start_theta);
                    theta = start_theta;
                    kappa = 0.0;
                } else {
                    double ds_curve = ds - straight_len;
                    double R = 6.0;
                    double arc_len = 0.5 * pi * R;
                    double straight_end_x = start_x + straight_len * std::cos(start_theta);
                    double straight_end_y = start_y + straight_len * std::sin(start_theta);
                    double center_x = straight_end_x + R * std::cos(start_theta + pi / 2.0);
                    double center_y = straight_end_y + R * std::sin(start_theta + pi / 2.0);
                    
                    if (ds_curve <= arc_len) {
                        double angle = ds_curve / R;
                        x = center_x + R * std::cos(start_theta - pi / 2.0 + angle);
                        y = center_y + R * std::sin(start_theta - pi / 2.0 + angle);
                        theta = start_theta + angle;
                        kappa = 1.0 / R;
                    } else {
                        double ds_after = ds_curve - arc_len;
                        double turn_end_x = center_x + R * std::cos(start_theta);
                        double turn_end_y = center_y + R * std::sin(start_theta);
                        x = turn_end_x + ds_after * std::cos(start_theta + pi / 2.0);
                        y = turn_end_y + ds_after * std::sin(start_theta + pi / 2.0);
                        theta = start_theta + pi / 2.0;
                        kappa = 0.0;
                    }
                }
            } 
            // Right turn scenario
            else if (g_ref_mode == "right_turn") {
                double straight_len = 10.0;
                if (ds < straight_len) {
                    x = start_x + ds * std::cos(start_theta);
                    y = start_y + ds * std::sin(start_theta);
                    theta = start_theta;
                    kappa = 0.0;
                } else {
                    double ds_curve = ds - straight_len;
                    double R = 6.0;
                    double arc_len = 0.5 * pi * R;
                    double straight_end_x = start_x + straight_len * std::cos(start_theta);
                    double straight_end_y = start_y + straight_len * std::sin(start_theta);
                    double center_x = straight_end_x + R * std::cos(start_theta - pi / 2.0);
                    double center_y = straight_end_y + R * std::sin(start_theta - pi / 2.0);
                    
                    if (ds_curve <= arc_len) {
                        double angle = ds_curve / R;
                        x = center_x + R * std::cos(start_theta + pi / 2.0 - angle);
                        y = center_y + R * std::sin(start_theta + pi / 2.0 - angle);
                        theta = start_theta - angle;
                        kappa = -1.0 / R;
                    } else {
                        double ds_after = ds_curve - arc_len;
                        double turn_end_x = center_x + R * std::cos(start_theta);
                        double turn_end_y = center_y + R * std::sin(start_theta);
                        x = turn_end_x + ds_after * std::cos(start_theta - pi / 2.0);
                        y = turn_end_y + ds_after * std::sin(start_theta - pi / 2.0);
                        theta = start_theta - pi / 2.0;
                        kappa = 0.0;
                    }
                }
            } 
            // U-turn scenario
            else if (g_ref_mode == "u_turn") {
                double straight_len = 10.0;
                if (ds < straight_len) {
                    x = start_x + ds * std::cos(start_theta);
                    y = start_y + ds * std::sin(start_theta);
                    theta = start_theta;
                    kappa = 0.0;
                } else {
                    double ds_curve = ds - straight_len;
                    double R = 7.0;
                    double arc_len = pi * R;
                    double straight_end_x = start_x + straight_len * std::cos(start_theta);
                    double straight_end_y = start_y + straight_len * std::sin(start_theta);
                    double center_x = straight_end_x + R * std::cos(start_theta + pi / 2.0);
                    double center_y = straight_end_y + R * std::sin(start_theta + pi / 2.0);
                    
                    if (ds_curve <= arc_len) {
                        double angle = ds_curve / R;
                        x = center_x + R * std::cos(start_theta - pi / 2.0 + angle);
                        y = center_y + R * std::sin(start_theta - pi / 2.0 + angle);
                        theta = start_theta + angle;
                        kappa = 1.0 / R;
                    } else {
                        double ds_after = ds_curve - arc_len;
                        double turn_end_x = center_x + R * std::cos(start_theta + pi / 2.0);
                        double turn_end_y = center_y + R * std::sin(start_theta + pi / 2.0);
                        x = turn_end_x - ds_after * std::cos(start_theta);
                        y = turn_end_y - ds_after * std::sin(start_theta);
                        theta = start_theta + pi;
                        kappa = 0.0;
                    }
                }
            } 
            // Lane change scenario
            else if (g_ref_mode == "lane_change") {
                double switch_x = 20.0;
                double H = 3.5;
                x = start_x + ds * std::cos(start_theta);
                double lateral_offset = 0.0;
                if (ds >= switch_x) {
                    double switch_ds = ds - switch_x;
                    double switch_len = 10.0;
                    if (switch_ds < switch_len) {
                        lateral_offset = H * (switch_ds / switch_len);
                } else {
                        lateral_offset = H;
                    }
                }
                y = start_y + ds * std::sin(start_theta) + lateral_offset * std::cos(start_theta);
                theta = start_theta;
                kappa = 0.0;
            } 
            // Default straight
            else {
                x = start_x + ds * std::cos(start_theta);
                y = start_y + ds * std::sin(start_theta);
                theta = start_theta;
                kappa = 0.0;
            }
        }

        double dkappa = 0.0;
        double ddkappa = 0.0;
        ref_points.emplace_back(x, y, theta, kappa, dkappa, ddkappa, s);
    }
    ReferenceLine reference_line(ref_points);

    // 2. Create Init Point
    TrajectoryPoint init_point;
    init_point.set_x(g_init_x);
    init_point.set_y(g_init_y);
    init_point.set_theta(g_init_theta);
    init_point.set_velocity(g_init_v);
    init_point.set_acceleration(g_init_a);
    init_point.set_kappa(g_init_kappa);
    init_point.set_relative_time(0.0);

    // 3. Create PathCorridor
    PathCorridor path_corridor;
    
    auto& cilqr_conf = config::PlanningConfigManager::Instance()->planning_config().cilqr_trajectory_conf;
    int horizon = cilqr_conf.horizon;
    std::cout << "Horizon: " << horizon << std::endl;

    // 3.1 Heuristic Trajectory (Reference Speed Profile)
    // Use HeuristicTrajectoryGenerator
    auto ref_line_ptr = std::make_shared<ReferenceLine>(reference_line);
    auto ref_path_ptr = std::make_shared<ReferencePath>();
    ref_path_ptr->set_reference_line(ref_line_ptr);
    
    SLPoint init_sl;
    if (reference_line.XYToSL({init_point.x(), init_point.y()}, &init_sl)) {
        init_point.set_s(init_sl.s());
        ref_path_ptr->mutable_init_s_state()->at(0) = init_sl.s();
    } else {
        std::cerr << "Init point projection failed" << std::endl;
        ref_path_ptr->mutable_init_s_state()->at(0) = 0.0;
    }
    *ref_path_ptr->mutable_init_trajectory_point() = init_point;

    CoarseTrajectoryInfo coarse_info;
    coarse_info.reference_path_ptr = ref_path_ptr;
    
    ad::HeuristicTrajectoryGenerator heuristic_gen(coarse_info);
    if (!heuristic_gen.Init()) {
         std::cerr << "Heuristic generator init failed" << std::endl;
         return -1;
    }
    if (!heuristic_gen.Run(path_corridor.mutable_heuristic_trajectory(), 
                           path_corridor.mutable_heuristic_frenet_trajectory())) {
         std::cerr << "Heuristic generator run failed" << std::endl;
         return -1;
    }
    
    // 3.1.1 如果启发式轨迹为空，手动生成一个简单的轨迹（使用参考路径的几何信息）
    auto& heuristic_traj_points = *path_corridor.mutable_heuristic_trajectory()->mutable_trajectory_points();
    if (heuristic_traj_points.empty()) {
        double dt = cilqr_conf.dt;
        int num_points = horizon;
        
        double v = std::min(std::max(g_init_v, cilqr_conf.min_speed), g_max_v);
        
        for (int i = 0; i < num_points; ++i) {
            TrajectoryPoint pt;
            double t = i * dt;
            double s = v * t;
            
            double ref_x = g_init_x, ref_y = g_init_y, ref_theta = g_init_theta, ref_kappa = g_init_kappa;
            if (s < reference_line.ref_points().back().s()) {
                for (size_t j = 0; j < reference_line.ref_points().size(); ++j) {
                    if (reference_line.ref_points()[j].s() >= s || j == reference_line.ref_points().size() - 1) {
                        const auto& ref_pt = reference_line.ref_points()[j];
                        ref_x = ref_pt.x();
                        ref_y = ref_pt.y();
                        ref_theta = ref_pt.theta();
                        ref_kappa = ref_pt.kappa();
                        break;
                    }
                }
            } else {
                const auto& last_pt = reference_line.ref_points().back();
                ref_x = last_pt.x();
                ref_y = last_pt.y();
                ref_theta = last_pt.theta();
                ref_kappa = last_pt.kappa();
            }
            
            pt.set_x(ref_x);
            pt.set_y(ref_y);
            pt.set_theta(ref_theta);
            pt.set_kappa(ref_kappa);
            pt.set_velocity(v);
            pt.set_acceleration(0.0);
            pt.set_relative_time(t);
            pt.set_s(s);
            heuristic_traj_points.push_back(pt);
        }
    }
    
    // 3.2 Path SL Corridor (Boundaries)
    const auto& heuristic_points = path_corridor.heuristic_trajectory().trajectory_points();
    for (const auto& pt : heuristic_points) {
        SLBoundPoint sl_pt;
        sl_pt.s = pt.s();
        sl_pt.hard_lb = -3.0;
        sl_pt.hard_ub = 3.0;
        path_corridor.mutable_path_sl_corridor()->mutable_sl_bound_points()->push_back(sl_pt);
    }

    // 4. Optimize
    CilqrTrajectoryOptimizer optimizer(reference_line);
    if (!optimizer.Init()) {
        std::cerr << "Optimizer Init failed" << std::endl;
        return -1;
    }

    if (!optimizer.Optimize(init_point, path_corridor)) {
        std::cerr << "Optimizer Optimize failed" << std::endl;
        return -1;
    }

    // 5. Get Result
    PathData path_data;
    if (!optimizer.FillPathData(&path_data)) {
        std::cerr << "FillPathData failed" << std::endl;
        return -1;
    }

    CilqrPathOptimizerDebug debug;
    optimizer.ExtractDebug(&debug);
    const auto& solution_info = debug.solution_info;

    // 6. Output JSON
    Json::Value output_json;
    
    // Output Optimized Trajectory
    Json::Value trajectory_array(Json::arrayValue);
    const auto& path_points = path_data.discretized_path().path_points();
    std::cerr << "JSON output: path_points.size()=" << path_points.size() << std::endl;
    if (!path_points.empty()) {
        std::cerr << "First path_point: x=" << path_points[0].x() 
                  << ", y=" << path_points[0].y() 
                  << ", theta=" << path_points[0].theta() << std::endl;
    }
    double dt = cilqr_conf.dt;
    for (size_t i = 0; i < path_points.size(); ++i) {
        const auto& point = path_points[i];
        Json::Value p;
        p["x"] = point.x();
        p["y"] = point.y();
        p["theta"] = point.theta();
        p["kappa"] = point.kappa();
        
        double dkappa = 0.0;
        double jerk = 0.0;
        double v = 0.0;
        double a = 0.0;
        double l = 0.0;
        double t = i * dt;

        if (i < solution_info.final_x_traj.size()) {
             // final_x_traj: [x, y, v, heading, kappa, a?]
             v = solution_info.final_x_traj[i](2);  // state_v
             a = (solution_info.final_x_traj[i].size() > 5) ? solution_info.final_x_traj[i](5) : 0.0;
        }
        if (i < solution_info.final_u_traj.size()) {
             dkappa = solution_info.final_u_traj[i](0);
             jerk = solution_info.final_u_traj[i](1);
        } else if (i > 0 && !solution_info.final_u_traj.empty()) {
             dkappa = solution_info.final_u_traj.back()(0);
             jerk = solution_info.final_u_traj.back()(1);
        }

        SLPoint sl_point;
        if (reference_line.XYToSL({point.x(), point.y()}, &sl_point)) {
            l = sl_point.l();
        }

        p["dkappa"] = dkappa;
        p["jerk"] = jerk;
        p["v"] = v;
        p["a"] = a;
        p["l"] = l;
        p["t"] = t;

        p["s"] = point.s();
        trajectory_array.append(p);
    }
    output_json["trajectory"] = trajectory_array;

    // Output Reference Line
    Json::Value ref_line_array(Json::arrayValue);
    for (const auto& point : ref_points) {
        Json::Value p;
        p["x"] = point.x();
        p["y"] = point.y();
        ref_line_array.append(p);
    }
    output_json["reference_line"] = ref_line_array;

    // Output Heuristic Trajectory
    Json::Value heuristic_array(Json::arrayValue);
    for (const auto& point : path_corridor.heuristic_trajectory().trajectory_points()) {
        Json::Value p;
        p["x"] = point.x();
        p["y"] = point.y();
        p["theta"] = point.theta();
        p["s"] = point.s();
        p["kappa"] = point.kappa();
        p["dkappa"] = point.dkappa();
        p["v"] = point.velocity();
        p["t"] = point.relative_time();
        heuristic_array.append(p);
    }
    output_json["heuristic_trajectory"] = heuristic_array;

    // Output Path Corridor
    Json::Value corridor_array(Json::arrayValue);
    const auto& sl_bounds = path_corridor.path_sl_corridor().sl_bound_points();
    for (const auto& pt : sl_bounds) {
        Json::Value p;
        p["s"] = pt.s;
        p["hard_lb"] = pt.hard_lb;
        p["hard_ub"] = pt.hard_ub;
        corridor_array.append(p);
    }
    output_json["path_corridor"] = corridor_array;

    std::cout << "JSON_START" << std::endl;
    Json::FastWriter writer;
    std::cout << writer.write(output_json);
    std::cout << "JSON_END" << std::endl;

    return 0;
}