/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-09-27 01:20:39
 * @LastEditTime: 2025-02-28 00:24:55
 * @FilePath: /toy-example-of-iLQR/src/utils.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "matplotlibcpp.h"
#include "utils.hpp"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <fstream>

using std::string;
using namespace Eigen;
namespace plt = matplotlibcpp;

ReferenceLine::ReferenceLine(std::vector<double> _x, std::vector<double> _y, double width /* = 0*/,
                             double accuracy /* = 0.1*/)
    : delta_s(accuracy), delta_d(width) {
    spline = CubicSpline2D(_x, _y);
    for (double s = 0.0; s <= spline.s.back(); s += delta_s) {
        Eigen::Vector2d pos = spline.calc_position(s);
        double lyaw = spline.calc_yaw(s);
        double lx = pos.x() - width * sin(lyaw);
        double ly = pos.y() + width * cos(lyaw);
        x.emplace_back(lx);
        y.emplace_back(ly);
        yaw.emplace_back(lyaw);
        longitude.emplace_back(s);
    }
}

RoutingLine RoutingLine::subset(size_t start, size_t length) {
    size = std::min(x.size(), std::min(y.size(), yaw.size()));
    if (start >= size || (start + length) > size || length <= 0) {
        throw std::out_of_range("RoutingLine::subset args is out of range.");
    }

    RoutingLine sub_routing;
    sub_routing.x = std::vector(this->x.begin() + start, this->x.begin() + start + length);
    sub_routing.y = std::vector(this->y.begin() + start, this->y.begin() + start + length);
    sub_routing.yaw = std::vector(this->yaw.begin() + start, this->yaw.begin() + start + length);
    sub_routing.size = length;

    return sub_routing;
}

Eigen::Vector3d RoutingLine::operator[](size_t index) const {
    if (index >= x.size() || index >= y.size() || index >= yaw.size()) {
        throw std::out_of_range("Index out of range");
    }

    return Eigen::Vector3d{x[index], y[index], yaw[index]};
}

Eigen::Vector3d ReferenceLine::calc_position(double cur_s) {
    Eigen::Vector2d pos = spline.calc_position(cur_s);
    double lyaw = spline.calc_yaw(cur_s);
    double lx = pos.x() - delta_d * sin(lyaw);
    double ly = pos.y() + delta_d * cos(lyaw);

    return {lx, ly, lyaw};
}

std::default_random_engine Random::engine(std::random_device{}());

double Random::uniform(double _min, double _max) {
    std::uniform_real_distribution<double> dist(_min, _max);
    return dist(Random::engine);
}

double Random::normal(double _mean, double _std) {
    std::normal_distribution<double> dist(_mean, _std);
    double random_value = dist(Random::engine);
    while ((random_value > 3 * _std) || (random_value < -3 * _std)) {
        random_value = dist(Random::engine);
    }

    return random_value;
}

namespace utils {

std::vector<RoutingLine> get_sub_routing_lines(const std::vector<RoutingLine>& routing_lines,
                                               int start_idx) {
    size_t lines_num = routing_lines.size();
    std::vector<RoutingLine> sub_routing_lines(lines_num);

    for (size_t i = 0; i < lines_num; ++i) {
        std::copy(routing_lines[i].x.begin() + start_idx, routing_lines[i].x.end(),
                  std::back_inserter(sub_routing_lines[i].x));
        std::copy(routing_lines[i].y.begin() + start_idx, routing_lines[i].y.end(),
                  std::back_inserter(sub_routing_lines[i].y));
        std::copy(routing_lines[i].yaw.begin() + start_idx, routing_lines[i].yaw.end(),
                  std::back_inserter(sub_routing_lines[i].yaw));
    }

    return sub_routing_lines;
}

Eigen::Matrix3Xd get_cur_obstacle_states(const std::vector<RoutingLine>& routing_lines,
                                         int time_index) {
    int obstalce_num = routing_lines.size() - 1;
    Eigen::Matrix3Xd cur_obstacle_states = Eigen::Matrix3Xd::Zero(3, obstalce_num);

    for (int idx = 1; idx < routing_lines.size(); ++idx) {
        cur_obstacle_states.col(idx - 1) = routing_lines[idx][time_index];
    }

    return cur_obstacle_states;
}

bool imread(std::string filename, Outlook& outlook) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        SPDLOG_ERROR("open {} failed !", filename);
        return false;
    }

    std::string line;
    getline(file, line);
    if (line != "Convert from PNG") {
        SPDLOG_ERROR("this format is not supported: {}", filename);
        return false;
    }
    getline(file, line);
    std::istringstream iss(line);
    iss >> outlook.rows >> outlook.cols >> outlook.colors;
    outlook.data.resize(outlook.rows * outlook.cols * outlook.colors);
    int idx = 0;
    while (getline(file, line)) {
        std::istringstream iss(line);
        for (int i = 0; i < outlook.colors; ++i) {
            iss >> outlook.data[idx++];
        }
    }
    file.close();

    return true;
}

// state: [x y v yaw]
void plot_vehicle(const Outlook& out, const Eigen::Vector4d& state, const Eigen::Vector2d& para,
                  ReferencePoint ref_point /* = ReferencePoint::GravityCenter */,
                  double ws /* = 0.*/) {
    Eigen::Vector3d state_convert;
    state_convert << state[0], state[1], state[3];

    plot_vehicle(out, state_convert, para, ref_point, ws);
}

// state: [x y yaw]
void plot_vehicle(const Outlook& out, const Eigen::Vector3d& state, const Eigen::Vector2d& para,
                  ReferencePoint ref_point /* = ReferencePoint::GravityCenter */,
                  double ws /* = 0.*/) {
    std::vector<double> state_vector = {state[0], state[1], state[2]};
    std::vector<double> para_vector = {para[0], para[1]};

    if (ref_point == ReferencePoint::RearCenter) {
        state_vector[0] += 0.5 * ws * cos(state[2]);
        state_vector[1] += 0.5 * ws * sin(state[2]);
    }

    imshow(out, state_vector, para_vector);
}

void plot_obstacle_boundary(const Eigen::Vector4d& ego_state,
                            const Eigen::Matrix3Xd& obstacles_info,
                            const Eigen::Vector3d& obstacle_attribute, double wheelbase,
                            ReferencePoint reference_point /* = ReferencePoint::GravityCenter */) {
    auto [ego_front, ego_rear] =
        get_vehicle_front_and_rear_centers(ego_state, wheelbase, reference_point);
    // 复制到局部变量以便在lambda中捕获
    Eigen::Vector2d ego_front_copy = ego_front;
    Eigen::Vector2d ego_rear_copy = ego_rear;
    double radius = obstacle_attribute[0] * 0.5;
    Eigen::ArrayXd t = Eigen::ArrayXd::LinSpaced(300, 0, 2 * M_PI);
    Eigen::VectorXd sample_x = radius * t.cos();
    Eigen::VectorXd sample_y = radius * t.sin();
    Eigen::VectorXd front_circle_x =
        sample_x.unaryExpr([ego_front_copy](double x) { return x + ego_front_copy[0]; });
    Eigen::VectorXd front_circle_y =
        sample_y.unaryExpr([ego_front_copy](double x) { return x + ego_front_copy[1]; });
    Eigen::VectorXd rear_circle_x =
        sample_x.unaryExpr([ego_rear_copy](double x) { return x + ego_rear_copy[0]; });
    Eigen::VectorXd rear_circle_y =
        sample_y.unaryExpr([ego_rear_copy](double x) { return x + ego_rear_copy[1]; });
    // 转换为std::vector以兼容没有NumPy的情况
    std::vector<double> front_circle_x_vec(front_circle_x.data(), front_circle_x.data() + front_circle_x.size());
    std::vector<double> front_circle_y_vec(front_circle_y.data(), front_circle_y.data() + front_circle_y.size());
    std::vector<double> rear_circle_x_vec(rear_circle_x.data(), rear_circle_x.data() + rear_circle_x.size());
    std::vector<double> rear_circle_y_vec(rear_circle_y.data(), rear_circle_y.data() + rear_circle_y.size());
    // 移除zorder参数以避免类型错误，使用color参数即可
    plt::plot(front_circle_x_vec, front_circle_y_vec, {{"color", "red"}});
    plt::plot(rear_circle_x_vec, rear_circle_y_vec, {{"color", "red"}});

    int obstacle_num = obstacles_info.cols();
    for (size_t idx = 0; idx < obstacle_num; ++idx) {
        Eigen::Vector3d cur_state = obstacles_info.col(idx);
        Eigen::Vector2d ellipse_ab = get_ellipsoid_obstacle_scales(obstacle_attribute);
        Eigen::ArrayXd t = Eigen::ArrayXd::LinSpaced(300, 0, 2 * M_PI);
        Eigen::VectorXd sample_x = ellipse_ab[0] * t.cos();
        Eigen::VectorXd sample_y = ellipse_ab[1] * t.sin();
        Eigen::Matrix2Xd points(2, sample_x.size());
        points.row(0) = sample_x.transpose();
        points.row(1) = sample_y.transpose();
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << cos(cur_state[2]), -sin(cur_state[2]), sin(cur_state[2]),
            cos(cur_state[2]);
        Eigen::Matrix2Xd rotated_points = rotation_matrix * points;
        Eigen::VectorXd points_x =
            rotated_points.row(0).unaryExpr([cur_state](double x) { return x + cur_state[0]; });
        Eigen::VectorXd points_y =
            rotated_points.row(1).unaryExpr([cur_state](double x) { return x + cur_state[1]; });
        // 转换为std::vector以兼容没有NumPy的情况
        std::vector<double> points_x_vec(points_x.data(), points_x.data() + points_x.size());
        std::vector<double> points_y_vec(points_y.data(), points_y.data() + points_y.size());
        plt::plot(points_x_vec, points_y_vec, "-r");
    }
}

void imshow(const Outlook& out, const std::vector<double>& state, const std::vector<double>& para) {
    static PyObject* imshow_func = nullptr;
    if (imshow_func == nullptr) {
        Py_Initialize();
#ifndef WITHOUT_NUMPY
        _import_array();
#endif

        std::filesystem::path source_file_path(__FILE__);
        std::filesystem::path project_path = source_file_path.parent_path().parent_path();
        std::string script_path = project_path / "scripts" / "utils";
        PyRun_SimpleString("import sys");
        PyRun_SimpleString(fmt::format("sys.path.append('{}')", script_path).c_str());

        PyObject* py_name = PyUnicode_DecodeFSDefault("imshow");
        PyObject* py_module = PyImport_Import(py_name);
        Py_DECREF(py_name);
        if (py_module != nullptr) {
            imshow_func = PyObject_GetAttrString(py_module, "imshow");
        }
        if (imshow_func == nullptr || !PyCallable_Check(imshow_func)) {
            spdlog::error(
                "py.imshow call failed and the vehicle drawing will only "
                "support linestyle");
            imshow_func = nullptr;
        }
    }

    std::vector<double> state_list{0, 0, 0};

#ifndef WITHOUT_NUMPY
    PyObject* vehicle_state = matplotlibcpp::detail::get_array(state);
    PyObject* vehicle_para = matplotlibcpp::detail::get_array(para);
    npy_intp dims[3] = {out.rows, out.cols, out.colors};

    const float* imptr = &(out.data[0]);

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyArray_SimpleNewFromData(3, dims, NPY_FLOAT, (void*)imptr));
    PyTuple_SetItem(args, 1, vehicle_state);
    PyTuple_SetItem(args, 2, vehicle_para);

    if (imshow_func != nullptr) {
        PyObject* ret = PyObject_CallObject(imshow_func, args);
        
        Py_DECREF(args);
        if (ret) {
            Py_DECREF(ret);
        } else {
            PyErr_Print();  // 打印Python错误信息
        }
    } else {
        Py_DECREF(args);
        SPDLOG_WARN("imshow_func is null, vehicle image will not be displayed");
    }
#else
    // Fallback without NumPy - 使用matplotlibcpp的imshow函数
    // 注意：这可能需要不同的实现方式
    spdlog::warn("NumPy not available, imshow functionality may be limited");
    // 如果没有NumPy，尝试使用matplotlibcpp的通用imshow
    // 这里暂时跳过，因为需要NumPy支持
#endif
}

Eigen::Vector4d kinematic_propagate(
    const Eigen::Vector4d& cur_x, const Eigen::Vector2d& cur_u, double dt, double wheelbase,
    ReferencePoint ref_point /* = ReferencePoint::GravityCenter */) {
    double beta = atan(tan(cur_u[1] / 2));
    Eigen::Vector4d next_x;

    // clang-format off
    if (ref_point == ReferencePoint::RearCenter) {
        next_x << cur_x[0] + cur_x[2] * cos(cur_x[3]) * dt,
                  cur_x[1] + cur_x[2] * sin(cur_x[3]) * dt,
                  cur_x[2] + cur_u[0] * dt,
                  cur_x[3] + cur_x[2] * tan(cur_u[1]) * dt / wheelbase;
    } else {
        next_x << cur_x[0] + cur_x[2] * cos(beta + cur_x[3]) * dt,
                  cur_x[1] + cur_x[2] * sin(beta + cur_x[3]) * dt,
                  cur_x[2] + cur_u[0] * dt,
                  cur_x[3] + 2 * cur_x[2] * sin(beta) * dt / wheelbase;
    }
    // clang-format on

    return next_x;
}

std::tuple<Eigen::MatrixX4d, Eigen::MatrixX2d> get_kinematic_model_derivatives(
    const Eigen::MatrixX4d& x, const Eigen::MatrixX2d& u, double dt, double wheelbase,
    uint32_t steps, ReferencePoint ref_point /* = ReferencePoint::GravityCenter */) {
    Eigen::VectorXd N_velo = x.col(2).head(steps);
    Eigen::VectorXd N_yaw = x.col(3).head(steps);
    Eigen::VectorXd N_delta = u.col(1).head(steps);
    Eigen::VectorXd N_beta = (u.col(1) / 2).array().tan().atan();
    Eigen::VectorXd N_beta_over_stl =
        0.5 * (1 + u.col(1).array().tan().square()) / (1 + 0.25 * u.col(1).array().tan().square());

    //  f(state, ctrl) over state of t_0 to t_N
    //  df_dx.shape: (N, 4, 4)
    //  For t_k, df_dx[k] is organized by:
    //      [[x_k+1     -> x_k, x_k+1     -> y_k, x_k+1     -> v_k, x_k+1     -> theta_k]
    //       [y_k+1     -> x_k, y_k+1     -> y_k, y_k+1     -> v_k, y_k+1     -> theta_k]
    //       [v_k+1     -> x_k, v_k+1     -> y_k, v_k+1     -> v_k, v_k+1     -> theta_k]
    //       [theta_k+1 -> x_k, theta_k+1 -> y_k, theta_k+1 -> v_k, theta_k+1 -> theta_k]]
    Eigen::MatrixX4d df_dx(steps * 4, 4);
    //  f(state, ctrl) over ctrl of t_0 to t_N
    //  df_du.shape: (N, 4, 2)
    //  For t_k, df_du[k] is organized by:
    //      [[x_k+1     -> a_k, x_k+1     -> delta_k]
    //       [y_k+1     -> a_k, y_k+1     -> delta_k]
    //       [v_k+1     -> a_k, v_k+1     -> delta_k]
    //       [theta_k+1 -> a_k, theta_k+1 -> delta_k]]
    Eigen::MatrixX2d df_du(steps * 4, 2);

    for (uint32_t i = 0; i < steps; ++i) {
        df_dx.block<4, 4>(i * 4, 0).setIdentity();
        df_du.block<4, 2>(i * 4, 0).setZero();

        if (ref_point == ReferencePoint::RearCenter) {
            df_dx(i * 4, 2) = cos(N_yaw[i]) * dt;
            df_dx(i * 4, 3) = N_velo[i] * (-sin(N_yaw[i])) * dt;
            df_dx(i * 4 + 1, 2) = sin(N_yaw[i]) * dt;
            df_dx(i * 4 + 1, 3) = N_velo[i] * cos(N_yaw[i]) * dt;
            df_dx(i * 4 + 3, 2) = 2 * sin(N_delta[i]) * dt / wheelbase;

            df_du(i * 4 + 2, 0) = dt;
            df_du(i * 4 + 3, 1) =
                (N_velo[i] * dt / wheelbase) / (cos(N_delta[i]) * cos(N_delta[i]));
        } else {
            df_dx(i * 4, 2) = cos(N_beta[i] + N_yaw[i]) * dt;
            df_dx(i * 4, 3) = N_velo[i] * (-sin(N_beta[i] + N_yaw[i])) * dt;
            df_dx(i * 4 + 1, 2) = sin(N_beta[i] + N_yaw[i]) * dt;
            df_dx(i * 4 + 1, 3) = N_velo[i] * cos(N_beta[i] + N_yaw[i]) * dt;
            df_dx(i * 4 + 3, 2) = 2 * sin(N_beta[i]) * dt / wheelbase;

            df_du(i * 4, 1) = N_velo[i] * (-sin(N_beta[i] + N_yaw[i])) * dt * N_beta_over_stl[i];
            df_du(i * 4 + 1, 1) = N_velo[i] * cos(N_beta[i] + N_yaw[i]) * dt * N_beta_over_stl[i];
            df_du(i * 4 + 2, 0) = dt;
            df_du(i * 4 + 3, 1) =
                (2 * N_velo[i] * dt / wheelbase) * cos(N_beta[i]) * N_beta_over_stl[i];
        }
    }

    return std::make_tuple(df_dx, df_du);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> get_vehicle_front_and_rear_centers(
    const Eigen::Vector4d& state, double wheelbase,
    ReferencePoint ref_point /* = ReferencePoint::GravityCenter */) {
    double yaw = state[3];
    Eigen::Vector2d front_pnt;
    Eigen::Vector2d rear_pnt;
    Eigen::Vector2d whba_vec = wheelbase * Eigen::Vector2d{cos(yaw), sin(yaw)};

    if (ref_point == ReferencePoint::RearCenter) {
        front_pnt = state.head(2) + whba_vec;
        rear_pnt = state.head(2);
    } else {
        front_pnt = state.head(2) + 0.5 * whba_vec;
        rear_pnt = state.head(2) - 0.5 * whba_vec;
    }

    return std::make_tuple(front_pnt, rear_pnt);
}

std::tuple<Eigen::Matrix<double, 4, 2>, Eigen::Matrix<double, 4, 2>>
get_vehicle_front_and_rear_center_derivatives(double yaw, double wheelbase,
                                              ReferencePoint ref_point) {
    double half_whba = 0.5 * wheelbase;
    Eigen::Matrix<double, 4, 2> front_pnt_over_state;
    Eigen::Matrix<double, 4, 2> rear_pnt_over_state;
    // front point over (center) state:
    //      [[x_fr -> x_c, x_fr -> y_c, x_fr -> v, x_fr -> yaw]
    //       [y_fr -> x_c, y_fr -> y_c, y_fr -> v, y_fr -> yaw]]
    // rear point over (center) state:
    //      <similarly...>
    front_pnt_over_state << 1, 0, 0, 1, 0, 0, half_whba * (-sin(yaw)), half_whba * cos(yaw);
    rear_pnt_over_state << 1, 0, 0, 1, 0, 0, -half_whba * (-sin(yaw)), -half_whba * cos(yaw);

    if (ref_point == ReferencePoint::RearCenter) {
        front_pnt_over_state(3, 0) = wheelbase * (-sin(yaw));
        front_pnt_over_state(3, 1) = wheelbase * cos(yaw);
        rear_pnt_over_state(3, 0) = 0;
        rear_pnt_over_state(3, 1) = 0;
    }

    return std::make_tuple(front_pnt_over_state, rear_pnt_over_state);
}

Eigen::Vector2d get_ellipsoid_obstacle_scales(const Eigen::Vector3d& obs_attr,
                                              double ego_pnt_radius /* = 0*/) {
    double a = 0.5 * obs_attr[1] + obs_attr[2] * 6 + ego_pnt_radius;
    double b = 0.5 * obs_attr[0] + obs_attr[2] + ego_pnt_radius;

    return Eigen::Vector2d{a, b};
}

double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector3d& obs_state,
                               const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    double theta = obs_state[2];
    Eigen::Vector2d diff = pnt - elp_center;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;
    double result = 1 - (pow(pnt_std[0], 2) / pow(ellipse_ab[0], 2) +
                         pow(pnt_std[1], 2) / pow(ellipse_ab[1], 2));

    return result;
}

Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                    const Eigen::Vector3d& obs_state,
                                                    const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    Eigen::Vector2d diff = pnt - elp_center;
    double theta = obs_state[2];
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;

    // (1) constraint over standard point vec.:
    //      [c -> x_std, c -> y_std]
    Eigen::Vector2d res_over_pnt_std = {-2 * pnt_std[0] / pow(ellipse_ab[0], 2),
                                        -2 * pnt_std[1] / pow(ellipse_ab[1], 2)};

    // (2) standard point vec. over difference vec.:
    //      [[x_std -> x_diff, x_std -> y_diff]
    //       [y_std -> x_diff, y_std -> y_diff]]
    Eigen::Matrix2d pnt_std_over_diff = rotation_matrix.transpose();

    // (3) difference vec. over original point vec.:
    //      [[x_diff -> x, x_diff -> y]
    //       [y_diff -> x, y_diff -> y]]
    Eigen::Matrix2d diff_over_pnt = Eigen::Matrix2d::Identity();

    // chain (1)(2)(3) together:
    //      [c -> x, c -> y]
    Eigen::Vector2d res_over_pnt = diff_over_pnt * pnt_std_over_diff * res_over_pnt_std;

    return res_over_pnt;
}

Eigen::MatrixX4d get_boundary(const Eigen::MatrixX4d& refline, double width) {
    double half_width = width / 2;
    int n_points = refline.rows();
    Eigen::MatrixX4d boundary = Eigen::MatrixX4d::Zero(n_points - 1, 4);

    for (int i = 1; i < n_points; ++i) {
        double cur_x = refline(i, 0);
        double cur_y = refline(i, 1);
        double cur_yaw = refline(i, 3);
        boundary(i - 1, 0) = cur_x - half_width * sin(cur_yaw);
        boundary(i - 1, 1) = cur_y + half_width * cos(cur_yaw);
        boundary(i - 1, 2) = cur_x + half_width * sin(cur_yaw);
        boundary(i - 1, 3) = cur_y - half_width * cos(cur_yaw);
    }

    return boundary;
}

// 路径模型传播函数（5维状态空间）
Vector5d utils::path_model_propagate(const Vector5d& cur_x,
                                     const Eigen::Vector2d& cur_u, double dt) {
    // 状态: [x, y, v, heading, kappa]
    // 控制: [acceleration, dkappa]
    double v = cur_x[2];
    double heading = cur_x[3];
    double kappa = cur_x[4];
    double a = cur_u[0];
    double dkappa = cur_u[1];

    Vector5d next_x;
    next_x << cur_x[0] + v * cos(heading) * dt,      // x
              cur_x[1] + v * sin(heading) * dt,      // y
              cur_x[2] + a * dt,                      // v
              cur_x[3] + v * kappa * dt,              // heading
              cur_x[4] + dkappa * dt;                 // kappa

    return next_x;
}

// 路径模型线性化（计算A和B矩阵）
std::tuple<MatrixX5d, Eigen::MatrixX2d> utils::get_path_model_derivatives(
    const MatrixX5d& x, const Eigen::MatrixX2d& u, double dt, uint32_t steps) {
    // A矩阵：∂f/∂x (5x5)
    // B矩阵：∂f/∂u (5x2)
    MatrixX5d df_dx(steps * 5, 5);
    Eigen::MatrixX2d df_du(steps * 5, 2);

    for (uint32_t i = 0; i < steps; ++i) {
        double v = x(i, 2);
        double heading = x(i, 3);
        double kappa = x(i, 4);

        // 初始化为单位矩阵
        df_dx.block<5, 5>(i * 5, 0).setIdentity();
        df_du.block<5, 2>(i * 5, 0).setZero();

        // A矩阵：∂f/∂x
        // ∂x/∂v = cos(heading) * dt
        df_dx(i * 5, 2) = cos(heading) * dt;
        // ∂x/∂heading = -v * sin(heading) * dt
        df_dx(i * 5, 3) = -v * sin(heading) * dt;

        // ∂y/∂v = sin(heading) * dt
        df_dx(i * 5 + 1, 2) = sin(heading) * dt;
        // ∂y/∂heading = v * cos(heading) * dt
        df_dx(i * 5 + 1, 3) = v * cos(heading) * dt;

        // ∂heading/∂v = kappa * dt
        df_dx(i * 5 + 3, 2) = kappa * dt;
        // ∂heading/∂kappa = v * dt
        df_dx(i * 5 + 3, 4) = v * dt;

        // B矩阵：∂f/∂u
        // ∂v/∂a = dt
        df_du(i * 5 + 2, 0) = dt;
        // ∂kappa/∂dkappa = dt
        df_du(i * 5 + 4, 1) = dt;
    }

    return std::make_tuple(df_dx, df_du);
}

// 从参考线计算曲率
double utils::calc_kappa_from_reference(const ReferenceLine& ref_line, double s) {
    // 使用三次样条插值计算曲率
    // kappa = d^2y/dx^2 / (1 + (dy/dx)^2)^(3/2)
    // 对于参数化曲线 (x(s), y(s))，曲率公式为：
    // kappa = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
    
    if (s < 0) s = 0;
    if (s > ref_line.length()) s = ref_line.length();
    
    // 使用数值微分计算曲率
    double ds = 0.01;  // 小的步长
    double s1 = std::max(0.0, s - ds);
    double s2 = std::min(ref_line.length(), s + ds);
    
    Eigen::Vector2d pos1 = ref_line.spline.calc_position(s1);
    Eigen::Vector2d pos = ref_line.spline.calc_position(s);
    Eigen::Vector2d pos2 = ref_line.spline.calc_position(s2);
    
    double dx_ds = (pos2[0] - pos1[0]) / (2 * ds);
    double dy_ds = (pos2[1] - pos1[1]) / (2 * ds);
    double d2x_ds2 = (pos2[0] - 2 * pos[0] + pos1[0]) / (ds * ds);
    double d2y_ds2 = (pos2[1] - 2 * pos[1] + pos1[1]) / (ds * ds);
    
    double denominator = pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
    if (denominator < 1e-6) {
        return 0.0;  // 避免除零
    }
    
    double kappa = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator;
    return kappa;
}

std::vector<std::vector<double>> get_closed_curve(const Eigen::MatrixX4d& refline) {
    int n_points = refline.rows();
    std::vector<std::vector<double>> closed_curve(2, std::vector<double>(2 * n_points, 0));

    for (int i = n_points - 1; i >= 0; --i) {
        closed_curve[0][n_points - 1 - i] = refline(i, 0);
        closed_curve[1][n_points - 1 - i] = refline(i, 1);
    }
    for (int i = 0; i < n_points; ++i) {
        closed_curve[0][n_points + i] = refline(i, 2);
        closed_curve[1][n_points + i] = refline(i, 3);
    }

    return closed_curve;
}

}  // namespace utils
