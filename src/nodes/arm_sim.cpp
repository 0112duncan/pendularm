#include "json.hpp"
#include "node.hpp"

#include <cstdint>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <vector>
#include <tuple>

int n_lengths = []() {
    const char* env = std::getenv("ARM_SIM_LINKS");
    return env != nullptr && std::string(env) == "3" ? 3 : 2;
}();

constexpr double PI = 3.14159265358979323846;
std::string method = "rk4";
double sim_time = 0;
double dt = 0.005;
double gravity = 9.81;
int n_masses = n_lengths;
std::vector<double> theta(n_lengths, 0.0);     //right is 0, straight up pi/2, down -pi/2
std::vector<double> theta_dot(n_lengths, 0.0);
std::vector<double> desired_position(n_lengths, 0.0);
std::vector<double> desired_velocity(n_lengths, 0.0);
std::vector<double> masses(n_lengths, 1.0);
std::vector<double> lengths(n_lengths, 1.0);
bool pause = false;
std::vector<double> pid_p(n_lengths, 150.0);
std::vector<double> pid_i(n_lengths, 1.0);
std::vector<double> pid_d(n_lengths, 5.0);
std::vector<double> accumulate_error(n_lengths, 0.0);
std::vector<double> pid_position_error(n_lengths, 0.0);
std::vector<double> pid_velocity_error(n_lengths, 0.0);
std::vector<double> pid_p_term(n_lengths, 0.0);
std::vector<double> pid_i_term(n_lengths, 0.0);
std::vector<double> pid_d_term(n_lengths, 0.0);
constexpr double PID_INTEGRAL_LIMIT = 10.0;
std::vector<double> t(n_lengths, 0.0);
std::vector<double> a0(n_lengths, 0.0);
bool is_pid_on = false;
std::mutex arm_mutex;

void initial_pid(std::vector<double>& pid_p, std::vector<double>& pid_i, std::vector<double>& pid_d) {
    if (n_lengths == 2){
        pid_p[0] = 100;
        pid_i[0] = 150;
        pid_d[0] = 15;
        pid_p[1] = 50;
        pid_i[1] = 100;
        pid_d[1] = 10;
    }
    if (n_lengths == 3){
        pid_p[0] = 150;
        pid_i[0] = 40;
        pid_d[0] = 20;
        pid_p[1] = 75;
        pid_i[1] = 25;
        pid_d[1] = 10;
        pid_p[2] = 35;
        pid_i[2] = 12;
        pid_d[2] = 5;
    }
}
// /arm_sim/set_integrator
//service:  {set_method": "euler"|"midpoint"|"verlet"|"rk4", "timestep": <positive seconds>}
std::tuple<bool, autorob::Json, std::string> set_integrator(const autorob::Json& args) {
    bool valid = true;
    std::string error_msg;

    const autorob::Json* method_arg = args.get("method");
    if (method_arg != nullptr) {
        if (!method_arg->is_string()) {
            valid = false;
            error_msg = "method must be a string";
        } else {
            const std::string requested_method = method_arg->as_string();
            if (requested_method != "euler" && requested_method != "midpoint" &&
                requested_method != "verlet" && requested_method != "rk4") {
                valid = false;
                error_msg = "method must be euler, midpoint, verlet, or rk4";
            } else {
                method = requested_method;
            }
        }
    }

    const autorob::Json* timestep_arg = args.get("timestep");
    if (timestep_arg != nullptr) {
        if (!timestep_arg->is_number()) {
            valid = false;
            if (!error_msg.empty()) error_msg += "; ";
            error_msg += "timestep must be numeric";
        } else {
            const double requested_timestep = timestep_arg->as_number();
            if (!std::isfinite(requested_timestep) || requested_timestep <= 0.0) {
                valid = false;
                if (!error_msg.empty()) error_msg += "; ";
                error_msg += "timestep must be finite and positive";
            } else {
                dt = requested_timestep;
            }
        }
    }

    return {valid, autorob::Json::Object{{"method", method}, {"timestep", dt}}, error_msg};
}

// /arm_sim/set_params
// {"gravity": <>=0>, "masses": [n values, each >0], "lengths": [n values, each >0]}
// (n is however many links the runtime was started with; gravity defaults to 9.81 m/s²)
std::tuple<bool, autorob::Json, std::string> set_params(const autorob::Json& args) {
    bool gravity_valid = true;
    bool masses_valid = true;
    bool lengths_valid = true;
    std::string error_msg = "";
    
    if (args.get("gravity") != nullptr) {
        if (args.get("gravity")->is_number() &&
            std::isfinite(args.get("gravity")->as_number()) &&
            args.get("gravity")->as_number() >= 0) {
            gravity = args.get("gravity")->as_number();
        }
        else{
            gravity_valid = false;
            error_msg = "gravity element <0 or not number";
        }
    }

    if (args.get("masses") != nullptr) {
        if (!args.get("masses")->is_array() ||
            args.get("masses")->as_array().size() != static_cast<std::size_t>(n_masses)) {
            masses_valid = false;
            error_msg = "masses wrong type or length";
        }
        else {
            std::vector<autorob::Json> mass_arr = args.get("masses")->as_array();
            std::vector<double> temp_mass_arr;
            for (std::size_t i = 0; i < mass_arr.size(); ++i) {
                if (mass_arr[i].is_number() && std::isfinite(mass_arr[i].as_number()) &&
                    mass_arr[i].as_number() > 0) {
                    temp_mass_arr.push_back(mass_arr[i].as_number());
                }
                else{
                    masses_valid = false;
                    error_msg = "mass arr contain wrong element";
                    break;
                }
            }
            if (masses_valid) {
                masses = temp_mass_arr;
            }
        }
    }

    if (args.get("lengths") != nullptr) {
        if (!args.get("lengths")->is_array() ||
            args.get("lengths")->as_array().size() != static_cast<std::size_t>(n_lengths)) {
            lengths_valid = false;
            error_msg = "lengths wrong type or length";
        }
        else {
            std::vector<autorob::Json> lengths_arr = args.get("lengths")->as_array();
            std::vector<double> temp_lengths_arr;
            for (std::size_t i = 0; i < lengths_arr.size(); ++i) {
                if (lengths_arr[i].is_number() &&
                    std::isfinite(lengths_arr[i].as_number()) &&
                    lengths_arr[i].as_number() > 0) {
                    temp_lengths_arr.push_back(lengths_arr[i].as_number());
                }
                else{
                    lengths_valid = false;
                    error_msg = "lengths array contains an invalid element";
                    break;
                }
            }
            if (lengths_valid) {
                lengths = temp_lengths_arr;
            }
        }
    }

    autorob::Json::Array masses_json;
    autorob::Json::Array lengths_json;

    for (std::size_t i = 0; i < masses.size(); ++i) {
        masses_json.push_back(masses[i]);
        lengths_json.push_back(lengths[i]);
    }

    autorob::Json result = autorob::Json(autorob::Json::Object{
        {"gravity", gravity},
        {"masses", masses_json}, 
        {"lengths", lengths_json}
    });

    return {(gravity_valid && masses_valid && lengths_valid), result, error_msg};
}

// /arm_sim/pause
// {"data": <bool>}
// /joint_states keeps publishing a frozen snapshot and every other service keeps responding normally, 
// but the physics step and the PID integral both stop advancing.
std::tuple<bool, autorob::Json, std::string>  pause_service(const autorob::Json& args){
    const autorob::Json* requested = args.get("data");
    if (requested != nullptr && !requested->is_bool()) {
        return {false, autorob::Json::Object{{"data", pause}},
                "pause data must be boolean"};
    }
    if (requested != nullptr) pause = requested->as_bool();
    std::string msg;
    if (pause) {
        msg = "physic paused";
    }
    else{
        msg = "physic resumed";
    }
    autorob::Json result = autorob::Json::Object{
        {"data", pause}
    };
    return {true, result, msg};
}

// /arm_sim/reset
// {"position": [...], "velocity": [...]}
// Snaps (q, qdot) back to a fixed starting pose and zeroes the simulation clock,
// and resets the PID controller's setpoint and integral to matc
std::tuple<bool, autorob::Json, std::string> reset_service() {
    sim_time = 0;
    theta.assign(n_lengths, 0.0);
    theta_dot.assign(n_lengths, 0.0);
    desired_position.assign(n_lengths, 0.0);
    desired_velocity.assign(n_lengths, 0.0);
    accumulate_error.assign(n_lengths, 0.0);
    pid_position_error.assign(n_lengths, 0.0);
    pid_velocity_error.assign(n_lengths, 0.0);
    pid_p_term.assign(n_lengths, 0.0);
    pid_i_term.assign(n_lengths, 0.0);
    pid_d_term.assign(n_lengths, 0.0);
    t.assign(n_lengths, 0.0);
    a0.assign(n_lengths, 0.0);
    std::string msg = "reset";

    autorob::Json::Array position_json;
    autorob::Json::Array velocity_json;

    for (std::size_t i = 0; i < theta.size(); ++i) {
        position_json.push_back(theta[i]);
        velocity_json.push_back(theta_dot[i]);
    }

    autorob::Json result = autorob::Json::Object{
        {"position", position_json},
        {"velocity", velocity_json}
    };

    return {true, result, msg};
}
// /pid_controller/enable
// {"data": <bool>}
std::tuple<bool, autorob::Json, std::string> on_off_pid(const autorob::Json& args) {
    const autorob::Json* requested = args.get("data");
    if (!requested) {
        return {true, autorob::Json::Object{{"data", is_pid_on}}, ""};
    }
    if (!requested->is_bool()) {
        return {false, autorob::Json::Object{{"data", is_pid_on}},
                "pid enable needs a boolean data field"};
    }
    const bool next_state = requested->as_bool();
    if (next_state && !is_pid_on) {
        accumulate_error.assign(n_lengths, 0.0);
    }
    is_pid_on = next_state;
    if (!is_pid_on) {
        std::fill(t.begin(), t.end(), 0.0);
        std::fill(pid_p_term.begin(), pid_p_term.end(), 0.0);
        std::fill(pid_i_term.begin(), pid_i_term.end(), 0.0);
        std::fill(pid_d_term.begin(), pid_d_term.end(), 0.0);
    }
    return {true, autorob::Json::Object{{"data", is_pid_on}},
            is_pid_on ? "pid is on" : "pid is off"};
}

// /pid_controller/set_gains
// {"kp": [...], "ki": [...], "kd": [...]}
std::tuple<bool, autorob::Json, std::string> set_pid_gains(const autorob::Json& args) {
    bool valid = true;
    std::string error_msg;

    auto apply_gain = [&](const char* key, std::vector<double>& current) {
        const autorob::Json* value = args.get(key);
        if (value == nullptr) return true;
        if (!value->is_array() || value->as_array().size() != current.size()) {
            if (!error_msg.empty()) error_msg += "; ";
            error_msg += std::string(key) + " must contain one value per joint";
            return false;
        }

        std::vector<double> parsed;
        parsed.reserve(current.size());
        for (const autorob::Json& item : value->as_array()) {
            if (!item.is_number() || !std::isfinite(item.as_number()) || item.as_number() < 0.0) {
                if (!error_msg.empty()) error_msg += "; ";
                error_msg += std::string(key) + " values must be finite and non-negative";
                return false;
            }
            parsed.push_back(item.as_number());
        }
        current = std::move(parsed);
        return true;
    };

    if (!apply_gain("kp", pid_p)) valid = false;
    if (!apply_gain("ki", pid_i)) valid = false;
    if (!apply_gain("kd", pid_d)) valid = false;

    autorob::Json::Array pid_p_json, pid_i_json, pid_d_json;
    for (int i = 0; i < n_lengths; ++i) {
        pid_p_json.push_back(pid_p[i]);
        pid_i_json.push_back(pid_i[i]);
        pid_d_json.push_back(pid_d[i]);
    }
    autorob::Json result = autorob::Json::Object{
        {"kp", pid_p_json}, {"ki", pid_i_json}, {"kd", pid_d_json}
    };
    return {valid, result, error_msg};
}

void update_acceleration_2link(const std::vector<double>& masses, const std::vector<double>& lengths, const std::vector<double>& theta, const std::vector<double>& theta_dot, std::vector<double>& a0) {
    double m1 = masses[0], m2 = masses[1];
    double l1 = lengths[0], l2 = lengths[1];
    double q1 = theta[0], q2 = theta[1];
    double dq1 = theta_dot[0], dq2 = theta_dot[1];

    double phi1 = q1;
    double phi2 = q1 + q2;

    double c2 = std::cos(q2);
    double s2 = std::sin(q2);

    double M11 = m1*l1*l1/3.0 + m2*(l1*l1 + l1*l2*c2 + l2*l2/3.0);
    double M12 = m2*(l1*l2*c2/2.0 + l2*l2/3.0);
    double M21 = M12;
    double M22 = m2*l2*l2/3.0;

    double h1 = -m2*l1*l2*s2*(dq1*dq2 + dq2*dq2/2.0);
    double h2 = m2*l1*l2*s2*dq1*dq1/2.0;

    double G1 = gravity*(l1*std::cos(phi1)*(m1/2.0 + m2) + l2*std::cos(phi2)*(m2/2.0));
    double G2 = gravity*l2*std::cos(phi2)*(m2/2.0);

    double rhs1 = t[0] - h1 - G1;
    double rhs2 = t[1] - h2 - G2;

    double det = M11*M22 - M12*M21;

    a0.resize(2);
    a0[0] = (M22*rhs1 - M12*rhs2)/det;
    a0[1] = (-M21*rhs1 + M11*rhs2)/det;
}

void update_acceleration_3link(const std::vector<double>& masses, const std::vector<double>& lengths, const std::vector<double>& theta, const std::vector<double>& theta_dot, std::vector<double>& a0){
    double m1 = masses[0];
    double m2 = masses[1];
    double m3 = masses[2];
    double l1 = lengths[0];
    double l2 = lengths[1];
    double l3 = lengths[2];
    double q1 = theta[0];
    double q2 = theta[1];
    double q3 = theta[2];
    double dq1 = theta_dot[0];
    double dq2 = theta_dot[1];
    double dq3 = theta_dot[2];
    double phi1 = q1;
    double phi2 = q1 + q2;
    double phi3 = q1 + q2 + q3;
    double c2  = std::cos(q2);
    double c3  = std::cos(q3);
    double c23 = std::cos(q2 + q3);

    // ---------------- M(q) ----------------
    double M11 = m1*l1*l1/3.0 + m2*(l1*l1 + l1*l2*c2 + l2*l2/3.0) + m3*(l1*l1 + l2*l2 + l3*l3/3.0 + 2*l1*l2*c2 + l1*l3*c23 + l2*l3*c3);
    double M12 = m2*(l1*l2*c2/2.0 + l2*l2/3.0) + m3*(l1*l2*c2 + l1*l3*c23/2.0 + l2*l2 + l2*l3*c3 + l3*l3/3.0);
    double M13 = m3*(l1*l3*c23/2.0 + l2*l3*c3/2.0 + l3*l3/3.0);
    double M22 = m2*l2*l2/3.0 + m3*(l2*l2 + l2*l3*c3 + l3*l3/3.0);
    double M23 = m3*(l2*l3*c3/2.0 + l3*l3/3.0);
    double M33 = m3*l3*l3/3.0;

    // symmetric matrix
    double M[3][3] = {
        {M11, M12, M13},
        {M12, M22, M23},
        {M13, M23, M33}
    };

    // -------- C(q,qdot) * qdot --------
    double A = l1*l2*(m2 + 2*m3)*std::sin(q2);
    double B = l1*l3*m3*std::sin(q2 + q3);
    double C = l2*l3*m3*std::sin(q3);

    double w1 = dq1;
    double w2 = dq1 + dq2;
    double w3 = dq1 + dq2 + dq3;

    double h1 = -0.5*A*(w2*w2 - w1*w1) - 0.5*B*(w3*w3 - w1*w1) - 0.5*C*(w3*w3 - w2*w2);
    double h2 = 0.5*(A + B)*w1*w1 - 0.5*C*(w3*w3 - w2*w2);
    double h3 = 0.5*B*w1*w1 +0.5*C*w2*w2;

    // ---------------- G(q) ----------------
    double G1 = gravity * (l1*std::cos(phi1)*(m1/2.0 + m2 + m3) + l2*std::cos(phi2)*(m2/2.0 + m3) + l3*std::cos(phi3)*(m3/2.0));
    double G2 = gravity * (l2*std::cos(phi2)*(m2/2.0 + m3) + l3*std::cos(phi3)*(m3/2.0));
    double G3 = gravity*l3*std::cos(phi3)*(m3/2.0);

    // M*qddot = torque - C*qdot - G
    double rhs[3] = {t[0]-h1-G1, t[1]-h2-G2, t[2]-h3-G3};

    // augmented matrix [M | rhs]
    double A_mat[3][4] = {
        {M[0][0], M[0][1], M[0][2], rhs[0]},
        {M[1][0], M[1][1], M[1][2], rhs[1]},
        {M[2][0], M[2][1], M[2][2], rhs[2]}
    };

    // Gaussian elimination
    for (int i = 0; i < 3; i++) {
        int pivot = i;
        for (int r = i + 1; r < 3; r++) {
            if (std::abs(A_mat[r][i]) > std::abs(A_mat[pivot][i])) {
                pivot = r;
            }
        }
        for (int c = 0; c < 4; c++) {
            std::swap(A_mat[i][c], A_mat[pivot][c]);
        }
        for (int r = i + 1; r < 3; r++) {
            double factor = A_mat[r][i] / A_mat[i][i];
            for (int c = i; c < 4; c++) {
                A_mat[r][c] -= factor * A_mat[i][c];
            }
        }
    }

    a0.resize(3);
    // back substitution
    for (int i = 2; i >= 0; i--) {
        double value = A_mat[i][3];
        for (int j = i + 1; j < 3; j++) {
            value -= A_mat[i][j] * a0[j];
        }
        a0[i] = value / A_mat[i][i];
    }
}

void update_acceleration(const std::vector<double>& masses, const std::vector<double>& lengths, const std::vector<double>& theta, const std::vector<double>& theta_dot, std::vector<double>& a0) {
    if (n_lengths == 2) {
        update_acceleration_2link(
            masses, lengths, theta, theta_dot, a0
        );
    }
    else if (n_lengths == 3) {
        update_acceleration_3link(
            masses, lengths, theta, theta_dot, a0
        );
    }
}

autorob::Json make_timestamp(double sim_time) {
    std::uint64_t sec = static_cast<std::uint64_t>(sim_time);
    std::uint64_t nanosec = static_cast<std::uint64_t>((sim_time - static_cast<double>(sec)) * 1e9);
    // protect against floating-point rounding
    if (nanosec >= 1000000000) {
        sec++;
        nanosec -= 1000000000;
    }
    return autorob::Json::Object{
        {"sec", static_cast<double>(sec)},
        {"nanosec", static_cast<double>(nanosec)}
    };
}

double total_energy() {
    std::vector<double> orientation(n_lengths, 0.0);
    std::vector<double> angular_velocity(n_lengths, 0.0);
    for (int i = 0; i < n_lengths; ++i) {
        orientation[i] = theta[i] + (i ? orientation[i - 1] : 0.0);
        angular_velocity[i] = theta_dot[i] + (i ? angular_velocity[i - 1] : 0.0);
    }

    double kinetic = 0.0;
    double potential = 0.0;
    for (int i = 0; i < n_lengths; ++i) {
        double y = 0.0, vx = 0.0, vy = 0.0;
        double minimum_y = 0.0;
        for (int j = 0; j <= i; ++j) {
            const double radius = j == i ? lengths[i] / 2.0 : lengths[j];
            minimum_y -= radius;
            y += radius * std::sin(orientation[j]);
            vx -= radius * std::sin(orientation[j]) * angular_velocity[j];
            vy += radius * std::cos(orientation[j]) * angular_velocity[j];
        }
        const double inertia = masses[i] * lengths[i] * lengths[i] / 12.0;
        kinetic += 0.5 * masses[i] * (vx * vx + vy * vy) +
                   0.5 * inertia * angular_velocity[i] * angular_velocity[i];
        // U = m*g*h, with h measured above this COMs lowest reachable height.
        potential += masses[i] * gravity * (y - minimum_y);
    }
    return kinetic + potential;
}

int main(){
    autorob::Node node("arm_sim");
    node.subscribe("/joint_trajectory", "trajectory_msgs/JointTrajectory", [](const autorob::Json& message) {
        const autorob::Json* points_arg = message.get("points");
        if (!points_arg || !points_arg->is_array()) return;
        const auto& points = points_arg->as_array();
        if (points.empty()) {
            return;
        }
        const auto& last_point = points.back();
        const autorob::Json* positions_arg = last_point.get("positions");
        if (!positions_arg || !positions_arg->is_array()) return;
        const auto& positions = positions_arg->as_array();
        if (positions.size() != static_cast<std::size_t>(n_lengths)) return;

        std::vector<double> requested_positions;
        requested_positions.reserve(positions.size());
        for (const autorob::Json& position : positions) {
            if (!position.is_number() || !std::isfinite(position.as_number())) return;
            requested_positions.push_back(position.as_number());
        }

        std::vector<double> requested_velocities(n_lengths, 0.0);
        const autorob::Json* velocities_arg = last_point.get("velocities");
        if (velocities_arg && velocities_arg->is_array() && velocities_arg->as_array().size() == static_cast<std::size_t>(n_lengths)) {
            bool velocities_valid = true;
            for (int i = 0; i < n_lengths; ++i) {
                const autorob::Json& velocity = velocities_arg->as_array()[i];
                if (!velocity.is_number() || !std::isfinite(velocity.as_number())) {
                    velocities_valid = false;
                    break;
                }
                requested_velocities[i] = velocity.as_number();
            }
            if (!velocities_valid) {
                std::fill(requested_velocities.begin(), requested_velocities.end(), 0.0);
            }
        }

        std::lock_guard<std::mutex> lock(arm_mutex);
        desired_position = std::move(requested_positions);
        desired_velocity = std::move(requested_velocities);
    });

    node.advertise_service("/arm_sim/set_integrator", "arm_sim/set_integrator", [](const autorob::Json& args) {
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = set_integrator(args);
            return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise_service("/arm_sim/set_params", "arm_sim/set_params", [](const autorob::Json& args) {
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = set_params(args);
            return autorob::ServiceResult{std::get<0>(result),std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise_service("/arm_sim/pause", "arm_sim/pause", [](const autorob::Json& args) {
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = pause_service(args);
            return autorob::ServiceResult{std::get<0>(result),std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise_service("/arm_sim/reset", "arm_sim/reset", [](const autorob::Json& args) {
            (void)args;
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = reset_service();
            return autorob::ServiceResult{std::get<0>(result),std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise_service("/pid_controller/enable", "pid_controller/enable", [](const autorob::Json& args) {
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = on_off_pid(args);
            return autorob::ServiceResult{std::get<0>(result),std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise_service("/pid_controller/set_gains", "pid_controller/set_gains", [](const autorob::Json& args) {
            std::lock_guard<std::mutex> lock(arm_mutex);
            auto result = set_pid_gains(args);
            return autorob::ServiceResult{std::get<0>(result),std::get<1>(result), std::get<2>(result)};
        }
    );

    node.advertise("/joint_states", "sensor_msgs/JointState");
    node.advertise("/arm_sim/energy", "std_msgs/Float64");

    node.advertise("/pid_controller/state", "pid_controller/State");
    initial_pid(pid_p, pid_i, pid_d);
    while (true) {
        double sleep_duration = 0.02;        
        {
        std::lock_guard<std::mutex> lock(arm_mutex);
        if (!pause) {
            const std::vector<double> theta_before_step = theta;
            // Required PID law: position P/I plus commanded-minus-measured
            // velocity for the derivative term.
            for (int i = 0; i < n_lengths; ++i) {
                pid_position_error[i] = desired_position[i] - theta[i];
                pid_velocity_error[i] = desired_velocity[i] - theta_dot[i];
                if (is_pid_on) {
                    accumulate_error[i] = std::clamp(accumulate_error[i] + pid_position_error[i] * dt, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
                    pid_p_term[i] = pid_p[i] * pid_position_error[i];
                    pid_i_term[i] = pid_i[i] * accumulate_error[i];
                    pid_d_term[i] = pid_d[i] * pid_velocity_error[i];
                    t[i] = pid_p_term[i] + pid_i_term[i] + pid_d_term[i];
                } else {
                    pid_p_term[i] = 0.0;
                    pid_i_term[i] = 0.0;
                    pid_d_term[i] = 0.0;
                    t[i] = 0.0;
                }
            }
            //calculate
            update_acceleration(masses, lengths, theta, theta_dot, a0);

            if (method == "euler") {
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta[i] = theta[i] + theta_dot[i] * dt;
                    theta_dot[i] = theta_dot[i] +  a0[i] * dt;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
            }
            else if (method == "midpoint") {
                std::vector<double> theta_dot_t0 = theta_dot;
                std::vector<double> theta_t0 = theta;

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta_dot[i] = theta_dot_t0[i] + a0[i]*dt/2;
                    theta[i] = theta_t0[i] + theta_dot_t0[i]*dt/2;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta[i] = theta_t0[i] + theta_dot[i]*dt;
                    theta_dot[i] = theta_dot_t0[i] + a0[i]*dt;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
            }
            else if (method == "verlet") {
                std::vector<double> theta_dot_t0 = theta_dot;
                std::vector<double> a_t0 = a0;

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta[i] = theta[i] + theta_dot_t0[i]*dt + a_t0[i]*dt*dt/2;
                    theta_dot[i] = theta_dot_t0[i] + a_t0[i]*dt;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta_dot[i] = theta_dot_t0[i] + (a_t0[i] + a0[i])*dt/2;
                }
            }
            else if (method == "rk4") {
                std::vector<double> theta_t0 = theta;
                std::vector<double> theta_dot_t0 = theta_dot;
                std::vector<double> k1_v;
                std::vector<double> k1_a;
                std::vector<double> k2_v;
                std::vector<double> k2_a;
                std::vector<double> k3_v;
                std::vector<double> k3_a;
                std::vector<double> k4_v;
                std::vector<double> k4_a;

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k1_v.push_back(theta_dot[i]);
                    k1_a.push_back(a0[i]);
                }
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k2_v.push_back(theta_dot_t0[i] + k1_a[i]*dt/2);
                    theta_dot[i] = theta_dot_t0[i] + k1_a[i]*dt/2;
                    theta[i] = theta_t0[i] + k1_v[i]*dt/2;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k2_a.push_back(a0[i]);
                }

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k3_v.push_back(theta_dot_t0[i] + k2_a[i]*dt/2);
                    theta_dot[i] = theta_dot_t0[i] + k2_a[i]*dt/2;
                    theta[i] = theta_t0[i] + k2_v[i]*dt/2;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k3_a.push_back(a0[i]);
                }

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k4_v.push_back(theta_dot_t0[i] + k3_a[i]*dt);
                    theta_dot[i] = theta_dot_t0[i] + k3_a[i]*dt;
                    theta[i] = theta_t0[i] + k3_v[i]*dt;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    k4_a.push_back(a0[i]);
                }

                for (std::size_t i = 0; i < theta.size(); ++i) {
                    theta_dot[i] = theta_dot_t0[i] + (k1_a[i] + 2*k2_a[i] + 2*k3_a[i] + k4_a[i])/6*dt;
                    theta[i] = theta_t0[i] + (k1_v[i] + 2*k2_v[i] + 2*k3_v[i] + k4_v[i])/6*dt;
                }
                update_acceleration(masses, lengths, theta, theta_dot, a0);
            }

            bool finite_state = std::isfinite(total_energy());
            for (int i = 0; i < n_lengths; ++i) {
                finite_state = finite_state && std::isfinite(theta[i]) &&
                               std::isfinite(theta_dot[i]) && std::isfinite(t[i]);
            }
            if (finite_state) {
                sim_time += dt;
            } else {
                // Resume control on the next tick if PID was running before
                // this rejected step; an explicitly-disabled PID stays off.
                const bool restart_pid = is_pid_on;
                is_pid_on = false;
                theta = theta_before_step;
                constexpr double TWO_PI = 6.28318530717958647692;
                for (double& angle : theta) {
                    angle = std::isfinite(angle) ? std::remainder(angle, TWO_PI) : 0.0;
                }
                // Keeping the pre-failure velocity can make every later
                // gravity-only step overflow again. Preserve the equivalent
                // pose, but remove the unstable kinetic state.
                theta_dot.assign(n_lengths, 0.0);
                std::fill(t.begin(), t.end(), 0.0);
                std::fill(a0.begin(), a0.end(), 0.0);
                std::fill(accumulate_error.begin(), accumulate_error.end(), 0.0);
                std::fill(pid_p_term.begin(), pid_p_term.end(), 0.0);
                std::fill(pid_i_term.begin(), pid_i_term.end(), 0.0);
                std::fill(pid_d_term.begin(), pid_d_term.end(), 0.0);
                is_pid_on = restart_pid;
                std::cerr << "non-finite simulation step rejected; unstable state cleared; PID "
                          << (restart_pid ? "restarted" : "remains disabled") << '\n';
            }
        }

        autorob::Json::Array theta_json;
        autorob::Json::Array theta_dot_json;
        autorob::Json::Array torque_json;
        autorob::Json::Array name_json;
        autorob::Json::Array desired_position_json;
        autorob::Json::Array desired_velocity_json;
        autorob::Json::Array position_error_json;
        autorob::Json::Array velocity_error_json;
        autorob::Json::Array p_term_json;
        autorob::Json::Array i_term_json;
        autorob::Json::Array d_term_json;

        //{"header": {...}, "name": [...], "position": [...], "velocity": [...], "effort": [...]}
        for (int i = 0; i < n_lengths; i++) {
            name_json.push_back("joint" + std::to_string(i + 1));
            theta_json.push_back(theta[i]);
            theta_dot_json.push_back(theta_dot[i]);
            torque_json.push_back(t[i]);
            desired_position_json.push_back(desired_position[i]);
            desired_velocity_json.push_back(desired_velocity[i]);
            position_error_json.push_back(pid_position_error[i]);
            velocity_error_json.push_back(pid_velocity_error[i]);
            p_term_json.push_back(pid_p_term[i]);
            i_term_json.push_back(pid_i_term[i]);
            d_term_json.push_back(pid_d_term[i]);
        }

        autorob::Json msg = autorob::Json::Object{
            {"header", autorob::Json::Object{
                {"stamp", make_timestamp(sim_time)},
                {"frame_id", ""}
            }},
            {"name", name_json},
            {"position", theta_json},
            {"velocity", theta_dot_json},
            {"effort", torque_json}
        };

        autorob::Json pid_state_msg = autorob::Json::Object{
            {"enabled", is_pid_on},
            {"name", name_json},
            {"setpoint_position", desired_position_json},
            {"setpoint_velocity", desired_velocity_json},
            {"position_error", position_error_json},
            {"velocity_error", velocity_error_json},
            {"p_term", p_term_json},
            {"i_term", i_term_json},
            {"d_term", d_term_json},
            {"output", torque_json}
        };
        node.publish("/joint_states", msg);
        node.publish("/pid_controller/state", pid_state_msg);
        node.publish("/arm_sim/energy", autorob::Json::Object{{"data", total_energy()}});
        // Keep transport and service-facing publications responsive even when
        // the selected physics timestep is extremely small or large. The
        // integrators and PID still use the exact configured dt.
        sleep_duration = std::clamp(dt, 0.001, 1.0 / 60.0);
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
    }
}
