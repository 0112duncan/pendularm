#include "json.hpp"
#include "node.hpp"

#include <cstdint>
#include <iostream>
#include <cmath>
#include <tuple>

//{"function": "<expression in t>", "x0": <f64>, "xdot0": <f64, optional, default 0>, "dt": <f64>, "steps": <u64>, "integrator": "euler"|"midpoint"|"verlet"|"rk4"}

double parse_add_sub(const std::string& function, std::size_t& index, double t0, bool& valid);

void skip_space(const std::string& function, std::size_t& index) {
    while(index < function.size() && function[index] == ' ') {
        index++;
    }
}

double parse_atom(const std::string& function, std::size_t& index, double t0, bool& valid) {
    skip_space(function, index);
    if (index >= function.size()) {
        valid = false;
        return 0;
    }

    if (function[index] >= '0' && function[index] <= '9') {
        std::string number;
        bool has_dot = false;
        while (index < function.size() && (function[index] == '.' || (function[index] >= '0' && function[index] <= '9')) ) {
            if (function[index] >= '0' && function[index] <= '9') {
                number.push_back(function[index]);
            }
            if (function[index] == '.'){
                if (index + 1 >= function.size() || has_dot || !(function[index + 1] >= '0' && function[index + 1] <= '9')) {
                    valid = false;
                    break;
                }
                number.push_back(function[index]);
                has_dot = true;
            }
            index++;
        }
        return(std::stod(number));
    }

    if (function[index] == 't' && function.compare(index, 3, "tan") != 0) {
        index++;
        return t0;
    }

    // named functions: cos sin tan exp ln sqrt abs
    if (function.compare(index, 3, "cos") == 0) {
        index += 3;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::cos(inside);  //radian
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 3, "sin") == 0) {
        index += 3;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::sin(inside);  //radian
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 3, "tan") == 0) {
        index += 3;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::tan(inside);  
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 3, "exp") == 0) {
        index += 3;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::exp(inside); 
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 3, "abs") == 0) {
        index += 3;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::abs(inside);  
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 4, "sqrt") == 0) {
        index += 4;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::sqrt(inside); 
        }
        else{
            valid = false;
            return 0;
        }
    }

    if (function.compare(index, 2, "ln") == 0) {
        index += 2;
        if (index < function.size() && function[index] == '(') {
            index++;
            double inside = parse_add_sub(function, index, t0, valid);
            if (index >= function.size() || function[index] != ')') {
                valid = false;
                return 0;
            }
            index++;
            return std::log(inside); 
        }
        else{
            valid = false;
            return 0;
        }
    }

    //parenthese
    if (function[index] == '(') {
        index++;  // move past '('
        double inside = parse_add_sub(function, index, t0, valid);
        skip_space(function, index);

        if (index >= function.size() || function[index] != ')') {
            valid = false;
            return 0;
        }

        index++;  // move past ')'
        return inside;
    }

    // otherwise invalid
    valid = false;
    return 0;
}

double parse_unary(const std::string& function, std::size_t& index, double t0, bool& valid) {
    skip_space(function, index);
    if (index + 1 < function.size() && function[index] == '-') {
        index++;
        // consume -
        return -parse_unary(function, index, t0, valid);
    }
    return parse_atom(function, index, t0, valid);
}

double parse_power(const std::string& function, std::size_t& index, double t0, bool& valid) {
    skip_space(function, index);
    double left = parse_unary(function, index, t0, valid);
    skip_space(function, index);

    while (index + 1 < function.size() && function[index] == '^'){
        index++;
        // consume ^
        double power = parse_power(function, index, t0, valid);
        left = std::pow(left, power);
        skip_space(function, index);
    }
    return left;
}

double parse_mul_div(const std::string& function, std::size_t& index, double t0, bool& valid) {
    skip_space(function, index);
    double left = parse_power(function, index, t0, valid);
    skip_space(function, index);

    while (index + 1 < function.size() && (function[index] == '*' || function[index] == '/')){
        skip_space(function, index);
        if (index + 1 < function.size() && function[index] == '*') {
            index++;
            // consume *
            double right = parse_power(function, index, t0, valid);
            left = left * right;
            skip_space(function, index);
        }
        else if (index + 1 < function.size() && function[index] == '/') {
            index++;
            // consume /
            double right = parse_power(function, index, t0, valid);
            left = left / right;
            skip_space(function, index);
        }
    }
    return left;
}

double parse_add_sub(const std::string& function, std::size_t& index, double t0, bool& valid) {
    skip_space(function, index);
    double left = parse_mul_div(function, index, t0, valid);
    skip_space(function, index);

    while (index + 1 < function.size() && (function[index] == '+' || function[index] == '-')){
        if (index + 1 < function.size() && function[index] == '+') {
            index++;
            // consume +
            double right = parse_mul_div(function, index, t0, valid);
            left = left + right;
            skip_space(function, index);
        }
        else if (index + 1 < function.size() && function[index] == '-') {
            index++;
            // consume -
            double right = parse_mul_div(function, index, t0, valid);
            left = left - right;
            skip_space(function, index);
        }
    }
    return left;
}

std::pair<double, bool> function_t(const std::string& function, double t0) {
    std::size_t index = 0;
    bool valid = true;
    skip_space(function, index);
    double value = parse_add_sub(function, index, t0, valid);
    skip_space(function, index);

    if (index != function.size()){
        valid = false;
        value = 0;
    }
    return {value, valid};
}

std::tuple<bool, autorob::Json, std::string> integrator(const autorob::Json& args) {
    bool valid = true;
    std::string error_msg;
    std::string function;
    std::string method_name;
    double x0 = 0.0;
    double xdot0 = 0.0;
    double dt = 0.0;
    std::uint64_t steps = 0;

    auto reject = [&](const std::string& message) {
        valid = false;
        error_msg = message;
    };

    const autorob::Json* function_arg = args.get("function");
    const autorob::Json* x0_arg = args.get("x0");
    const autorob::Json* dt_arg = args.get("dt");
    const autorob::Json* steps_arg = args.get("steps");
    const autorob::Json* method_arg = args.get("integrator");
    const autorob::Json* xdot_arg = args.get("xdot0");

    if (!function_arg || !x0_arg || !dt_arg || !steps_arg || !method_arg) {
        reject("function, x0, dt, steps, and integrator are required");
    } else if (!function_arg->is_string() || !x0_arg->is_number() ||
               !dt_arg->is_number() || !steps_arg->is_number() ||
               !method_arg->is_string()) {
        reject("invalid argument type");
    } else if (xdot_arg && !xdot_arg->is_number()) {
        reject("xdot0 must be numeric");
    } else {
        function = function_arg->as_string();
        x0 = x0_arg->as_number();
        xdot0 = xdot_arg ? xdot_arg->as_number() : 0.0;
        dt = dt_arg->as_number();
        method_name = method_arg->as_string();
        const double requested_steps = steps_arg->as_number();

        if (!std::isfinite(x0) || !std::isfinite(xdot0) || !std::isfinite(dt) ||
            !std::isfinite(requested_steps)) {
            reject("numeric arguments must be finite");
        } else if (dt <= 0.0) {
            reject("dt must be positive");
        } else if (requested_steps < 1.0 || std::floor(requested_steps) != requested_steps) {
            reject("steps must be a positive integer");
        } else if (requested_steps > 1000000.0) {
            reject("steps exceeds the service limit of 1000000");
        } else if (method_name != "euler" && method_name != "midpoint" &&
                   method_name != "verlet" && method_name != "rk4") {
            reject("integrator must be euler, midpoint, verlet, or rk4");
        } else {
            steps = static_cast<std::uint64_t>(requested_steps);
        }
    }

    std::vector<double> xdot1 = {xdot0};
    std::vector<double> x1 = {x0};
    std::vector<double> time = {0.0};

    auto acceleration = [&](double t_value, double& value) {
        const auto parsed = function_t(function, t_value);
        if (!parsed.second || !std::isfinite(parsed.first)) {
            reject("function is invalid or produced a non-finite value");
            return false;
        }
        value = parsed.first;
        return true;
    };

    for (std::uint64_t i = 0; valid && i < steps; ++i) {
        const double current_t = time.back();
        const double current_x = x1.back();
        const double current_v = xdot1.back();
        double next_x = current_x;
        double next_v = current_v;
        double a_start = 0.0;

        if (!acceleration(current_t, a_start)) break;

        if (method_name == "euler") {
            next_x = current_x + current_v * dt;
            next_v = current_v + a_start * dt;
        } else if (method_name == "midpoint") {
            double a_mid = 0.0;
            if (!acceleration(current_t + dt / 2.0, a_mid)) break;
            const double v_mid = current_v + a_start * dt / 2.0;
            next_x = current_x + v_mid * dt;
            next_v = current_v + a_mid * dt;
        } else if (method_name == "verlet") {
            double a_end = 0.0;
            if (!acceleration(current_t + dt, a_end)) break;
            next_x = current_x + current_v * dt + a_start * dt * dt / 2.0;
            next_v = current_v + (a_start + a_end) * dt / 2.0;
        } else {
            double k2_a = 0.0, k3_a = 0.0, k4_a = 0.0;
            if (!acceleration(current_t + dt / 2.0, k2_a) ||
                !acceleration(current_t + dt / 2.0, k3_a) ||
                !acceleration(current_t + dt, k4_a)) break;
            const double k1_v = current_v;
            const double k2_v = current_v + a_start * dt / 2.0;
            const double k3_v = current_v + k2_a * dt / 2.0;
            const double k4_v = current_v + k3_a * dt;
            next_x = current_x +
                     (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) * dt / 6.0;
            next_v = current_v +
                     (a_start + 2.0 * k2_a + 2.0 * k3_a + k4_a) * dt / 6.0;
        }

        if (!std::isfinite(next_x) || !std::isfinite(next_v)) {
            reject("integration produced a non-finite result");
            break;
        }
        x1.push_back(next_x);
        xdot1.push_back(next_v);
        time.push_back(static_cast<double>(i + 1) * dt);
    }
    //returns {"times": [...], "positions": [...], "velocities": [...]}
    autorob::Json::Array times_json;
    autorob::Json::Array x1_json;
    autorob::Json::Array xdot1_json;

    for (std::size_t i = 0; i < time.size(); ++i) {
        times_json.push_back(time[i]);
        x1_json.push_back(x1[i]);
        xdot1_json.push_back(xdot1[i]);
    }

    autorob::Json answer = autorob::Json(autorob::Json::Object{
        {"times", times_json},
        {"positions", x1_json},
        {"velocities", xdot1_json}
    });

    return {valid, answer, error_msg};
}

int main(){
    autorob::Node node("arm_sim");

    node.advertise_service("/arm_sim/integration_step", "arm_sim/integration_step", [](const autorob::Json& args) {
            auto result = integrator(args);
            return autorob::ServiceResult{ std::get<0>(result), std::get<1>(result), std::get<2>(result)};
        }
    );

    node.wait();
}
