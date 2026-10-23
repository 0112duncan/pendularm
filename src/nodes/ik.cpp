#include "json.hpp"
#include "node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>
constexpr double PI = 3.14159265358979323846;

namespace {

bool solve_two_link(double x, double y, double first, double second, std::vector<double>& positions, std::string& error) {
    const double radius_squared = x * x + y * y;
    const double maximum = first + second;
    const double minimum = std::abs(first - second);
    constexpr double tolerance = 1e-9;
    if (radius_squared > maximum * maximum + tolerance ||
        radius_squared < minimum * minimum - tolerance) {
        error = "target is outside the two-link workspace";
        return false;
    }

    const double cosine = std::clamp((radius_squared - first * first - second * second) / (2.0 * first * second), -1.0, 1.0);
    const double q2 = std::acos(cosine);
    const double q1 = std::atan2(y, x) - std::atan2(second * std::sin(q2), first + second * std::cos(q2));
    positions = {q1, q2};
    return true;
}

std::tuple<bool, autorob::Json, std::string> solve_ik(const autorob::Json& args, const std::vector<double>& lengths) {
    const autorob::Json* x_value = args.get("x");
    const autorob::Json* y_value = args.get("y");
    if (!x_value || !y_value || !x_value->is_number() || !y_value->is_number()) {
        return {false, autorob::Json::Object{}, "x and y must be numeric"};
    }
    if (lengths.size() != 2 && lengths.size() != 3) {
        return {false, autorob::Json::Object{}, "arm must have two or three links"};
    }

    const double x = x_value->as_number();
    const double y = y_value->as_number();
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return {false, autorob::Json::Object{}, "x and y must be finite"};
    }

    std::vector<double> positions;
    std::string error;
    if (lengths.size() == 2) {
        if (!solve_two_link(x, y, lengths[0], lengths[1], positions, error)) {
            return {false, autorob::Json::Object{}, error};
        }
    } else {
        double phi = 0.0;
        if (const autorob::Json* phi_value = args.get("phi")) {
            if (!phi_value->is_number() || !std::isfinite(phi_value->as_number())) {
                return {false, autorob::Json::Object{}, "phi must be numeric"};
            }
            phi = phi_value->as_number();
        }
        const double wrist_x = x - lengths[2] * std::cos(phi);
        const double wrist_y = y - lengths[2] * std::sin(phi);
        if (!solve_two_link(wrist_x, wrist_y, lengths[0], lengths[1], positions, error)) {
            return {false, autorob::Json::Object{}, error};
        }
        positions.push_back(phi - positions[0] - positions[1]);
    }

    autorob::Json::Array output;
    for (double position : positions) output.push_back(position);
    return {true, autorob::Json::Object{{"positions", output}}, ""};
}

}  // namespace

int main() {
    autorob::Node node("ik");
    node.advertise_service("/ik/solve", "ik/solve", [&node](const autorob::Json& args) {
        const autorob::ServiceResult parameters = node.call_service("/arm_sim/set_params", autorob::Json::Object{});
        const autorob::Json* length_values = parameters.values.get("lengths");
        if (!parameters.result || !length_values || !length_values->is_array()) {
            return autorob::ServiceResult{false, autorob::Json::Object{}, "could not read arm parameters"};
        }
        std::vector<double> lengths;
        for (const autorob::Json& value : length_values->as_array()) {
            if (!value.is_number() || value.as_number() <= 0.0) {
                return autorob::ServiceResult{false, autorob::Json::Object{}, "arm lengths are invalid"};
            }
            lengths.push_back(value.as_number());
        }
        const auto result = solve_ik(args, lengths);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });
    node.wait();
}
