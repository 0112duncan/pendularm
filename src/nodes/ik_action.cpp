#include "json.hpp"
#include "node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

namespace {

struct ActionState {
    bool active = false;
    std::uint64_t counter = 0;
    std::string goal_id;

    double target_x = 0.0;
    double target_y = 0.0;
    double target_phi = 0.0;
    bool has_phi = false;

    double epsilon = 0.05;
    double success_hold = 0.20;
    double start_time = 0.0;
    bool inside_epsilon = false;
    double inside_since = 0.0;

    std::vector<double> commanded_positions;
    std::vector<double> lengths;
    std::vector<double> latest_positions;
    double latest_sim_time = 0.0;
    bool have_joint_state = false;
    double last_distance = 0.0;
};

ActionState action;
std::mutex action_mutex;

bool stamp_to_seconds(const autorob::Json& message, double& time) {
    const autorob::Json* header = message.get("header");
    if (header == nullptr) return false;
    const autorob::Json* stamp = header->get("stamp");
    if (stamp == nullptr) return false;
    const autorob::Json* sec = stamp->get("sec");
    const autorob::Json* nanosec = stamp->get("nanosec");
    if (sec == nullptr || nanosec == nullptr || !sec->is_number() || !nanosec->is_number()) return false;
    time = sec->as_number() + nanosec->as_number()*1e-9;
    return true;
}

autorob::Json make_timestamp(double time) {
    if (time < 0.0) time = 0.0;
    std::uint64_t sec = static_cast<std::uint64_t>(time);
    std::uint64_t nanosec = static_cast<std::uint64_t>((time - static_cast<double>(sec))*1e9);
    if (nanosec >= 1000000000ULL) {
        ++sec;
        nanosec -= 1000000000ULL;
    }
    return autorob::Json::Object{
        {"sec", static_cast<double>(sec)},
        {"nanosec", static_cast<double>(nanosec)}
    };
}

autorob::Json make_target(double x, double y, bool has_phi, double phi) {
    autorob::Json::Object target{{"x", x}, {"y", y}};
    if (has_phi) target["phi"] = phi;
    return target;
}

bool json_array_to_doubles(const autorob::Json* value, std::vector<double>& out) {
    if (value == nullptr || !value->is_array()) return false;
    std::vector<double> temp;
    for (const autorob::Json& item : value->as_array()) {
        if (!item.is_number()) return false;
        temp.push_back(item.as_number());
    }
    out = temp;
    return true;
}

autorob::Json::Array doubles_to_json(const std::vector<double>& values) {
    autorob::Json::Array result;
    for (double value : values) result.push_back(value);
    return result;
}

bool forward_kinematics(const std::vector<double>& q, const std::vector<double>& lengths, double& x, double& y) {
    if (q.size() != lengths.size() || q.empty()) return false;
    x = 0.0;
    y = 0.0;
    double phi = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
        phi += q[i];
        x += lengths[i]*std::cos(phi);
        y += lengths[i]*std::sin(phi);
    }
    return true;
}

double distance_for_state(const std::vector<double>& q, const std::vector<double>& lengths, double target_x, double target_y) {
    double x = 0.0, y = 0.0;
    if (!forward_kinematics(q, lengths, x, y)) return 0.0;
    return std::hypot(target_x - x, target_y - y);
}

void publish_preempted(autorob::Node& node) {
    autorob::Json result_msg;
    bool publish = false;

    {
        std::lock_guard<std::mutex> lock(action_mutex);
        if (!action.active) return;

        double final_distance = distance_for_state(action.latest_positions, action.lengths, action.target_x, action.target_y);
        action.last_distance = final_distance;
        result_msg = autorob::Json::Object{
            {"goal_id", action.goal_id},
            {"outcome", "preempted"},
            {"target", make_target(action.target_x, action.target_y, action.has_phi, action.target_phi)},
            {"final_distance", final_distance}
        };
        action.active = false;
        action.inside_epsilon = false;
        publish = true;
    }

    if (publish) node.publish("/ik_action/result", result_msg);
}

std::tuple<bool, autorob::Json, std::string> send_goal(const autorob::Json& args, autorob::Node& node) {
    const autorob::Json* x_arg = args.get("x");
    const autorob::Json* y_arg = args.get("y");
    const autorob::Json* phi_arg = args.get("phi");
    const autorob::Json* epsilon_arg = args.get("epsilon");
    const autorob::Json* success_hold_arg = args.get("success_hold");

    if (x_arg == nullptr || y_arg == nullptr || !x_arg->is_number() || !y_arg->is_number()) {
        return {false, autorob::Json::Object{}, "x or y missing or not numeric"};
    }

    double x = x_arg->as_number();
    double y = y_arg->as_number();
    bool has_phi = phi_arg != nullptr;
    double phi = 0.0;
    if (has_phi) {
        if (!phi_arg->is_number()) return {false, autorob::Json::Object{}, "phi is not numeric"};
        phi = phi_arg->as_number();
    }

    double epsilon = 0.05;
    double success_hold = 0.20;
    if (epsilon_arg != nullptr) {
        if (!epsilon_arg->is_number() || epsilon_arg->as_number() <= 0.0) {
            return {false, autorob::Json::Object{}, "epsilon must be positive"};
        }
        epsilon = epsilon_arg->as_number();
    }
    if (success_hold_arg != nullptr) {
        if (!success_hold_arg->is_number() || success_hold_arg->as_number() < 0.0) {
            return {false, autorob::Json::Object{}, "success_hold must be non-negative"};
        }
        success_hold = success_hold_arg->as_number();
    }

    autorob::Json::Object ik_request{{"x", x}, {"y", y}};
    if (has_phi) ik_request["phi"] = phi;

    // Solve first. If this fails, the currently-active goal must remain untouched.
    autorob::ServiceResult ik_response = node.call_service("/ik/solve", autorob::Json(ik_request));
    if (!ik_response.result) {
        return {false, autorob::Json::Object{}, "IK target is unreachable or invalid"};
    }

    std::vector<double> commanded_positions;
    if (!json_array_to_doubles(ik_response.values.get("positions"), commanded_positions) || commanded_positions.empty()) {
        return {false, autorob::Json::Object{}, "IK response missing positions"};
    }

    autorob::ServiceResult params = node.call_service("/arm_sim/set_params", autorob::Json::Object{});
    std::vector<double> lengths;
    if (!params.result || !json_array_to_doubles(params.values.get("lengths"), lengths) || lengths.size() != commanded_positions.size()) {
        return {false, autorob::Json::Object{}, "could not get matching arm lengths"};
    }

    // Reachable new goals preempt the old goal before the new one starts.
    publish_preempted(node);

    std::string new_goal_id;
    double start_time = 0.0;
    {
        std::lock_guard<std::mutex> lock(action_mutex);
        ++action.counter;
        new_goal_id = "goal_" + std::to_string(action.counter);
        start_time = action.latest_sim_time;

        action.active = true;
        action.goal_id = new_goal_id;
        action.target_x = x;
        action.target_y = y;
        action.target_phi = phi;
        action.has_phi = has_phi;
        action.epsilon = epsilon;
        action.success_hold = success_hold;
        action.start_time = start_time;
        action.inside_epsilon = false;
        action.commanded_positions = commanded_positions;
        action.lengths = lengths;
        action.last_distance = distance_for_state(action.latest_positions, action.lengths, x, y);
    }

    autorob::Json::Array joint_names;
    autorob::Json::Array zero_velocities;
    for (std::size_t i = 0; i < commanded_positions.size(); ++i) {
        joint_names.push_back("joint" + std::to_string(i + 1));
        zero_velocities.push_back(0.0);
    }

    autorob::Json point = autorob::Json::Object{
        {"positions", doubles_to_json(commanded_positions)},
        {"velocities", zero_velocities},
        {"accelerations", autorob::Json::Array{}},
        {"time_from_start", autorob::Json::Object{{"sec", 0.0}, {"nanosec", 0.0}}}
    };

    autorob::Json trajectory = autorob::Json::Object{
        {"header", autorob::Json::Object{{"stamp", make_timestamp(start_time)}, {"frame_id", ""}}},
        {"joint_names", joint_names},
        {"points", autorob::Json::Array{point}}
    };
    node.publish("/joint_trajectory", trajectory);

    return {true, autorob::Json::Object{{"goal_id", new_goal_id}}, ""};
}

std::tuple<bool, autorob::Json, std::string> cancel_goal(const autorob::Json& args, autorob::Node& node) {
    const autorob::Json* goal_arg = args.get("goal_id");
    if (goal_arg != nullptr && !goal_arg->is_string()) {
        return {false, autorob::Json::Object{}, "goal_id must be a string"};
    }

    autorob::Json result_msg;
    {
        std::lock_guard<std::mutex> lock(action_mutex);
        if (!action.active) {
            return {false, autorob::Json::Object{}, "no active goal"};
        }
        if (goal_arg != nullptr && goal_arg->as_string() != action.goal_id) {
            return {false, autorob::Json::Object{}, "goal_id does not match active goal"};
        }

        double final_distance = distance_for_state(action.latest_positions, action.lengths, action.target_x, action.target_y);
        action.last_distance = final_distance;
        result_msg = autorob::Json::Object{
            {"goal_id", action.goal_id},
            {"outcome", "preempted"},
            {"target", make_target(action.target_x, action.target_y, action.has_phi, action.target_phi)},
            {"final_distance", final_distance}
        };
        action.active = false;
        action.inside_epsilon = false;
    }

    node.publish("/ik_action/result", result_msg);
    return {true, autorob::Json::Object{}, ""};
}

void handle_joint_state(const autorob::Json& message, autorob::Node& node) {
    double sim_time = 0.0;
    std::vector<double> current_positions;
    if (!stamp_to_seconds(message, sim_time)) return;
    if (!json_array_to_doubles(message.get("position"), current_positions)) return;

    autorob::Json feedback_msg;
    autorob::Json reached_msg;
    bool publish_feedback = false;
    bool publish_reached = false;

    {
        std::lock_guard<std::mutex> lock(action_mutex);
        action.latest_sim_time = sim_time;
        action.latest_positions = current_positions;
        action.have_joint_state = true;

        if (!action.active || current_positions.size() != action.lengths.size()) return;

        if (sim_time < action.start_time) {
            action.start_time = sim_time;
            action.inside_epsilon = false;
        }
        double distance = distance_for_state(current_positions, action.lengths, action.target_x, action.target_y);
        action.last_distance = distance;
        double elapsed = std::max(0.0, sim_time - action.start_time);

        feedback_msg = autorob::Json::Object{
            {"goal_id", action.goal_id},
            {"target", make_target(action.target_x, action.target_y, action.has_phi, action.target_phi)},
            {"positions", doubles_to_json(action.commanded_positions)},
            {"distance_remaining", distance},
            {"elapsed", elapsed}
        };
        publish_feedback = true;

        if (distance <= action.epsilon) {
            if (!action.inside_epsilon) {
                action.inside_epsilon = true;
                action.inside_since = sim_time;
            }

            if (action.success_hold == 0.0 || sim_time - action.inside_since >= action.success_hold) {
                reached_msg = autorob::Json::Object{
                    {"goal_id", action.goal_id},
                    {"outcome", "reached"},
                    {"target", make_target(action.target_x, action.target_y, action.has_phi, action.target_phi)},
                    {"final_distance", distance}
                };
                action.active = false;
                action.inside_epsilon = false;
                publish_reached = true;
            }
        }
        else {
            action.inside_epsilon = false;
        }
    }

    if (publish_feedback) node.publish("/ik_action/feedback", feedback_msg);
    if (publish_reached) node.publish("/ik_action/result", reached_msg);
}

}

int main() {
    autorob::Node node("ik_action");

    node.advertise("/joint_trajectory", "trajectory_msgs/JointTrajectory");
    node.advertise("/ik_action/feedback", "ik_action/feedback");
    node.advertise("/ik_action/result", "ik_action/result");

    node.advertise_service("/ik_action/send_goal", "ik_action/send_goal", [&node](const autorob::Json& args) {
        auto result = send_goal(args, node);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });

    node.advertise_service("/ik_action/cancel_goal", "ik_action/cancel_goal", [&node](const autorob::Json& args) {
        auto result = cancel_goal(args, node);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });

    node.subscribe("/joint_states", "sensor_msgs/JointState", [&node](const autorob::Json& message) {
        handle_joint_state(message, node);
    });

    node.wait();
}
