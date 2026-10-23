#include "json.hpp"
#include "node.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

constexpr double PI = 3.14159265358979323846;

struct TrialState {
    bool running = false;
    double duration = 10.0;
    double epsilon = 0.5;
    double success_hold = 0.20;
    std::size_t targets_reached = 0;
    std::chrono::steady_clock::time_point start_time;
    double elapsed = 0.0;

    std::string current_goal_id;
    bool has_target = false;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_phi = 0.0;
    bool has_phi = false;

    bool have_error = false;
    double error = 0.0;
    bool have_desired_positions = false;
    std::vector<double> desired_positions;
    std::string action_status = "idle";
    bool next_target_pending = false;
    std::string early_result_goal_id;
    std::string early_result_outcome;
};

TrialState trial;
std::mutex trial_mutex;

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

autorob::Json make_target(double x, double y, bool has_phi, double phi) {
    autorob::Json::Object target{{"x", x}, {"y", y}};
    if (has_phi) target["phi"] = phi;
    return target;
}

double elapsed_now_locked() {
    if (!trial.running) return trial.elapsed;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - trial.start_time).count();
}

void clear_inflight_locked() {
    trial.current_goal_id.clear();
    trial.has_target = false;
    trial.have_error = false;
    trial.have_desired_positions = false;
    trial.desired_positions.clear();
    trial.action_status = "idle";
    trial.next_target_pending = false;
    trial.early_result_goal_id.clear();
    trial.early_result_outcome.clear();
}

void cancel_goal_id(autorob::Node& node, const std::string& goal_id) {
    if (goal_id.empty()) return;
    node.call_service("/ik_action/cancel_goal", autorob::Json::Object{{"goal_id", goal_id}});
}

void publish_status(autorob::Node& node) {
    autorob::Json message;
    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        if (trial.running) trial.elapsed = elapsed_now_locked();

        autorob::Json::Object status{
            {"running", trial.running},
            {"elapsed", trial.elapsed},
            {"duration", trial.duration},
            {"targets_reached", static_cast<double>(trial.targets_reached)},
            {"action_status", trial.action_status}
        };
        status["target"] = trial.has_target
            ? make_target(trial.target_x, trial.target_y, trial.has_phi, trial.target_phi)
            : autorob::Json(nullptr);
        status["error"] = trial.have_error
            ? autorob::Json(trial.error)
            : autorob::Json(nullptr);
        status["desired_positions"] = trial.have_desired_positions
            ? autorob::Json(doubles_to_json(trial.desired_positions))
            : autorob::Json(nullptr);
        message = autorob::Json(std::move(status));
    }
    node.publish("/ik_trial/status", message);
}

bool sample_and_send_target(autorob::Node& node, std::string& error_message) {
    autorob::ServiceResult params = node.call_service("/arm_sim/set_params", autorob::Json::Object{});
    std::vector<double> lengths;
    if (!params.result || !json_array_to_doubles(params.values.get("lengths"), lengths) || (lengths.size() != 2 && lengths.size() != 3)) {
        error_message = "could not get arm lengths";
        return false;
    }

    double epsilon = 0.0;
    double success_hold = 0.0;
    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        if (!trial.running) {
            error_message = "trial is not running";
            return false;
        }
        epsilon = trial.epsilon;
        success_hold = trial.success_hold;
    }

    static thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> angle_dist(-PI, PI);

    for (int attempt = 0; attempt < 25; ++attempt) {
        std::vector<double> q(lengths.size());
        for (double& angle : q) angle = angle_dist(rng);

        double x = 0.0, y = 0.0, phi = 0.0;
        for (std::size_t i = 0; i < q.size(); ++i) {
            phi += q[i];
            x += lengths[i]*std::cos(phi);
            y += lengths[i]*std::sin(phi);
        }

        autorob::Json::Object request{
            {"x", x},
            {"y", y},
            {"epsilon", epsilon},
            {"success_hold", success_hold}
        };
        if (lengths.size() == 3) request["phi"] = phi;

        {
            std::lock_guard<std::mutex> lock(trial_mutex);
            if (!trial.running) {
                error_message = "trial is not running";
                return false;
            }
            trial.action_status = "starting";
            trial.early_result_goal_id.clear();
            trial.early_result_outcome.clear();
        }

        autorob::ServiceResult response = node.call_service("/ik_action/send_goal", autorob::Json(request));
        if (!response.result) continue;

        const autorob::Json* goal_id_arg = response.values.get("goal_id");
        if (goal_id_arg == nullptr || !goal_id_arg->is_string()) continue;
        std::string new_goal_id = goal_id_arg->as_string();

        bool still_running = false;
        {
            std::lock_guard<std::mutex> lock(trial_mutex);
            still_running = trial.running;
            if (still_running) {
                const bool reached_early =
                    trial.early_result_goal_id == new_goal_id &&
                    trial.early_result_outcome == "reached";
                trial.early_result_goal_id.clear();
                trial.early_result_outcome.clear();
                if (reached_early) {
                    ++trial.targets_reached;
                    trial.current_goal_id.clear();
                    trial.has_target = false;
                    trial.action_status = "reached";
                    trial.next_target_pending = true;
                } else {
                    trial.current_goal_id = new_goal_id;
                    trial.has_target = true;
                    trial.target_x = x;
                    trial.target_y = y;
                    trial.target_phi = phi;
                    trial.has_phi = lengths.size() == 3;
                    trial.have_error = false;
                    trial.have_desired_positions = false;
                    trial.desired_positions.clear();
                    trial.action_status = "active";
                }
            }
        }

        if (!still_running) {
            cancel_goal_id(node, new_goal_id);
            error_message = "trial stopped while starting target";
            return false;
        }
        return true;
    }

    error_message = "could not sample a reachable target";
    return false;
}

std::tuple<bool, autorob::Json, std::string> start_trial(const autorob::Json& args, autorob::Node& node) {
    double duration;
    double epsilon;
    double success_hold;
    std::string old_goal_id;

    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        duration = trial.duration;
        epsilon = trial.epsilon;
        success_hold = trial.success_hold;
        old_goal_id = trial.current_goal_id;
    }

    const autorob::Json* duration_arg = args.get("duration");
    const autorob::Json* epsilon_arg = args.get("epsilon");
    const autorob::Json* hold_arg = args.get("success_hold");

    if (duration_arg != nullptr) {
        if (!duration_arg->is_number() || duration_arg->as_number() <= 0.0) {
            return {false, autorob::Json::Object{}, "duration must be positive"};
        }
        duration = duration_arg->as_number();
    }
    if (epsilon_arg != nullptr) {
        if (!epsilon_arg->is_number() || epsilon_arg->as_number() <= 0.0) {
            return {false, autorob::Json::Object{}, "epsilon must be positive"};
        }
        epsilon = epsilon_arg->as_number();
    }
    if (hold_arg != nullptr) {
        if (!hold_arg->is_number() || hold_arg->as_number() < 0.0) {
            return {false, autorob::Json::Object{}, "success_hold must be non-negative"};
        }
        success_hold = hold_arg->as_number();
    }

    // Only cancel a goal that belonged to the previous trial.
    cancel_goal_id(node, old_goal_id);

    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        trial.running = true;
        trial.duration = duration;
        trial.epsilon = epsilon;
        trial.success_hold = success_hold;
        trial.targets_reached = 0;
        trial.start_time = std::chrono::steady_clock::now();
        trial.elapsed = 0.0;
        clear_inflight_locked();
    }

    std::string sample_error;
    if (!sample_and_send_target(node, sample_error)) {
        std::lock_guard<std::mutex> lock(trial_mutex);
        trial.elapsed = elapsed_now_locked();
        trial.running = false;
        clear_inflight_locked();
        return {false, autorob::Json::Object{}, sample_error};
    }

    publish_status(node);
    return {true, autorob::Json::Object{}, ""};
}

std::tuple<bool, autorob::Json, std::string> skip_trial(const autorob::Json&, autorob::Node& node) {
    std::string old_goal_id;
    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        if (!trial.running) return {false, autorob::Json::Object{}, "no trial is running"};
        trial.elapsed = elapsed_now_locked();
        if (trial.elapsed >= trial.duration) return {false, autorob::Json::Object{}, "trial duration elapsed"};
        old_goal_id = trial.current_goal_id;
    }

    cancel_goal_id(node, old_goal_id);

    std::string sample_error;
    if (!sample_and_send_target(node, sample_error)) {
        std::lock_guard<std::mutex> lock(trial_mutex);
        trial.elapsed = elapsed_now_locked();
        trial.running = false;
        clear_inflight_locked();
        return {false, autorob::Json::Object{}, sample_error};
    }

    publish_status(node);
    return {true, autorob::Json::Object{}, ""};
}

std::tuple<bool, autorob::Json, std::string> stop_trial(const autorob::Json&, autorob::Node& node) {
    std::string old_goal_id;
    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        if (!trial.running) return {false, autorob::Json::Object{}, "no trial is running"};
        trial.elapsed = elapsed_now_locked();
        old_goal_id = trial.current_goal_id;
        trial.running = false;
        clear_inflight_locked();
    }

    cancel_goal_id(node, old_goal_id);
    publish_status(node);
    return {true, autorob::Json::Object{}, ""};
}

void handle_action_feedback(const autorob::Json& message) {
    const autorob::Json* goal_id_arg = message.get("goal_id");
    const autorob::Json* error_arg = message.get("distance_remaining");
    if (goal_id_arg == nullptr || !goal_id_arg->is_string()) return;

    std::vector<double> desired_positions;
    bool have_positions = json_array_to_doubles(message.get("positions"), desired_positions);

    std::lock_guard<std::mutex> lock(trial_mutex);
    if (!trial.running || goal_id_arg->as_string() != trial.current_goal_id) return;

    if (error_arg != nullptr && error_arg->is_number()) {
        trial.error = error_arg->as_number();
        trial.have_error = true;
    }
    if (have_positions) {
        trial.desired_positions = desired_positions;
        trial.have_desired_positions = true;
    }
    trial.action_status = "active";
}

void handle_action_result(const autorob::Json& message, autorob::Node& node) {
    const autorob::Json* goal_id_arg = message.get("goal_id");
    const autorob::Json* outcome_arg = message.get("outcome");
    if (goal_id_arg == nullptr || outcome_arg == nullptr ||
        !goal_id_arg->is_string() || !outcome_arg->is_string()) return;

    bool publish_now = false;
    {
        std::lock_guard<std::mutex> lock(trial_mutex);
        if (!trial.running) return;

        if (trial.current_goal_id.empty() && trial.action_status == "starting") {
            trial.early_result_goal_id = goal_id_arg->as_string();
            trial.early_result_outcome = outcome_arg->as_string();
            return;
        }
        if (goal_id_arg->as_string() != trial.current_goal_id) return;

        trial.elapsed = elapsed_now_locked();
        const std::string outcome = outcome_arg->as_string();
        if (outcome == "reached") {
            ++trial.targets_reached;
            trial.action_status = "reached";
            trial.current_goal_id.clear();
            if (trial.elapsed >= trial.duration) {
                trial.running = false;
                clear_inflight_locked();
                publish_now = true;
            } else {
                // Topic callbacks run on the middleware receiver thread. Queue
                // service work for the main loop so replies can still be received.
                trial.next_target_pending = true;
            }
        } else if (outcome == "preempted") {
            trial.action_status = "preempted";
        }
    }

    if (publish_now) publish_status(node);
}

}

int main() {
    autorob::Node node("ik_trial");

    node.advertise("/ik_trial/status", "ik_trial/status");

    node.advertise_service("/ik_trial/start", "ik_trial/start", [&node](const autorob::Json& args) {
        auto result = start_trial(args, node);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });

    node.advertise_service("/ik_trial/skip", "ik_trial/skip", [&node](const autorob::Json& args) {
        auto result = skip_trial(args, node);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });

    node.advertise_service("/ik_trial/stop", "ik_trial/stop", [&node](const autorob::Json& args) {
        auto result = stop_trial(args, node);
        return autorob::ServiceResult{std::get<0>(result), std::get<1>(result), std::get<2>(result)};
    });

    node.subscribe("/ik_action/feedback", "ik_action/feedback", [](const autorob::Json& message) {
        handle_action_feedback(message);
    });

    node.subscribe("/ik_action/result", "ik_action/result", [&node](const autorob::Json& message) {
        handle_action_result(message, node);
    });

    while (true) {
        bool running = false;
        bool expired = false;
        bool start_next = false;
        std::string goal_to_cancel;

        {
            std::lock_guard<std::mutex> lock(trial_mutex);
            running = trial.running;
            if (running) {
                trial.elapsed = elapsed_now_locked();
                if (trial.elapsed >= trial.duration) {
                    expired = true;
                    goal_to_cancel = trial.current_goal_id;
                    trial.running = false;
                    clear_inflight_locked();
                } else if (trial.next_target_pending) {
                    trial.next_target_pending = false;
                    start_next = true;
                }
            }
        }

        if (expired) {
            cancel_goal_id(node, goal_to_cancel);
            publish_status(node);
        } else if (start_next) {
            std::string sample_error;
            if (!sample_and_send_target(node, sample_error)) {
                std::lock_guard<std::mutex> lock(trial_mutex);
                trial.elapsed = elapsed_now_locked();
                trial.running = false;
                clear_inflight_locked();
                trial.action_status = "error: " + sample_error;
            }
            publish_status(node);
        } else if (running) {
            publish_status(node);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}