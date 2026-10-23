#!/usr/bin/env python3
"""Black-box PID stability checks through the public runtime protocol."""

import math

from test_dynamics_timeouts import start_runtime, stop_runtime
from test_project_protocol import Client, check_joint_state, timestamp, wait_for_service


TARGETS = (
    [0.5, -0.35, 0.25],
    [-1.2, 0.0, 0.0],
    [1.2, 0.0, 0.0],
    [0.0, 1.2, -1.2],
    [-2.5, 1.5, 0.8],
)


def run_target(caller, listener, target, duration=6.0):
    assert caller.call("/arm_sim/pause", {"data": True})["result"]
    assert caller.call("/arm_sim/reset", {})["result"]
    caller.publish("/joint_trajectory", {"points": [{"positions": target}]})
    assert caller.call("/pid_controller/enable", {"data": True})["result"]
    assert caller.call("/arm_sim/pause", {"data": False})["result"]

    peak_speed = 0.0
    peak_effort = 0.0
    last = None
    while last is None or timestamp(last) < duration:
        last = listener.next_publication("/joint_states", timeout=2.0)
        check_joint_state(last, len(target))
        peak_speed = max(peak_speed, *(abs(value) for value in last["velocity"]))
        peak_effort = max(peak_effort, *(abs(value) for value in last["effort"]))

    final_error = max(
        abs(target[index] - last["position"][index])
        for index in range(len(target))
    )
    # A very large step can create a fast but finite transient.  The important
    # regression is that it damps out instead of growing on every update.
    assert math.isfinite(peak_speed) and peak_speed < 30.0, (target, peak_speed, last)
    assert math.isfinite(peak_effort) and peak_effort < 1000.0, (target, peak_effort, last)
    assert final_error < 0.18, (target, final_error, last)
    return final_error, peak_speed, peak_effort


def main():
    runtime = start_runtime(3)
    clients = []
    try:
        caller = Client()
        listener = Client()
        clients.extend((caller, listener))
        wait_for_service(caller, "/arm_sim/set_params")
        assert caller.call("/arm_sim/set_integrator", {
            "method": "rk4", "timestep": 0.01,
        })["result"]
        assert caller.call("/arm_sim/set_params", {
            "gravity": 9.81,
            "masses": [1.0, 1.0, 1.0],
            "lengths": [1.0, 1.0, 1.0],
        })["result"]
        listener.subscribe("/joint_states", "sensor_msgs/JointState")

        for target in TARGETS:
            error, speed, effort = run_target(caller, listener, target)
            print(
                f"target={target}: final_error={error:.4f}, "
                f"peak_speed={speed:.3f}, peak_effort={effort:.3f}"
            )
        print("PASS: 3-link default PID remains stable across target poses")
    finally:
        stop_runtime(runtime, clients)


if __name__ == "__main__":
    main()
