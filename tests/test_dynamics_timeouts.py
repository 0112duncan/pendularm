#!/usr/bin/env python3
"""Focused service-latency, arm-dynamics, and default-PID regression checks."""

import concurrent.futures
import math
import os
import signal
import statistics
import subprocess
import time

from test_project_protocol import (
    Client,
    ROOT,
    assert_rejected,
    check_joint_state,
    timestamp,
    wait_for_runtime,
    wait_for_service,
)


def solve(matrix, rhs):
    size = len(rhs)
    augmented = [matrix[row][:] + [rhs[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        assert abs(augmented[pivot][column]) > 1e-12
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        for row in range(column + 1, size):
            factor = augmented[row][column] / augmented[column][column]
            for entry in range(column, size + 1):
                augmented[row][entry] -= factor * augmented[column][entry]
    result = [0.0] * size
    for row in range(size - 1, -1, -1):
        result[row] = (
            augmented[row][size]
            - sum(augmented[row][column] * result[column]
                  for column in range(row + 1, size))
        ) / augmented[row][row]
    return result


def reference_acceleration(q, qdot, masses, lengths, gravity, effort):
    """General n-link uniform-rod dynamics from COM Jacobians."""
    count = len(q)
    phi = []
    omega = []
    angle = 0.0
    angular_velocity = 0.0
    for index in range(count):
        angle += q[index]
        angular_velocity += qdot[index]
        phi.append(angle)
        omega.append(angular_velocity)

    mass_matrix = [[0.0] * count for _ in range(count)]
    coriolis = [0.0] * count
    gravity_load = [0.0] * count

    for link in range(count):
        jacobian_x = [0.0] * count
        jacobian_y = [0.0] * count
        for joint in range(link + 1):
            for segment in range(joint, link):
                jacobian_x[joint] -= lengths[segment] * math.sin(phi[segment])
                jacobian_y[joint] += lengths[segment] * math.cos(phi[segment])
            jacobian_x[joint] -= 0.5 * lengths[link] * math.sin(phi[link])
            jacobian_y[joint] += 0.5 * lengths[link] * math.cos(phi[link])

        bias_x = 0.0
        bias_y = 0.0
        for segment in range(link):
            bias_x -= lengths[segment] * math.cos(phi[segment]) * omega[segment] ** 2
            bias_y -= lengths[segment] * math.sin(phi[segment]) * omega[segment] ** 2
        bias_x -= 0.5 * lengths[link] * math.cos(phi[link]) * omega[link] ** 2
        bias_y -= 0.5 * lengths[link] * math.sin(phi[link]) * omega[link] ** 2

        inertia = masses[link] * lengths[link] ** 2 / 12.0
        for first in range(link + 1):
            gravity_load[first] += masses[link] * gravity * jacobian_y[first]
            coriolis[first] += masses[link] * (
                jacobian_x[first] * bias_x + jacobian_y[first] * bias_y
            )
            for second in range(link + 1):
                mass_matrix[first][second] += masses[link] * (
                    jacobian_x[first] * jacobian_x[second]
                    + jacobian_y[first] * jacobian_y[second]
                ) + inertia

    rhs = [
        effort[index] - coriolis[index] - gravity_load[index]
        for index in range(count)
    ]
    return solve(mass_matrix, rhs)


def start_runtime(links):
    environment = os.environ.copy()
    environment["ARM_SIM_LINKS"] = str(links)
    process = subprocess.Popen(
        [str(ROOT / "scripts/run.sh")],
        cwd=ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    wait_for_runtime()
    return process


def stop_runtime(process, clients):
    for client in clients:
        try:
            client.close()
        except OSError:
            pass
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2)
    time.sleep(0.15)


def timed_call(client, service, arguments, timeout=1.0):
    started = time.monotonic()
    response = client.call(service, arguments, timeout=timeout)
    return time.monotonic() - started, response


def audit_service_latency(caller, links, clients):
    durations = []
    queries = (
        ("/arm_sim/set_integrator", {}),
        ("/arm_sim/set_params", {}),
        ("/arm_sim/pause", {}),
        ("/pid_controller/enable", {}),
        ("/pid_controller/set_gains", {}),
        ("/arm_sim/reset", {}),
        ("/ik/solve", {"x": 1.0, "y": 1.0, **({"phi": 0.0} if links == 3 else {})}),
        ("/arm_sim/integration_step", {
            "function": "t", "x0": 0.0, "dt": 0.01,
            "steps": 20, "integrator": "rk4",
        }),
    )
    for _ in range(3):
        for service, arguments in queries:
            duration, response = timed_call(caller, service, arguments)
            assert response["result"], (service, response)
            durations.append(duration)

    for malformed in (
        {"function": "t", "x0": 0, "dt": 0.01, "steps": -1, "integrator": "rk4"},
        {"function": "t", "x0": 0, "dt": 0.01, "steps": 1.5, "integrator": "rk4"},
        {"function": "t", "x0": 0, "dt": 0.01, "steps": 1_000_001, "integrator": "rk4"},
    ):
        duration, response = timed_call(caller, "/arm_sim/integration_step", malformed)
        assert_rejected(response)
        durations.append(duration)

    assert caller.call("/arm_sim/pause", {"data": True})["result"]
    worker_clients = [Client() for _ in range(8)]
    clients.extend(worker_clients)

    def query_many(client):
        local = []
        for _ in range(12):
            elapsed, response = timed_call(client, "/arm_sim/set_params", {})
            assert response["result"]
            local.append(elapsed)
        return local

    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        for result in executor.map(query_many, worker_clients):
            durations.extend(result)

    # A large physics dt must not turn pause snapshots into multi-second gaps.
    assert caller.call("/arm_sim/set_integrator",
                       {"method": "euler", "timestep": 5.0})["result"]
    snapshot = Client(timeout=0.5)
    clients.append(snapshot)
    snapshot.subscribe("/joint_states", "sensor_msgs/JointState")
    first = snapshot.next_publication("/joint_states", timeout=0.5)
    second = snapshot.next_publication("/joint_states", timeout=0.5)
    assert timestamp(first) == timestamp(second)
    assert caller.call("/arm_sim/set_integrator",
                       {"method": "euler", "timestep": 0.02})["result"]

    assert max(durations) < 0.75, max(durations)
    return statistics.median(durations), max(durations)


def audit_velocity_pid(caller, links, clients):
    assert caller.call("/arm_sim/pause", {"data": True})["result"]
    assert caller.call("/arm_sim/set_integrator",
                       {"method": "euler", "timestep": 0.005})["result"]
    assert caller.call("/arm_sim/set_params", {
        "gravity": 0.0,
        "masses": [1.0] * links,
        "lengths": [1.0] * links,
    })["result"]
    assert caller.call("/arm_sim/reset", {})["result"]
    assert caller.call("/pid_controller/set_gains", {
        "kp": [0.0] * links,
        "ki": [0.0] * links,
        "kd": [2.0] * links,
    })["result"]
    caller.publish("/joint_trajectory", {
        "points": [{"positions": [0.0] * links, "velocities": [1.0] * links}]
    })
    assert caller.call("/pid_controller/enable", {"data": True})["result"]
    listener = Client()
    clients.append(listener)
    listener.subscribe("/joint_states", "sensor_msgs/JointState")
    assert caller.call("/arm_sim/pause", {"data": False})["result"]
    while True:
        state = listener.next_publication("/joint_states")
        if timestamp(state) > 0:
            break
    assert max(abs(value - 2.0) for value in state["effort"]) < 1e-9, state
    assert caller.call("/pid_controller/enable", {"data": False})["result"]


def audit_dynamics(caller, links, clients):
    dt = 0.005
    masses = [0.8 + 0.35 * index for index in range(links)]
    lengths = [0.7 + 0.2 * index for index in range(links)]
    gravity = 6.3
    gains = [4.0 - 0.5 * index for index in range(links)]
    target = [0.8, -0.6, 0.4][:links]

    assert caller.call("/arm_sim/pause", {"data": True})["result"]
    assert caller.call("/arm_sim/set_integrator",
                       {"method": "euler", "timestep": dt})["result"]
    assert caller.call("/arm_sim/set_params", {
        "gravity": gravity, "masses": masses, "lengths": lengths,
    })["result"]
    assert caller.call("/arm_sim/reset", {})["result"]
    assert caller.call("/pid_controller/set_gains", {
        "kp": gains, "ki": [0.0] * links, "kd": [0.0] * links,
    })["result"]
    caller.publish("/joint_trajectory", {
        "points": [{"positions": target, "velocities": [0.0] * links}]
    })
    assert caller.call("/pid_controller/enable", {"data": True})["result"]

    listener = Client()
    clients.append(listener)
    listener.subscribe("/joint_states", "sensor_msgs/JointState")
    assert caller.call("/arm_sim/pause", {"data": False})["result"]
    observed = None
    while observed is None or timestamp(observed) < 0.5:
        observed = listener.next_publication("/joint_states")

    steps = round(timestamp(observed) / dt)
    q = [0.0] * links
    qdot = [0.0] * links
    for _ in range(steps):
        effort = [gains[index] * (target[index] - q[index])
                  for index in range(links)]
        acceleration = reference_acceleration(
            q, qdot, masses, lengths, gravity, effort
        )
        q = [q[index] + qdot[index] * dt for index in range(links)]
        qdot = [qdot[index] + acceleration[index] * dt for index in range(links)]

    position_error = max(abs(q[index] - observed["position"][index])
                         for index in range(links))
    velocity_error = max(abs(qdot[index] - observed["velocity"][index])
                         for index in range(links))
    assert position_error < 2e-8, (q, observed["position"], position_error)
    assert velocity_error < 2e-8, (qdot, observed["velocity"], velocity_error)
    assert caller.call("/pid_controller/enable", {"data": False})["result"]
    return position_error, velocity_error


def audit_default_pid(caller, links, clients, default_gains):
    target = [0.5, -0.35, 0.25][:links]
    method = os.environ.get("PID_TEST_METHOD", "rk4")
    timestep = float(os.environ.get("PID_TEST_TIMESTEP", "0.01"))
    assert caller.call("/arm_sim/pause", {"data": True})["result"]
    assert caller.call("/arm_sim/set_integrator",
                       {"method": method, "timestep": timestep})["result"]
    assert caller.call("/arm_sim/set_params", {
        "gravity": 9.81,
        "masses": [1.0] * links,
        "lengths": [1.0] * links,
    })["result"]
    assert caller.call("/arm_sim/reset", {})["result"]
    assert caller.call("/pid_controller/set_gains", default_gains)["result"]
    caller.publish("/joint_trajectory", {
        "points": [{"positions": target}]
    })
    assert caller.call("/pid_controller/enable", {"data": True})["result"]
    listener = Client()
    clients.append(listener)
    listener.subscribe("/joint_states", "sensor_msgs/JointState")
    assert caller.call("/arm_sim/pause", {"data": False})["result"]

    last = None
    while last is None or timestamp(last) < 8.0:
        last = listener.next_publication("/joint_states", timeout=2.0)
        check_joint_state(last, links)
    errors = [abs(target[index] - last["position"][index])
              for index in range(links)]
    assert max(errors) < 0.08, (target, last, errors, default_gains)
    assert max(abs(value) for value in last["velocity"]) < 0.20, last
    return max(errors), max(abs(value) for value in last["velocity"])


def audit(links):
    process = start_runtime(links)
    clients = []
    try:
        caller = Client()
        clients.append(caller)
        wait_for_service(caller, "/arm_sim/set_params")
        default_gains = caller.call("/pid_controller/set_gains", {})["values"]
        candidate = os.environ.get("PID_TEST_GAINS")
        if candidate:
            gain_groups = candidate.split(",")
            if len(gain_groups) != 3:
                raise ValueError(
                    "PID_TEST_GAINS must be kp,ki,kd; each value may be a "
                    "slash-separated per-joint list"
                )

            def parse_gain_group(group):
                values = [float(value) for value in group.split("/")]
                if len(values) == 1:
                    values *= links
                if len(values) != links:
                    raise ValueError(
                        f"expected 1 or {links} gains in {group!r}"
                    )
                return values

            default_gains = {
                "kp": parse_gain_group(gain_groups[0]),
                "ki": parse_gain_group(gain_groups[1]),
                "kd": parse_gain_group(gain_groups[2]),
            }

        median_latency, maximum_latency = audit_service_latency(caller, links, clients)
        audit_velocity_pid(caller, links, clients)
        position_error, velocity_error = audit_dynamics(caller, links, clients)
        convergence_error, convergence_velocity = audit_default_pid(
            caller, links, clients, default_gains
        )
        assert process.poll() is None
        print(
            f"PASS: {links}-link latency median={median_latency * 1000:.1f}ms "
            f"max={maximum_latency * 1000:.1f}ms; dynamics errors "
            f"q={position_error:.2e}, qdot={velocity_error:.2e}; default PID "
            f"position={convergence_error:.4f}, velocity={convergence_velocity:.4f}"
        )
    finally:
        stop_runtime(process, clients)


def main():
    selected = os.environ.get("PID_TEST_LINKS")
    link_counts = (int(selected),) if selected else (2, 3)
    for links in link_counts:
        audit(links)
    print("PASS: service latency, dynamics, and default PID")


if __name__ == "__main__":
    main()
