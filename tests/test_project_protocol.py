#!/usr/bin/env python3
"""Black-box contract and resilience checks for the complete PendulArm runtime."""

import itertools
import json
import math
import os
import signal
import socket
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
ADDRESS = ("127.0.0.1", 9095)
IDS = itertools.count(1)


class Client:
    def __init__(self, timeout=2.0):
        self.socket = socket.create_connection(ADDRESS, timeout=timeout)
        self.socket.settimeout(timeout)
        self.buffer = b""

    def close(self):
        self.socket.close()

    def send(self, message):
        payload = json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n"
        self.socket.sendall(payload.encode())

    def receive(self, timeout=None):
        if timeout is not None:
            self.socket.settimeout(timeout)
        while b"\n" not in self.buffer:
            chunk = self.socket.recv(65536)
            if not chunk:
                raise RuntimeError("gateway closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def call(self, service, args, timeout=2.0):
        request_id = f"audit-{next(IDS)}"
        self.send({
            "op": "call_service",
            "service": service,
            "id": request_id,
            "args": args,
        })
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{service} did not respond")
            response = self.receive(remaining)
            if response.get("op") == "service_response" and response.get("id") == request_id:
                validate_envelope(response, service)
                return response

    def subscribe(self, topic, message_type):
        self.send({"op": "subscribe", "topic": topic, "type": message_type,
                   "id": f"sub-{next(IDS)}"})

    def publish(self, topic, message):
        self.send({"op": "publish", "topic": topic, "msg": message})

    def next_publication(self, topic, timeout=2.0):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no publication received on {topic}")
            message = self.receive(remaining)
            if message.get("op") == "publish" and message.get("topic") == topic:
                return message["msg"]


def validate_envelope(response, service):
    assert response.get("service") == service, response
    assert isinstance(response.get("result"), bool), response
    assert isinstance(response.get("status"), str), response
    assert isinstance(response.get("values"), dict), response
    if not response["result"]:
        assert response["status"], f"{service} rejected without a status: {response}"


def finite_array(value, length):
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def assert_rejected(response):
    assert response["result"] is False, response
    assert response["status"], response


def wait_for_runtime():
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            probe = socket.create_connection(ADDRESS, timeout=0.1)
            probe.close()
            return
        except OSError:
            time.sleep(0.03)
    raise RuntimeError("runtime hub did not start")


def wait_for_service(client, service):
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            response = client.call(service, {}, timeout=0.5)
            if response["result"]:
                return response
        except (OSError, TimeoutError):
            pass
        time.sleep(0.03)
    raise RuntimeError(f"{service} did not become available")


def check_joint_state(message, links):
    assert set(message) == {"header", "name", "position", "velocity", "effort"}, message
    assert message["name"] == [f"joint{i + 1}" for i in range(links)]
    assert finite_array(message["position"], links)
    assert finite_array(message["velocity"], links)
    assert finite_array(message["effort"], links)
    stamp = message["header"]["stamp"]
    assert set(stamp) == {"sec", "nanosec"}
    assert isinstance(message["header"]["frame_id"], str)
    assert 0 <= stamp["nanosec"] < 1_000_000_000


def timestamp(message):
    stamp = message["header"]["stamp"]
    return stamp["sec"] + stamp["nanosec"] * 1e-9


def audit_runtime(links):
    env = os.environ.copy()
    env["ARM_SIM_LINKS"] = str(links)
    runtime = subprocess.Popen(
        [str(ROOT / "scripts/run.sh")],
        cwd=ROOT,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    clients = []
    try:
        wait_for_runtime()
        caller = Client()
        listener = Client()
        publisher = Client()
        clients += [caller, listener, publisher]

        params = wait_for_service(caller, "/arm_sim/set_params")
        assert finite_array(params["values"]["masses"], links)
        assert finite_array(params["values"]["lengths"], links)

        listener.subscribe("/joint_states", "sensor_msgs/JointState")
        listener.subscribe("/arm_sim/energy", "std_msgs/Float64")
        state = listener.next_publication("/joint_states")
        check_joint_state(state, links)
        energy = listener.next_publication("/arm_sim/energy")
        assert math.isfinite(energy["data"]) and energy["data"] >= 0.0

        # Optional simulation fields query current state and apply independently.
        current = caller.call("/arm_sim/set_integrator", {})
        assert current["result"] and set(current["values"]) == {"method", "timestep"}
        changed = caller.call("/arm_sim/set_integrator",
                              {"method": "rk4", "timestep": 0.005})
        assert changed["result"] and changed["values"] == {"method": "rk4", "timestep": 0.005}
        partial = caller.call("/arm_sim/set_integrator",
                              {"method": "midpoint", "timestep": "bad"})
        assert_rejected(partial)
        assert partial["values"] == {"method": "midpoint", "timestep": 0.005}
        for bad in (
            {"method": 1},
            {"method": "bogus"},
            {"timestep": 0},
            {"timestep": -0.1},
            {"timestep": "fast"},
        ):
            assert_rejected(caller.call("/arm_sim/set_integrator", bad))
        assert caller.call("/arm_sim/set_integrator", {})["result"]

        queried = caller.call("/arm_sim/set_params", {})
        assert queried["result"]
        partial = caller.call("/arm_sim/set_params", {
            "gravity": -1,
            "masses": [1.25] * links,
            "lengths": [1.0] * links,
        })
        assert_rejected(partial)
        assert partial["values"]["masses"] == [1.25] * links
        for bad in (
            {"gravity": "earth"},
            {"masses": [1.0] * (links - 1)},
            {"masses": [1.0] * (links - 1) + [0.0]},
            {"lengths": "long"},
            {"lengths": [1.0] * (links - 1) + [-1.0]},
        ):
            response = caller.call("/arm_sim/set_params", bad)
            assert_rejected(response)
            assert set(response["values"]) == {"gravity", "masses", "lengths"}
        restored = caller.call("/arm_sim/set_params", {
            "gravity": 9.81,
            "masses": [1.0] * links,
            "lengths": [1.0] * links,
        })
        assert restored["result"]

        # With PID off, RK4 should preserve mechanical energy for both dynamics
        # models. This catches finite-but-incorrect mass/Coriolis/gravity terms.
        assert caller.call("/arm_sim/set_integrator",
                           {"method": "rk4", "timestep": 0.002})["result"]
        assert caller.call("/arm_sim/reset", {})["result"]
        physics_listener = Client()
        clients.append(physics_listener)
        physics_listener.subscribe("/arm_sim/energy", "std_msgs/Float64")
        samples = []
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            value = physics_listener.next_publication("/arm_sim/energy", timeout=1.0)["data"]
            assert isinstance(value, (int, float)) and math.isfinite(value)
            samples.append(value)
        assert len(samples) >= 100
        energy_span = max(samples) - min(samples)
        assert energy_span <= max(1e-3, abs(samples[0]) * 5e-4), {
            "links": links,
            "minimum": min(samples),
            "maximum": max(samples),
            "span": energy_span,
        }

        pause_query = caller.call("/arm_sim/pause", {})
        assert pause_query["result"] and isinstance(pause_query["values"]["data"], bool)
        pause_bad = caller.call("/arm_sim/pause", {"data": 1})
        assert_rejected(pause_bad)
        paused = caller.call("/arm_sim/pause", {"data": True})
        assert paused["result"] and paused["values"] == {"data": True}
        pause_listener = Client()
        clients.append(pause_listener)
        pause_listener.subscribe("/joint_states", "sensor_msgs/JointState")
        previous_stamp = None
        frozen = False
        for _ in range(5):
            current_stamp = timestamp(pause_listener.next_publication("/joint_states"))
            if current_stamp == previous_stamp:
                frozen = True
                break
            previous_stamp = current_stamp
        assert frozen, "joint state time did not freeze while paused"
        resumed = caller.call("/arm_sim/pause", {"data": False})
        assert resumed["result"] and resumed["values"] == {"data": False}

        reset = caller.call("/arm_sim/reset", {"ignored": True})
        assert reset["result"]
        assert reset["values"] == {
            "position": [0.0] * links,
            "velocity": [0.0] * links,
        }

        pid_query = caller.call("/pid_controller/enable", {})
        assert pid_query["result"] and pid_query["values"] == {"data": False}
        assert_rejected(caller.call("/pid_controller/enable", {"data": "yes"}))
        gains = caller.call("/pid_controller/set_gains", {})
        assert gains["result"] and set(gains["values"]) == {"kp", "ki", "kd"}
        for key in ("kp", "ki", "kd"):
            assert finite_array(gains["values"][key], links)
        partial_gains = caller.call("/pid_controller/set_gains", {
            "kp": [8.0] * links,
            "ki": [-1.0] * links,
            "kd": [0.8] * links,
        })
        assert_rejected(partial_gains)
        assert partial_gains["values"]["kp"] == [8.0] * links
        assert partial_gains["values"]["kd"] == [0.8] * links
        assert partial_gains["values"]["ki"] != [-1.0] * links
        for bad in (
            {"kp": "high"},
            {"ki": [0.0] * (links - 1)},
            {"kd": [0.0] * (links - 1) + [-0.1]},
        ):
            assert_rejected(caller.call("/pid_controller/set_gains", bad))
        assert caller.call("/pid_controller/set_gains", {
            "kp": [8.0] * links,
            "ki": [0.0] * links,
            "kd": [0.8] * links,
        })["result"]

        # Malformed trajectory publications must be ignored without partial writes.
        for malformed in (
            {},
            {"points": "bad"},
            {"points": []},
            {"points": [{}]},
            {"points": [{"positions": "bad"}]},
            {"points": [{"positions": [0.1] * (links - 1)}]},
            {"points": [{"positions": [0.1] * (links - 1) + ["bad"]}]},
        ):
            publisher.publish("/joint_trajectory", malformed)
        assert caller.call("/arm_sim/set_params", {})["result"]

        # The standalone integration service must reject safely and stay responsive.
        base = {
            "function": "1",
            "x0": 0.0,
            "xdot0": 0.0,
            "dt": 0.1,
            "steps": 3,
            "integrator": "euler",
        }
        for method in ("euler", "midpoint", "verlet", "rk4"):
            request = dict(base, integrator=method)
            response = caller.call("/arm_sim/integration_step", request)
            assert response["result"], response
            assert set(response["values"]) == {"times", "positions", "velocities"}
            for key in response["values"]:
                assert finite_array(response["values"][key], 4)

        malformed_requests = [
            {},
            dict(base, function=1),
            dict(base, x0="zero"),
            dict(base, xdot0="zero"),
            dict(base, dt=0),
            dict(base, dt=-0.1),
            dict(base, steps=0),
            dict(base, steps=-1),
            dict(base, steps=1.5),
            dict(base, steps=1_000_001),
            dict(base, integrator="bad"),
            dict(base, function="not_a_function(t)"),
            dict(base, function="sqrt(-1)"),
            dict(base, function="1/0"),
        ]
        for request in malformed_requests:
            assert_rejected(caller.call("/arm_sim/integration_step", request))
        assert caller.call("/arm_sim/integration_step", base)["result"]

        # IK must reject invalid/unreachable targets and remain usable afterward.
        assert_rejected(caller.call("/ik/solve", {}))
        assert_rejected(caller.call("/ik/solve", {"x": "far", "y": 0}))
        assert_rejected(caller.call("/ik/solve", {"x": links + 1.0, "y": 0}))
        ik_request = {"x": 1.0, "y": 1.0}
        if links == 3:
            ik_request["phi"] = 0.0
        solved = caller.call("/ik/solve", ik_request)
        assert solved["result"] and finite_array(solved["values"]["positions"], links)

        assert_rejected(caller.call("/ik_action/send_goal", {}))
        assert_rejected(caller.call("/ik_action/send_goal",
                                    {"x": links + 1.0, "y": 0.0}))
        goal = caller.call("/ik_action/send_goal",
                           dict(ik_request, epsilon=0.05, success_hold=0.1))
        assert goal["result"] and isinstance(goal["values"]["goal_id"], str)
        goal_id = goal["values"]["goal_id"]
        assert_rejected(caller.call("/ik_action/cancel_goal", {"goal_id": "wrong"}))
        cancelled = caller.call("/ik_action/cancel_goal", {"goal_id": goal_id})
        assert cancelled["result"]

        assert_rejected(caller.call("/ik_trial/skip", {}))
        assert_rejected(caller.call("/ik_trial/stop", {}))
        assert_rejected(caller.call("/ik_trial/start", {"duration": 0}))

        # Exercise PID and dynamics long enough to catch freezes/non-finite output.
        assert caller.call("/arm_sim/set_integrator",
                           {"method": "rk4", "timestep": 0.005})["result"]
        assert caller.call("/arm_sim/reset", {})["result"]
        goal = caller.call("/ik_action/send_goal",
                           dict(ik_request, epsilon=0.05, success_hold=0.1))
        assert goal["result"]
        enabled = caller.call("/pid_controller/enable", {"data": True})
        assert enabled["result"] and enabled["values"] == {"data": True}
        deadline = time.monotonic() + 1.5
        observed = 0
        while time.monotonic() < deadline:
            state = listener.next_publication("/joint_states")
            check_joint_state(state, links)
            observed += 1
        assert observed >= 20
        assert caller.call("/arm_sim/set_params", {}, timeout=1.0)["result"]
        stopped = caller.call("/pid_controller/enable", {"data": False})
        assert stopped["result"] and stopped["values"] == {"data": False}

        # Even valid-but-extreme gains must not poison published JSON or hang
        # services. The simulator rolls back the bad step, clears unstable
        # state, and resumes PID on the next tick if it was previously enabled.
        assert caller.call("/pid_controller/set_gains", {
            "kp": [1e308] * links,
            "ki": [0.0] * links,
            "kd": [0.0] * links,
        })["result"]
        publisher.publish("/joint_trajectory", {
            "points": [{"positions": [1.0] * links}]
        })
        assert caller.call("/pid_controller/enable", {"data": True})["result"]
        stress_listener = Client()
        clients.append(stress_listener)
        stress_listener.subscribe("/joint_states", "sensor_msgs/JointState")
        for _ in range(20):
            check_joint_state(stress_listener.next_publication("/joint_states"), links)
        assert caller.call("/arm_sim/set_params", {}, timeout=1.0)["result"]
        recovered_pid = caller.call("/pid_controller/enable", {})
        assert recovered_pid["result"] and recovered_pid["values"] == {"data": True}
        assert caller.call("/pid_controller/set_gains", {
            "kp": [8.0] * links,
            "ki": [0.0] * links,
            "kd": [0.8] * links,
        })["result"]

        # Trial should continue issuing targets after reaching one, not stall at one.
        trial_listener = Client()
        clients.append(trial_listener)
        trial_listener.subscribe("/ik_trial/status", "ik_trial/status")
        trial = caller.call("/ik_trial/start", {
            "duration": 1.2,
            "epsilon": float(links * 2 + 1),
            "success_hold": 0.0,
        })
        assert trial["result"], trial
        max_reached = 0
        deadline = time.monotonic() + 2.5
        final_status = None
        while time.monotonic() < deadline:
            status = trial_listener.next_publication("/ik_trial/status", timeout=1.0)
            max_reached = max(max_reached, int(status["targets_reached"]))
            final_status = status
            if not status["running"]:
                break
        assert max_reached >= 2, final_status
        assert final_status is not None and final_status["running"] is False

        assert runtime.poll() is None, f"runtime exited with {runtime.returncode}"
        print(f"PASS: {links}-link services, topics, malformed inputs, PID, and IK trial")
    finally:
        for client in clients:
            try:
                client.close()
            except OSError:
                pass
        if runtime.poll() is None:
            os.killpg(runtime.pid, signal.SIGTERM)
        try:
            runtime.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(runtime.pid, signal.SIGKILL)
            runtime.wait(timeout=2)
        # Give the listening port a brief chance to leave the old process group.
        time.sleep(0.15)


def main():
    for links in (2, 3):
        audit_runtime(links)
    print("PASS: complete project protocol audit")


if __name__ == "__main__":
    main()
