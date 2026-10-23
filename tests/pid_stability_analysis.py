#!/usr/bin/env python3
"""Linearize the implemented sampled PID/arm step and report stability poles.

This is a developer diagnostic, not part of the runtime.  It mirrors the
zero-order-held effort and numerical integration used by arm_sim so tuning can
be checked without putting calculations in the web UI.
"""

import argparse
import itertools
import math

import numpy as np

from test_dynamics_timeouts import reference_acceleration


def gravity_load(q, masses, lengths, gravity):
    orientations = np.cumsum(q)
    result = np.zeros(len(q))
    for link in range(len(q)):
        for joint in range(link + 1):
            dy_dq = sum(
                lengths[segment] * math.cos(orientations[segment])
                for segment in range(joint, link)
            )
            dy_dq += 0.5 * lengths[link] * math.cos(orientations[link])
            result[joint] += masses[link] * gravity * dy_dq
    return result


def plant_step(q, velocity, effort, dt, method, masses, lengths, gravity):
    acceleration = lambda position, speed: np.asarray(reference_acceleration(
        list(position), list(speed), masses, lengths, gravity, list(effort)
    ))
    a0 = acceleration(q, velocity)

    if method == "euler":
        return q + velocity * dt, velocity + a0 * dt
    if method == "midpoint":
        midpoint_q = q + velocity * dt / 2.0
        midpoint_v = velocity + a0 * dt / 2.0
        midpoint_a = acceleration(midpoint_q, midpoint_v)
        return q + midpoint_v * dt, velocity + midpoint_a * dt
    if method == "verlet":
        next_q = q + velocity * dt + a0 * dt * dt / 2.0
        predicted_v = velocity + a0 * dt
        next_a = acceleration(next_q, predicted_v)
        return next_q, velocity + (a0 + next_a) * dt / 2.0
    if method == "rk4":
        k1_q, k1_v = velocity, a0
        k2_q = velocity + k1_v * dt / 2.0
        k2_v = acceleration(q + k1_q * dt / 2.0,
                            velocity + k1_v * dt / 2.0)
        k3_q = velocity + k2_v * dt / 2.0
        k3_v = acceleration(q + k2_q * dt / 2.0,
                            velocity + k2_v * dt / 2.0)
        k4_q = velocity + k3_v * dt
        k4_v = acceleration(q + k3_q * dt, velocity + k3_v * dt)
        return (
            q + (k1_q + 2 * k2_q + 2 * k3_q + k4_q) * dt / 6.0,
            velocity + (k1_v + 2 * k2_v + 2 * k3_v + k4_v) * dt / 6.0,
        )
    raise ValueError(method)


def closed_loop_step(state, target, kp, ki, kd, dt, method,
                     masses, lengths, gravity):
    count = len(target)
    q = state[:count]
    velocity = state[count:2 * count]
    integral = state[2 * count:]
    error = target - q
    next_integral = integral + error * dt
    effort = kp * error + ki * next_integral - kd * velocity
    next_q, next_velocity = plant_step(
        q, velocity, effort, dt, method, masses, lengths, gravity
    )
    return np.concatenate((next_q, next_velocity, next_integral))


def linearized_poles(target, kp, ki, kd, dt, method, masses, lengths, gravity):
    count = len(target)
    if np.any(ki <= 0.0):
        raise ValueError("analysis requires positive Ki for an equilibrium")
    equilibrium = np.concatenate((
        target,
        np.zeros(count),
        gravity_load(target, masses, lengths, gravity) / ki,
    ))
    size = len(equilibrium)
    jacobian = np.zeros((size, size))
    perturbation = 1e-6
    for column in range(size):
        offset = np.zeros(size)
        offset[column] = perturbation
        positive = closed_loop_step(
            equilibrium + offset, target, kp, ki, kd, dt, method,
            masses, lengths, gravity
        )
        negative = closed_loop_step(
            equilibrium - offset, target, kp, ki, kd, dt, method,
            masses, lengths, gravity
        )
        jacobian[:, column] = (positive - negative) / (2.0 * perturbation)
    return np.linalg.eigvals(jacobian)


def parse_values(text, count):
    values = np.asarray([float(value) for value in text.split("/")])
    if len(values) == 1:
        values = np.repeat(values, count)
    if len(values) != count:
        raise argparse.ArgumentTypeError(f"expected 1 or {count} values")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--links", type=int, choices=(2, 3), default=3)
    parser.add_argument("--kp")
    parser.add_argument("--ki")
    parser.add_argument("--kd")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--method", choices=("euler", "midpoint", "verlet", "rk4"),
                        default="rk4")
    args = parser.parse_args()

    defaults = {
        2: ("100/50", "150/100", "15/10"),
        3: ("150/75/35", "40/25/12", "20/10/5"),
    }
    default_kp, default_ki, default_kd = defaults[args.links]
    kp = parse_values(args.kp or default_kp, args.links)
    ki = parse_values(args.ki or default_ki, args.links)
    kd = parse_values(args.kd or default_kd, args.links)
    masses = [1.0] * args.links
    lengths = [1.0] * args.links

    # Joint angles are periodic, so this coarse grid spans a full revolution
    # and includes horizontal, upright, and hanging configurations.
    samples = (-math.pi, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi)
    worst_radius = 0.0
    worst_target = None
    worst_poles = None
    for target_tuple in itertools.product(samples, repeat=args.links):
        target = np.asarray(target_tuple)
        poles = linearized_poles(
            target, kp, ki, kd, args.dt, args.method,
            masses, lengths, 9.81
        )
        radius = max(abs(poles))
        if radius > worst_radius:
            worst_radius = radius
            worst_target = target_tuple
            worst_poles = poles

    verdict = "stable" if worst_radius < 1.0 else "UNSTABLE"
    print(f"{args.links}-link {args.method} dt={args.dt:g}: {verdict}")
    print(f"worst sampled spectral radius: {worst_radius:.8f} at q={worst_target}")
    print("poles:", " ".join(f"{pole.real:+.6f}{pole.imag:+.6f}j" for pole in worst_poles))


if __name__ == "__main__":
    main()
