# Pendularm

A planar 2-link or 3-link arm simulation using the project’s TCP/JSON hub.
The runtime publishes `/joint_states`, accepts `/joint_trajectory` setpoints, and
provides the simulation and PID services implemented in `src/nodes/arm_sim.cpp`.

## Build and run

```bash
make build
ARM_SIM_LINKS=2 make run      # use 3 for a three-link runtime
```

`make run` keeps the hub and the `arm_sim`, standalone `integrator_step`, `ik`, `ik_action`, and `ik_trial`
nodes alive in the foreground. It listens on `127.0.0.1:9095`.

## Web control room

In a second terminal, with the runtime already running:

```bash
make web
```

Open <http://127.0.0.1:8080>. The web UI is dependency-free and local-only. It
subscribes to `/joint_states` and draws the latest published arm state; it does
not simulate the arm, run an integrator, compute PID, or perform IK.

The UI displays the required `/joint_states` message unchanged and displays the
node-published `/arm_sim/energy` topic separately. Its right-side controls call
`/arm_sim/set_params`, `/ik_action/send_goal`, `/pid_controller/enable`,
`/pid_controller/set_gains`, and `/ik_trial/start`, `/ik_trial/skip`, and
`/ik_trial/stop`.

The browser cannot open a raw TCP socket to the hub, so `web/server.mjs` is a
small local WebSocket-to-TCP relay. It forwards JSON unchanged between the
browser and `127.0.0.1:9095`; it contains no simulation logic.

## Build systems

Both build systems now include the same runtime nodes:

```bash
make build
cmake -S . -B cmake-build
cmake --build cmake-build
```

`make clean` removes only the generated `build` directory.
