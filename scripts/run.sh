#!/usr/bin/env bash
set -u

project_dir=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd "$project_dir"

pids=()
cleanup() {
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 0' INT TERM

./build/autorob_hub &
pids+=("$!")

./build/integrator_step &
pids+=("$!")

./build/arm_sim &
pids+=("$!")

./build/ik &
pids+=("$!")

./build/ik_action &
pids+=("$!")

./build/ik_trial &
pids+=("$!")

# The runtime remains in the foreground. If one required process exits, stop all.
wait -n "${pids[@]}"
status=$?
if ((status == 0)); then status=1; fi
exit "$status"
