#!/usr/bin/env bash
set -euo pipefail

# -------- Configuration --------
CORE_SERVER="${CORE_SERVER:-6}"   # CPU core for run_server
CORE_CLIENT="${CORE_CLIENT:-7}"   # CPU core for franka_panda_client
PORT="${PORT:-50051}"             # Polymetis server port (for context/logging)

# Start command (you can append Hydra overrides when calling the script)
ROBOT_CMD=(launch_robot.py robot_client=franka_hardware)

# -------- Helpers --------
log() { printf "[pin] %s\n" "$*" >&2; }

get_pid_by_name() {
  # $1 = regex/pattern for pgrep -f
  # $2 = 1 => use sudo (needed for root-owned processes), 0 => no sudo
  local pattern="$1"
  local sudo_mode="${2:-0}"
  if [[ "$sudo_mode" == "1" ]]; then
    sudo pgrep -n -f "$pattern" 2>/dev/null || true
  else
    pgrep -n -f "$pattern" 2>/dev/null || true
  fi
}

pin_all_threads_quiet() {
  # Pins ALL threads (TIDs) of PID to a single core.
  # Suppresses taskset output to keep logs readable.
  # $1 = pid, $2 = core, $3 = 1 => sudo taskset, 0 => taskset
  local pid="$1"
  local core="$2"
  local use_sudo="${3:-0}"

  local tids=()
  if [[ -d "/proc/${pid}/task" ]]; then
    mapfile -t tids < <(ls -1 "/proc/${pid}/task" 2>/dev/null | sort -n)
  fi
  [[ "${#tids[@]}" -eq 0 ]] && return 1

  if [[ "$use_sudo" == "1" ]]; then
    for tid in "${tids[@]}"; do
      sudo taskset -pc "${core}" "${tid}" >/dev/null 2>&1 || true
    done
  else
    for tid in "${tids[@]}"; do
      taskset -pc "${core}" "${tid}" >/dev/null 2>&1 || true
    done
  fi

  echo "${#tids[@]}"
}

# -------- Main --------
main() {
  # Prime sudo once to avoid blocking later
  sudo -v

  log "Starting Polymetis (franka_hardware)..."
  "${ROBOT_CMD[@]}" "$@" &
  LAUNCH_PID=$!

  local server_pid="" client_pid=""
  log "Waiting for run_server + franka_panda_client PIDs (port ${PORT})..."

  for i in {1..300}; do  # ~30s total
    # run_server often runs with sudo/root in Polymetis -> use sudo pgrep
    server_pid="$(get_pid_by_name '(^|/)(run_server)([[:space:]]|$)' 1)"
    client_pid="$(get_pid_by_name '(^|/)(franka_panda_client)([[:space:]]|$)' 0)"

    if [[ -n "$server_pid" && -n "$client_pid" ]]; then
      break
    fi

    if (( i % 10 == 0 )); then
      log "still waiting... run_server='${server_pid:-}' franka_panda_client='${client_pid:-}'"
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      log "ERROR: launch_robot.py exited early."
      exit 1
    fi
    sleep 0.1
  done

  if [[ -z "$server_pid" || -z "$client_pid" ]]; then
    log "ERROR: Could not find both PIDs."
    log "Debug: sudo pgrep -fa run_server"
    sudo pgrep -fa run_server || true
    log "Debug: pgrep -fa franka_panda_client"
    pgrep -fa franka_panda_client || true
    wait "$LAUNCH_PID" || true
    exit 1
  fi

  printf "\n" >&2
  log "Pinning run_server PID=${server_pid} -> CPU ${CORE_SERVER}"
  local n_server n_client
  n_server="$(pin_all_threads_quiet "$server_pid" "$CORE_SERVER" 1 || echo 0)"

  log "Pinning franka_panda_client PID=${client_pid} -> CPU ${CORE_CLIENT}"
  n_client="$(pin_all_threads_quiet "$client_pid" "$CORE_CLIENT" 0 || echo 0)"

  printf "\n" >&2
  log "Pinned summary:"
  log "  run_server:          PID=${server_pid}  threads=${n_server}  allowed=$(sudo awk -F'\t' '/Cpus_allowed_list/ {print $2}' /proc/${server_pid}/status 2>/dev/null || echo '?')"
  log "  franka_panda_client: PID=${client_pid}  threads=${n_client}  allowed=$(awk -F'\t' '/Cpus_allowed_list/ {print $2}' /proc/${client_pid}/status 2>/dev/null || echo '?')"

  log "Thread placement snapshot (PSR can lag briefly after pinning):"
  sudo ps -L -o pid,tid,psr,rtprio,ni,comm -p "${server_pid}" | head -n 12 || true
  ps -L -o pid,tid,psr,rtprio,ni,comm -p "${client_pid}" | head -n 12 || true

  wait "$LAUNCH_PID"
}

main "$@"
