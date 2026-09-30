#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
P4_REPO="${P4_TRANSWING_REPO:-$(cd "$SCRIPT_DIR/../.." && pwd -P)}"
PORT="${P4_TRANSWING_CONTROL_SMOKE_PORT:-8876}"
LOG_FILE="${TMPDIR:-/tmp}/p4_control_station_smoke_$$.log"
server_pid=""

cleanup() {
  if [[ -n "$server_pid" ]] && kill -0 "$server_pid" 2>/dev/null; then
    # Background processes inherit ignored SIGINT from a non-interactive
    # shell, so use TERM and retain KILL only as a bounded fallback.
    kill -TERM "$server_pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      if ! kill -0 "$server_pid" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -KILL "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cd "$P4_REPO"
python3 -m Simulation.control_station.server \
  --repo "$P4_REPO" \
  --host 127.0.0.1 \
  --port "$PORT" >"$LOG_FILE" 2>&1 &
server_pid=$!

for _ in $(seq 1 80); do
  if python3 - "$PORT" <<'PY' >/dev/null 2>&1
import json
import sys
import urllib.request

port = int(sys.argv[1])
with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/health", timeout=1) as response:
    payload = json.load(response)
if payload.get("ok") is not True:
    raise SystemExit(1)
PY
  then
    echo "P4 Control Station health smoke test passed on port $PORT."
    exit 0
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "Control Station exited during startup." >&2
    tail -120 "$LOG_FILE" >&2
    exit 1
  fi
  sleep 0.2
done

echo "Control Station health endpoint did not become ready." >&2
tail -120 "$LOG_FILE" >&2
exit 1
