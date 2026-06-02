#!/usr/bin/env bash
set -euo pipefail

# Start the remote stereo teleoperation stack:
#   Janus VideoRoom + scene-relay + Fast-FoundationStereo + robot_server.py
#
# Usage:
#   ./start_stereo_teleop.sh
#   ./start_stereo_teleop.sh status
#   ./start_stereo_teleop.sh stop
#
# Common overrides:
#   JANUS_ROOM_LABEL=robot-scene FFS_VIDEO_DEVICE=0 ./start_stereo_teleop.sh
#   ROBOT_MOCK=1 ./start_stereo_teleop.sh
#   FFS_CMD='custom command here' ./start_stereo_teleop.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="${ROBOT_DIR:-$SCRIPT_DIR}"
MKROBOT_DIR="$(dirname "$ROBOT_DIR")"
HOME_DIR="${HOME:-/home/jr}"
FFS_DIR="${FFS_DIR:-$HOME_DIR/Fast-FoundationStereo}"
SPATIAL_DIR="${SPATIAL_DIR:-$HOME_DIR/StereoSpatial/SpatialCanvas}"

RUN_DIR="${RUN_DIR:-$ROBOT_DIR/.teleop}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
mkdir -p "$RUN_DIR" "$LOG_DIR"

JANUS_HTTP_URL="${JANUS_HTTP_URL:-http://127.0.0.1:8088/janus}"
JANUS_WS_URL="${JANUS_WS_URL:-ws://127.0.0.1:8188}"
JANUS_ROOM_LABEL="${JANUS_ROOM_LABEL:-robot-scene}"
JANUS_ROOM_SECRET="${JANUS_ROOM_SECRET:-adminpwd}"
JANUS_ROOM_DESCRIPTION="${JANUS_ROOM_DESCRIPTION:-Stereo Teleop Room}"
JANUS_PUBLISHERS="${JANUS_PUBLISHERS:-6}"
JANUS_BITRATE="${JANUS_BITRATE:-2048000}"
JANUS_FIR_FREQ="${JANUS_FIR_FREQ:-10}"
JANUS_VIDEOCODEC="${JANUS_VIDEOCODEC:-vp8,h264}"
JANUS_AUDIOCODEC="${JANUS_AUDIOCODEC:-opus}"

SCENE_RELAY_HOST="${SCENE_RELAY_HOST:-0.0.0.0}"
SCENE_RELAY_PORT="${SCENE_RELAY_PORT:-8190}"
SCENE_RELAY_URL="${SCENE_RELAY_URL:-ws://127.0.0.1:$SCENE_RELAY_PORT}"
SCENE_RELAY_ROOM="${SCENE_RELAY_ROOM:-robot-scene}"

FFS_CONDA_ENV="${FFS_CONDA_ENV:-ffs}"
FFS_CAMERA="${FFS_CAMERA:-decxin}"
FFS_VIDEO_DEVICE="${FFS_VIDEO_DEVICE:-0}"
FFS_DETECT_SOURCE="${FFS_DETECT_SOURCE:-left_ir}"
FFS_CALIBRATION_PATH="${FFS_CALIBRATION_PATH:-calibration/decxin/stereo_calib_120mm.npz}"
FFS_HOST="${FFS_HOST:-0.0.0.0}"
FFS_PORT="${FFS_PORT:-8765}"
FFS_SCENE_REFRESH_INTERVAL="${FFS_SCENE_REFRESH_INTERVAL:-2}"
FFS_SCENE_MAX_DEPTH="${FFS_SCENE_MAX_DEPTH:-8}"
FFS_EXTRA_ARGS="${FFS_EXTRA_ARGS:-}"

ROBOT_CONFIG="${ROBOT_CONFIG:-$ROBOT_DIR/robot_server_config.json}"
ROBOT_PYTHON="${ROBOT_PYTHON:-}"
ROBOT_MOCK="${ROBOT_MOCK:-0}"

ACTION="${1:-start}"

info() { printf '[teleop] %s\n' "$*"; }
warn() { printf '[teleop] WARN: %s\n' "$*" >&2; }
fail() { printf '[teleop] ERROR: %s\n' "$*" >&2; exit 1; }

pid_alive() {
  local pid_file="$1"
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

port_listening() {
  local port="$1"
  ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq "[:.]$port$"
}

wait_for_port() {
  local port="$1"
  local label="$2"
  local deadline=$((SECONDS + 20))
  until port_listening "$port"; do
    if (( SECONDS >= deadline )); then
      fail "$label did not open port $port"
    fi
    sleep 0.4
  done
}

wait_for_janus_http() {
  local deadline=$((SECONDS + 25))
  until python3 - "$JANUS_HTTP_URL" >/dev/null 2>&1 <<'PY'
import json
import sys
import urllib.request

url = sys.argv[1]
data = json.dumps({"janus": "info", "transaction": "teleop-info"}).encode()
req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
with urllib.request.urlopen(req, timeout=2) as resp:
    body = json.loads(resp.read().decode())
if body.get("janus") not in ("server_info", "success"):
    raise SystemExit(1)
PY
  do
    if (( SECONDS >= deadline )); then
      fail "Janus HTTP API is not ready at $JANUS_HTTP_URL"
    fi
    sleep 0.5
  done
}

derive_janus_room_number() {
  python3 - "$JANUS_ROOM_LABEL" <<'PY'
import sys
room_id = sys.argv[1]
h = 0
for ch in room_id:
    h = (h * 31 + ord(ch)) & 0xFFFFFFFF
print(100000 + (h % 900000))
PY
}

choose_robot_python() {
  if [[ -n "$ROBOT_PYTHON" ]]; then
    printf '%s\n' "$ROBOT_PYTHON"
    return
  fi
  for candidate in "$HOME_DIR/miniconda3/bin/python" "$(command -v python3 || true)"; do
    [[ -n "$candidate" && -x "$candidate" ]] || continue
    if "$candidate" - <<'PY' >/dev/null 2>&1
import websockets
PY
    then
      printf '%s\n' "$candidate"
      return
    fi
  done
  printf '%s\n' "$(command -v python3 || echo python3)"
}

check_robot_python_modules() {
  local py="$1"
  if [[ "$ROBOT_MOCK" == "1" ]]; then
    "$py" - <<'PY' || fail "robot_server --mock needs Python module: websockets"
import websockets
PY
    return
  fi
  "$py" - <<'PY' || fail "robot_server real mode needs Python modules: websockets and pyserial. Install them in ROBOT_PYTHON or run ROBOT_MOCK=1 for protocol testing."
import websockets
import serial
PY
}

ensure_janus() {
  if port_listening 8088 && port_listening 8188; then
    info "Janus already listening on 8088/8188"
    return
  fi

  info "Starting Janus"
  if command -v systemctl >/dev/null 2>&1; then
    sudo -n systemctl start janus 2>/dev/null || systemctl --user start janus 2>/dev/null || true
  fi

  if ! port_listening 8088 || ! port_listening 8188; then
    if pid_alive "$RUN_DIR/janus.pid"; then
      info "Janus foreground process already running pid=$(cat "$RUN_DIR/janus.pid")"
    else
      nohup janus -F /etc/janus >"$LOG_DIR/janus.log" 2>&1 &
      echo $! >"$RUN_DIR/janus.pid"
      info "Janus started in foreground pid=$(cat "$RUN_DIR/janus.pid") log=$LOG_DIR/janus.log"
    fi
  fi

  wait_for_port 8088 "Janus HTTP"
  wait_for_port 8188 "Janus WebSocket"
  wait_for_janus_http
}

ensure_janus_room() {
  local room_number
  room_number="$(derive_janus_room_number)"
  info "Ensuring Janus VideoRoom label=$JANUS_ROOM_LABEL number=$room_number"
  python3 - "$JANUS_HTTP_URL" "$room_number" "$JANUS_ROOM_SECRET" "$JANUS_ROOM_DESCRIPTION" "$JANUS_PUBLISHERS" "$JANUS_BITRATE" "$JANUS_FIR_FREQ" "$JANUS_VIDEOCODEC" "$JANUS_AUDIOCODEC" <<'PY'
import json
import sys
import time
import urllib.request

base_url, room, secret, description, publishers, bitrate, fir_freq, videocodec, audiocodec = sys.argv[1:]
room = int(room)

def post(url, payload):
    payload = dict(payload)
    payload.setdefault("transaction", f"teleop-{int(time.time() * 1000)}")
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode())

session = post(base_url, {"janus": "create"})["data"]["id"]
try:
    handle = post(f"{base_url}/{session}", {"janus": "attach", "plugin": "janus.plugin.videoroom"})["data"]["id"]
    body = {
        "request": "create",
        "room": room,
        "description": description,
        "secret": secret,
        "publishers": int(publishers),
        "bitrate": int(bitrate),
        "fir_freq": int(fir_freq),
        "audiocodec": audiocodec,
        "videocodec": videocodec,
        "record": False,
        "permanent": False,
    }
    response = post(f"{base_url}/{session}/{handle}", {"janus": "message", "body": body})
    plugin = response.get("plugindata", {}).get("data", {})
    if plugin.get("videoroom") == "created":
        print(f"created room {room}")
    elif plugin.get("error_code") in (427, 486) or "already" in str(plugin.get("error", "")).lower():
        print(f"room {room} already exists")
    elif plugin.get("videoroom") == "event":
        print(json.dumps(plugin))
    else:
        raise SystemExit(f"unexpected VideoRoom response: {json.dumps(response)}")
finally:
    try:
        post(f"{base_url}/{session}", {"janus": "destroy"})
    except Exception:
        pass
PY
}

ensure_scene_relay() {
  if port_listening "$SCENE_RELAY_PORT"; then
    info "scene-relay already listening on $SCENE_RELAY_PORT"
    return
  fi
  [[ -f "$SPATIAL_DIR/scene-relay.mjs" ]] || fail "scene-relay.mjs not found at $SPATIAL_DIR"
  info "Starting scene-relay on :$SCENE_RELAY_PORT"
  (cd "$SPATIAL_DIR" && nohup node scene-relay.mjs "$SCENE_RELAY_PORT" >"$LOG_DIR/scene-relay.log" 2>&1 & echo $! >"$RUN_DIR/scene-relay.pid")
  wait_for_port "$SCENE_RELAY_PORT" "scene-relay"
}

start_ffs() {
  if port_listening "$FFS_PORT"; then
    info "FFS perception service already listening on $FFS_PORT"
    return
  fi
  [[ -d "$FFS_DIR" ]] || fail "Fast-FoundationStereo not found at $FFS_DIR"

  local cmd
  if [[ -n "${FFS_CMD:-}" ]]; then
    cmd="$FFS_CMD"
  else
    cmd="source \"$HOME_DIR/miniconda3/etc/profile.d/conda.sh\" && conda run -n \"$FFS_CONDA_ENV\" python scripts/run_perception_service.py \
      --camera \"$FFS_CAMERA\" \
      --video-device \"$FFS_VIDEO_DEVICE\" \
      --detect-source \"$FFS_DETECT_SOURCE\" \
      --calibration-path \"$FFS_CALIBRATION_PATH\" \
      --transport websocket \
      --host \"$FFS_HOST\" \
      --port \"$FFS_PORT\" \
      --relay-url \"$SCENE_RELAY_URL\" \
      --scene-refresh-interval \"$FFS_SCENE_REFRESH_INTERVAL\" \
      --scene-max-depth \"$FFS_SCENE_MAX_DEPTH\" \
      $FFS_EXTRA_ARGS"
  fi

  info "Starting FFS perception service log=$LOG_DIR/ffs.log"
  (cd "$FFS_DIR" && nohup bash -lc "$cmd" >"$LOG_DIR/ffs.log" 2>&1 & echo $! >"$RUN_DIR/ffs.pid")
  sleep 2
  if ! pid_alive "$RUN_DIR/ffs.pid" && ! port_listening "$FFS_PORT"; then
    tail -80 "$LOG_DIR/ffs.log" >&2 || true
    fail "FFS exited during startup"
  fi
}

start_robot_server() {
  if pid_alive "$RUN_DIR/robot_server.pid"; then
    info "robot_server already running pid=$(cat "$RUN_DIR/robot_server.pid")"
    return
  fi
  [[ -f "$ROBOT_DIR/robot_server.py" ]] || fail "robot_server.py not found at $ROBOT_DIR"
  [[ -f "$ROBOT_CONFIG" ]] || fail "robot config not found: $ROBOT_CONFIG"

  local py
  py="$(choose_robot_python)"
  check_robot_python_modules "$py"

  local mock_arg=()
  if [[ "$ROBOT_MOCK" == "1" ]]; then
    mock_arg=(--mock)
  fi

  info "Starting robot_server with $py log=$LOG_DIR/robot_server.log"
  (
    cd "$ROBOT_DIR"
    nohup "$py" robot_server.py --config "$ROBOT_CONFIG" --relay-url "$SCENE_RELAY_URL" --room "$SCENE_RELAY_ROOM" "${mock_arg[@]}" >"$LOG_DIR/robot_server.log" 2>&1 &
    echo $! >"$RUN_DIR/robot_server.pid"
  )
  sleep 1
  if ! pid_alive "$RUN_DIR/robot_server.pid"; then
    tail -80 "$LOG_DIR/robot_server.log" >&2 || true
    fail "robot_server exited during startup"
  fi
}

show_status() {
  local room_number
  room_number="$(derive_janus_room_number)"
  info "Status"
  printf '  Janus HTTP:       %s (%s)\n' "$JANUS_HTTP_URL" "$(port_listening 8088 && echo up || echo down)"
  printf '  Janus WS:         %s (%s)\n' "$JANUS_WS_URL" "$(port_listening 8188 && echo up || echo down)"
  printf '  Janus room:       %s -> %s\n' "$JANUS_ROOM_LABEL" "$room_number"
  printf '  scene-relay:      %s (%s)\n' "$SCENE_RELAY_URL" "$(port_listening "$SCENE_RELAY_PORT" && echo up || echo down)"
  printf '  FFS WS:           ws://0.0.0.0:%s (%s)\n' "$FFS_PORT" "$(port_listening "$FFS_PORT" && echo up || echo down)"
  printf '  robot_server pid: %s\n' "$(pid_alive "$RUN_DIR/robot_server.pid" && cat "$RUN_DIR/robot_server.pid" || echo down)"
  printf '  logs:             %s\n' "$LOG_DIR"
}

stop_pid_file() {
  local name="$1"
  local pid_file="$2"
  if pid_alive "$pid_file"; then
    local pid
    pid="$(cat "$pid_file")"
    info "Stopping $name pid=$pid"
    kill "$pid" 2>/dev/null || true
    sleep 0.5
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

stop_stack() {
  stop_pid_file robot_server "$RUN_DIR/robot_server.pid"
  stop_pid_file ffs "$RUN_DIR/ffs.pid"
  stop_pid_file scene-relay "$RUN_DIR/scene-relay.pid"
  stop_pid_file janus "$RUN_DIR/janus.pid"
  info "Stopped processes tracked by $RUN_DIR"
}

start_stack() {
  ensure_janus
  ensure_janus_room
  ensure_scene_relay
  start_ffs
  start_robot_server
  show_status
  cat <<EOF

Operator page:
  http://<operator-ui-host>:5174/#leader-arm?bridge=ws://<operator-pc-ip>:7000

Remote endpoints:
  Janus HTTP:  $JANUS_HTTP_URL
  Janus WS:    $JANUS_WS_URL
  Scene Relay: ws://<remote-ip>:$SCENE_RELAY_PORT
  FFS WS:      ws://<remote-ip>:$FFS_PORT
EOF
}

case "$ACTION" in
  start|up)
    start_stack
    ;;
  status)
    show_status
    ;;
  stop|down)
    stop_stack
    ;;
  restart)
    stop_stack
    start_stack
    ;;
  *)
    fail "Usage: $0 {start|status|stop|restart}"
    ;;
esac
