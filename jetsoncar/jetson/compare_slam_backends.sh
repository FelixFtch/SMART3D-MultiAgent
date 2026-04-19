#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTL="${ROOT_DIR}/jetson_runtime_ctl.sh"
REPORT_ROOT="${ROOT_DIR}/reports"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
OUT_DIR="${REPORT_ROOT}/slam_compare_${STAMP}"
DURATION_SEC="${DURATION_SEC:-60}"
REBOOT_BETWEEN_RUNS="${REBOOT_BETWEEN_RUNS:-1}"

mkdir -p "${OUT_DIR}"

log() {
  printf '[slam-compare] %s\n' "$*"
}

wait_after_reboot() {
  bash "${CTL}" wait 240
}

run_backend() {
  local backend="$1"
  local health_json="${OUT_DIR}/${backend}_healthcheck.json"
  local metrics_json="${OUT_DIR}/${backend}_metrics.json"
  local start_log="${OUT_DIR}/${backend}_start.log"

  if [ "${REBOOT_BETWEEN_RUNS}" = "1" ]; then
    bash "${CTL}" reboot || true
    wait_after_reboot
  fi

  bash "${CTL}" stop || true
  sleep 5

  case "${backend}" in
    cartographer)
      log "Starting cartographer benchmark profile"
      START_ODOM=0 \
      START_ACTIVE=0 \
      START_WATCHDOG=1 \
      START_STP23L=1 \
      START_STP23L_BRAKE_GUARD=1 \
      SMART3D_CARTO_CONFIG_BASENAME=robotcar_2d.lua \
      bash "${CTL}" start-cartographer | tee "${start_log}"
      ;;
    slam_toolbox)
      log "Starting slam_toolbox benchmark profile"
      START_ODOM=0 \
      START_ODOM_STATIC_TF=1 \
      START_ACTIVE=0 \
      START_WATCHDOG=1 \
      START_STP23L=1 \
      START_STP23L_BRAKE_GUARD=1 \
      bash "${CTL}" start-slam | tee "${start_log}"
      ;;
    *)
      echo "unknown backend: ${backend}" >&2
      exit 1
      ;;
  esac

  sleep 10
  bash "${CTL}" healthcheck > "${health_json}" || true
  bash "${CTL}" benchmark-one "${backend}" "${DURATION_SEC}" "${metrics_json}" || true
  bash "${CTL}" stop || true
}

python3_compare() {
  python3 - "${OUT_DIR}" <<'PY'
import json
import pathlib
import sys

out_dir = pathlib.Path(sys.argv[1])
report_path = out_dir / "summary.md"
health = {}
metrics = {}

def load_json_with_banner(path):
    text = path.read_text()
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        raise ValueError(f"no JSON payload found in {path}")
    return json.loads(text[start : end + 1])

for backend in ("cartographer", "slam_toolbox"):
    health_path = out_dir / f"{backend}_healthcheck.json"
    metrics_path = out_dir / f"{backend}_metrics.json"
    if health_path.exists():
        health[backend] = load_json_with_banner(health_path)
    if metrics_path.exists():
        metrics[backend] = load_json_with_banner(metrics_path)

def group_value(backend, group, field, default="n/a"):
    try:
        return metrics[backend]["summary"][group][field]
    except Exception:
        return default

def topic_value(backend, topic_key, default="n/a"):
    try:
        return health[backend]["counts_6s"][topic_key]
    except Exception:
        return default

lines = []
lines.append(f"# Jetson SLAM Backend Compare")
lines.append("")
lines.append(f"Output dir: `{out_dir}`")
lines.append("")
lines.append("| Metric | cartographer | slam_toolbox |")
lines.append("| --- | ---: | ---: |")
for group in [
    "lidar",
    "imu",
    "cartographer",
    "slam_toolbox",
    "scan_normalizer",
    "stp23l",
    "stp23l_guard",
    "map_bridge",
    "pi5_client",
    "watchdog",
]:
    lines.append(
        f"| avg CPU {group} (%) | {group_value('cartographer', group, 'avg_cpu_percent')} | {group_value('slam_toolbox', group, 'avg_cpu_percent')} |"
    )
    lines.append(
        f"| max RSS {group} (MB) | {group_value('cartographer', group, 'max_rss_mb')} | {group_value('slam_toolbox', group, 'max_rss_mb')} |"
    )
for topic in ["scan", "imu", "front_range", "map", "map_campus", "pose", "image_raw", "camera_info"]:
    lines.append(
        f"| topic count {topic} (6s) | {topic_value('cartographer', topic)} | {topic_value('slam_toolbox', topic)} |"
    )

lines.append("")
lines.append("## Notes")
lines.append("")
for backend in ("cartographer", "slam_toolbox"):
    lines.append(f"### {backend}")
    if backend in health:
        lines.append(f"- counts_6s: `{health[backend].get('counts_6s', {})}`")
        tty = health[backend].get("tty_usb0_owner", [])
        if tty:
            lines.append("- /dev/ttyUSB0 owner:")
            for row in tty[:6]:
                lines.append(f"  - `{row}`")
    if backend in metrics:
        logs = metrics[backend].get("logs", {})
        for name, tail in logs.items():
            if tail:
                tail_text = "\\n".join(tail[-5:])
                lines.append(f"- {name} tail:")
                lines.append("```text")
                lines.append(tail_text)
                lines.append("```")
    lines.append("")

report_path.write_text("\n".join(lines))
print(report_path)
PY
}

log "Deploying latest Jetson runtime"
bash "${CTL}" deploy

run_backend cartographer
run_backend slam_toolbox

SUMMARY_PATH="$(python3_compare)"
log "Wrote summary to ${SUMMARY_PATH}"
