#!/usr/bin/env bash
set -uo pipefail

LOG_FILE="${I9_HOST_WATCHDOG_LOG:-/tmp/i9_host_watchdog.log}"
INTERVAL="${I9_HOST_WATCHDOG_INTERVAL:-2}"
HEARTBEAT_EVERY="${I9_HOST_WATCHDOG_HEARTBEAT_EVERY:-5}"
LOAD_WARN="${I9_HOST_WATCHDOG_LOAD_WARN:-8.0}"
GPU_TEMP_WARN_C="${I9_HOST_WATCHDOG_GPU_TEMP_WARN_C:-88}"
EVENT_COOLDOWN="${I9_HOST_WATCHDOG_EVENT_COOLDOWN:-10}"

last_event_ts=0
loop_count=0

ts() {
  date --iso-8601=seconds
}

wifi_summary() {
  if command -v iw >/dev/null 2>&1; then
    iw dev wlp3s0 link 2>/dev/null | sed 's/^[[:space:]]*//'
  elif [ -r /proc/net/wireless ]; then
    awk 'NR==1 || NR==2 || /wlp3s0:/' /proc/net/wireless
  else
    echo "wireless stats unavailable"
  fi
}

gpu_temp() {
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -n1 | tr -d ' '
  else
    echo "NA"
  fi
}

gpu_summary() {
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=utilization.gpu,temperature.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null
  else
    echo "nvidia-smi unavailable"
  fi
}

emit_event_snapshot() {
  local reason="$1"
  local now_ts
  now_ts="$(date +%s)"
  if [ $((now_ts - last_event_ts)) -lt "$EVENT_COOLDOWN" ]; then
    return
  fi
  last_event_ts="$now_ts"

  {
    echo "[$(ts)] EVENT $reason"
    echo "--- ps top ---"
    ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -20
    echo "--- gpu ---"
    gpu_summary
    echo "--- wifi ---"
    wifi_summary
    echo "--- ip wlp3s0 ---"
    ip -4 addr show wlp3s0 2>/dev/null || true
    echo "--- link stats ---"
    ip -s link show wlp3s0 2>/dev/null || true
    echo "--- udp sockets ---"
    ss -uapn 2>/dev/null | grep -E 'python3|rviz2|7400|7410|dds|cyclone' | head -60 || true
    echo "--- recent logs ---"
    for f in /tmp/i9_planner_campus.log /tmp/rviz_campus.log /tmp/i9_ros_watchdog.log; do
      [ -f "$f" ] || continue
      echo "== $f =="
      tail -n 40 "$f"
    done
    echo
  } >>"$LOG_FILE"
}

echo "[$(ts)] i9_host_watchdog start interval=${INTERVAL}s load_warn=${LOAD_WARN} gpu_temp_warn_c=${GPU_TEMP_WARN_C}" >>"$LOG_FILE"

while true; do
  loop_count=$((loop_count + 1))

  load1="$(awk '{print $1}' /proc/loadavg)"
  load5="$(awk '{print $2}' /proc/loadavg)"
  load15="$(awk '{print $3}' /proc/loadavg)"
  mem_total_mb="$(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo)"
  mem_avail_mb="$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)"
  gpu_temp_now="$(gpu_temp)"

  if [ $((loop_count % HEARTBEAT_EVERY)) -eq 1 ]; then
    {
      echo "[$(ts)] HEARTBEAT load=(${load1},${load5},${load15}) mem_avail_mb=${mem_avail_mb}/${mem_total_mb} gpu=$(gpu_summary | tr '\n' ';' | sed 's/;*$//')"
      echo "[$(ts)] WIFI $(wifi_summary | tr '\n' ';' | sed 's/;*$//')"
    } >>"$LOG_FILE"
  fi

  if awk -v x="$load1" -v y="$LOAD_WARN" 'BEGIN { exit !(x > y) }'; then
    emit_event_snapshot "load_spike load1=${load1}"
  fi

  if [ "$gpu_temp_now" != "NA" ] && awk -v x="$gpu_temp_now" -v y="$GPU_TEMP_WARN_C" 'BEGIN { exit !(x > y) }'; then
    emit_event_snapshot "gpu_temp_spike gpu_temp=${gpu_temp_now}"
  fi

  if ! pgrep -af 'i9_planner.py' >/dev/null 2>&1; then
    emit_event_snapshot "planner_missing"
  fi

  sleep "$INTERVAL"
done
