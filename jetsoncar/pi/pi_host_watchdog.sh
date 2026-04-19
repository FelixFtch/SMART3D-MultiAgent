#!/usr/bin/env bash
set -uo pipefail

LOG_FILE="${PI_HOST_WATCHDOG_LOG:-/tmp/pi_host_watchdog.log}"
INTERVAL="${PI_HOST_WATCHDOG_INTERVAL:-2}"
HEARTBEAT_EVERY="${PI_HOST_WATCHDOG_HEARTBEAT_EVERY:-5}"
LOAD_WARN="${PI_HOST_WATCHDOG_LOAD_WARN:-4.0}"
TEMP_WARN_C="${PI_HOST_WATCHDOG_TEMP_WARN_C:-78.0}"
EVENT_COOLDOWN="${PI_HOST_WATCHDOG_EVENT_COOLDOWN:-10}"

last_event_ts=0
loop_count=0

ts() {
  date --iso-8601=seconds
}

temp_millic() {
  if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
    cat /sys/class/thermal/thermal_zone0/temp
  else
    echo 0
  fi
}

temp_c() {
  awk -v raw="$(temp_millic)" 'BEGIN { printf "%.1f", raw / 1000.0 }'
}

throttled_state() {
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled 2>/dev/null | awk -F= '{print $2}'
  else
    echo "NA"
  fi
}

wifi_summary() {
  if command -v iw >/dev/null 2>&1; then
    iw dev wlan0 link 2>/dev/null | sed 's/^[[:space:]]*//'
  elif [ -r /proc/net/wireless ]; then
    awk 'NR==1 || NR==2 || /wlan0:/' /proc/net/wireless
  else
    echo "wireless stats unavailable"
  fi
}

container_stats() {
  if command -v docker >/dev/null 2>&1; then
    docker stats --no-stream ros2-humble 2>/dev/null
  fi
}

container_top() {
  if command -v docker >/dev/null 2>&1; then
    docker exec ros2-humble bash -lc \
      'ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -20' 2>/dev/null
  fi
}

recent_kernel_signals() {
  local text=""
  if command -v journalctl >/dev/null 2>&1; then
    text="$(journalctl -k --since '20 seconds ago' --no-pager 2>/dev/null | grep -Ei 'usb|tty|wlan0|brcm|lslidar|xhci|disconnect|reset|overflow' || true)"
  else
    text="$(dmesg | tail -n 80 | grep -Ei 'usb|tty|wlan0|brcm|lslidar|xhci|disconnect|reset|overflow' || true)"
  fi
  printf '%s' "$text"
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
    echo "--- docker stats ---"
    container_stats
    echo "--- container top ---"
    container_top
    echo "--- wlan0 link ---"
    wifi_summary
    echo "--- ip wlan0 ---"
    ip -4 addr show wlan0 2>/dev/null || true
    echo "--- recent kernel ---"
    if command -v journalctl >/dev/null 2>&1; then
      journalctl -k -n 40 --no-pager 2>/dev/null || true
    else
      dmesg | tail -n 40 || true
    fi
    echo
  } >>"$LOG_FILE"
}

echo "[$(ts)] pi_host_watchdog start interval=${INTERVAL}s load_warn=${LOAD_WARN} temp_warn_c=${TEMP_WARN_C}" >>"$LOG_FILE"

while true; do
  loop_count=$((loop_count + 1))

  load1="$(awk '{print $1}' /proc/loadavg)"
  load5="$(awk '{print $2}' /proc/loadavg)"
  load15="$(awk '{print $3}' /proc/loadavg)"
  mem_total_mb="$(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo)"
  mem_avail_mb="$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)"
  temp_now_c="$(temp_c)"
  throttled="$(throttled_state)"

  if [ $((loop_count % HEARTBEAT_EVERY)) -eq 1 ]; then
    {
      echo "[$(ts)] HEARTBEAT load=(${load1},${load5},${load15}) mem_avail_mb=${mem_avail_mb}/${mem_total_mb} temp_c=${temp_now_c} throttled=${throttled}"
      echo "[$(ts)] WIFI $(wifi_summary | tr '\n' ';' | sed 's/;*$//')"
      kernel_snippet="$(recent_kernel_signals)"
      if [ -n "$kernel_snippet" ]; then
        echo "[$(ts)] KERNEL $(printf '%s' "$kernel_snippet" | tr '\n' ';' | sed 's/;*$//')"
      fi
    } >>"$LOG_FILE"
  fi

  if awk -v x="$load1" -v y="$LOAD_WARN" 'BEGIN { exit !(x > y) }'; then
    emit_event_snapshot "load_spike load1=${load1}"
  fi

  if awk -v x="$temp_now_c" -v y="$TEMP_WARN_C" 'BEGIN { exit !(x > y) }'; then
    emit_event_snapshot "temp_spike temp_c=${temp_now_c}"
  fi

  if [ "$throttled" != "NA" ] && [ "$throttled" != "0x0" ]; then
    emit_event_snapshot "throttled state=${throttled}"
  fi

  sleep "$INTERVAL"
done
