#!/bin/bash
# Measure CPU/RAM cost of the libcamera_ros_driver on the Raspberry Pi.
#
# Reports three layers per sample (all true *interval* rates, read from /proc --
# NOT ps's misleading lifetime average):
#   1. driver  : summed %CPU of the driver process(es)   (can exceed 100% = multi-core)
#   2. system  : whole-Pi busy %  (0..100, idle-subtracted -> folds in transport/IRQ)
#   3. net      : eth0 TX MB/s     (catches a gigabit-bandwidth ceiling)
#
# Layout-aware PID selection (ROS1 nodelet managers):
#   stereo : one shared nodelet manager -> exactly 1 driver PID expected
#   mono    : two separate launches      -> 2 driver PIDs, summed
#
# Usage:
#   ./measure_cpu.sh stereo [duration_s] [iface] [match]
#   ./measure_cpu.sh mono   [duration_s] [iface] [match]
# Defaults: duration=60  iface=eth0  match=<mode-specific manager name(s)>
#   mono   -> camera_(front|back)_manager   (two nodelet managers, summed)
#   stereo -> stereo_manager                 (one shared nodelet manager)
# Pass a 4th arg to override the match (an extended regex, matched against cmdline).

set -u

MODE=${1:?usage: $0 <stereo|mono> [duration_s] [iface] [match]}
DURATION=${2:-60}
IFACE=${3:-eth0}

case "$MODE" in
  stereo) WANT=1; DEFAULT_MATCH='stereo_manager' ;;
  mono)   WANT=2; DEFAULT_MATCH='camera_(front|back)_manager' ;;
  *) echo "MODE must be 'stereo' (1 process) or 'mono' (2 processes)"; exit 1 ;;
esac
MATCH=${4:-$DEFAULT_MATCH}

NCPU=$(nproc)

# --- find the driver PID(s), excluding ourselves / grep / the launch wrapper ----
mapfile -t PIDS < <(pgrep -f "$MATCH" | while read -r p; do
  cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null)
  case "$cmd" in
    *measure_cpu*|*pgrep*|*roslaunch*) continue ;;   # skip self + launcher
    *) echo "$p" ;;                                    # pgrep -f already applied the regex
  esac
done)

if [ "${#PIDS[@]}" -eq 0 ]; then
  echo "No process matching '$MATCH' found. Is the driver running?"; exit 1
fi
if [ "${#PIDS[@]}" -ne "$WANT" ]; then
  echo "WARNING: mode '$MODE' expects $WANT driver process(es) but found ${#PIDS[@]}: ${PIDS[*]}"
  echo "         (continuing, summing all of them)"
fi
echo "Monitoring PID(s): ${PIDS[*]}  | cores: $NCPU | iface: $IFACE | ${DURATION}s"

# --- /proc readers (interval rates) --------------------------------------------
# total + idle jiffies across all cores
read_cpu() {  # echoes "<total> <idle>"
  awk '/^cpu /{ tot=0; for(i=2;i<=NF;i++) tot+=$i; print tot, $5+$6 }' /proc/stat
}
# summed utime+stime jiffies for the tracked PIDs. comm may contain spaces/parens,
# so parse the remainder after the last ')': there state=f1, utime=f12, stime=f13.
read_proc() {
  local sum=0 p s
  for p in "${PIDS[@]}"; do
    s=$(awk '{ r=substr($0, index($0,")")+2); split(r,f," "); print f[12]+f[13] }' \
          "/proc/$p/stat" 2>/dev/null) || s=0
    sum=$(( sum + ${s:-0} ))
  done
  echo "$sum"
}
read_tx() { cat "/sys/class/net/$IFACE/statistics/tx_bytes" 2>/dev/null || echo 0; }

LOG_DIR="$(pwd)/logs"; mkdir -p "$LOG_DIR"
TS=$(date +%Y%m%d_%H%M%S)
OUT="$LOG_DIR/measure_${MODE}_${TS}.csv"
echo "t,driver_cpu_pct,system_busy_pct,eth_tx_MBps,driver_rss_mb" > "$OUT"
echo "Saving: $OUT"
echo ""

# --- sample loop (1s interval; each reading is a delta over that second) --------
read t0 i0 < <(read_cpu); p0=$(read_proc); x0=$(read_tx)
for ((i = 1; i <= DURATION; i++)); do
  sleep 1
  # bail if a tracked process died (silent camera death is a real failure mode)
  for p in "${PIDS[@]}"; do
    [ -d "/proc/$p" ] || { echo "WARNING: PID $p terminated at sample $i"; break 2; }
  done

  read t1 i1 < <(read_cpu); p1=$(read_proc); x1=$(read_tx)
  dtot=$(( t1 - t0 )); didle=$(( i1 - i0 )); dproc=$(( p1 - p0 ))
  rss_mb=$(awk '{s+=$1}END{printf "%.1f", s*4/1024}' \
            <(for p in "${PIDS[@]}"; do awk '{print $24}' "/proc/$p/stat" 2>/dev/null; done))

  # driver %CPU (one-core scale, like top): jiffies vs one core's jiffies this interval
  driver=$(awk -v dp="$dproc" -v dt="$dtot" -v n="$NCPU" \
            'BEGIN{ printf "%.1f", (dt>0)? 100*dp/(dt/n) : 0 }')
  # system busy % (0..100, idle-subtracted)
  sysb=$(awk -v dt="$dtot" -v di="$didle" 'BEGIN{ printf "%.1f", (dt>0)? 100*(dt-di)/dt : 0 }')
  # eth0 TX MB/s over the 1s interval
  txmb=$(awk -v dx="$((x1 - x0))" 'BEGIN{ printf "%.1f", dx/1048576 }')

  echo "$(date +%s),$driver,$sysb,$txmb,$rss_mb" >> "$OUT"
  [ $((i % 10)) -eq 0 ] && printf "[%d/%d] driver:%s%%  system:%s%%  eth0:%s MB/s  rss:%s MB\n" \
                                   "$i" "$DURATION" "$driver" "$sysb" "$txmb" "$rss_mb"
  t0=$t1; i0=$i1; p0=$p1; x0=$x1
done

echo ""
echo "=== RESULTS ($MODE, ${#PIDS[@]} proc, $NCPU cores) ==="
awk -F',' 'NR>1{
  d+=$2; s+=$3; n+=$4; r+=$5; c++
  if($2>md)md=$2; if($3>ms)ms=$3; if($4>mn)mn=$4
}END{ if(c){
  printf "driver  CPU  avg %.1f%%  peak %.1f%%   (=%.2f cores avg)\n", d/c, md, d/c/100
  printf "system busy  avg %.1f%%  peak %.1f%%   (whole Pi, all cores)\n", s/c, ms
  printf "eth0    TX   avg %.1f MB/s  peak %.1f MB/s\n", n/c, mn
  printf "driver  RSS  avg %.1f MB\n", r/c
  printf "samples %d\n", c
}}' "$OUT"
echo "thread count: $(for p in "${PIDS[@]}"; do ls "/proc/$p/task" 2>/dev/null | wc -l; done | paste -sd+ | bc 2>/dev/null)"
# ponytail: pure /proc, no sysstat dep; O(#pids) parse per second is plenty for 2 procs.
#           Run with rviz subscribed AND closed -- the system_busy delta = the transport
#           cost (the no-subscriber gate avoids it). Nodelet shared-memory helps only a
#           co-located consumer, not rviz-over-Ethernet, which is gigabit-bandwidth bound.
