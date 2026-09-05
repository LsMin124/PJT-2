#!/bin/bash
# 스팟턴 실측 1회: Isaac(warehouse_sim 래퍼) 기동 → ready → spin_probe → 종료.  인자: 라벨(기본 run)
set +u
HERE=$(dirname "$(readlink -f "$0")"); OUT=$HERE/out; mkdir -p "$OUT"; LABEL=${1:-run}; LOG=$OUT/isaac_$LABEL.log
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
source /opt/ros/humble/setup.bash
if pgrep -f "warehouse_sim.py|wsim_wrap.py" >/dev/null; then echo "[run] Isaac 인스턴스가 이미 떠 있음"; exit 1; fi
( cd ~/isaacsim && exec ./python.sh "$HERE/wsim_wrap.py" > "$LOG" 2>&1 ) & IPID=$!
t0=$(date +%s); ready=0
for k in $(seq 1 150); do
  grep -q "\[wsim\] ready" "$LOG" && { ready=1; break; }
  kill -0 $IPID 2>/dev/null || break
  sleep 2
done
echo "[run] ready=$ready after $(( $(date +%s) - t0 ))s"
if [ $ready = 1 ]; then
  sleep 8
  python3 "$HERE/spin_probe.py" --label "$LABEL" --out "$OUT" 2>&1 | tee "$OUT/probe_$LABEL.log"
fi
pkill -TERM -f "wsim_wrap.py"; for k in $(seq 1 20); do pgrep -f "wsim_wrap.py" >/dev/null || break; sleep 2; done
pkill -KILL -f "wsim_wrap.py" 2>/dev/null
echo "[run] isaac offsets:"; grep "\[wrap\]" "$LOG"
[ $ready = 1 ] || { echo "[run] NOT READY — isaac log tail:"; tail -30 "$LOG"; }
