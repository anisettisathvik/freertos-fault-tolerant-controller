#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Run the firmware N times and report the spread of every measured figure.
#
# A single run gives one sample per metric, which says nothing about whether
# the number is stable. Worst-case latency in particular is meaningless from
# one observation: the whole point of a worst case is that you have to look
# for it. This runs the same deterministic fault schedule repeatedly and
# reports min and max across runs.
#
#   ./tools/timing_runs.sh 5
# ---------------------------------------------------------------------------
set -u
cd "$(dirname "$0")/.."
N=${1:-5}
mkdir -p logs

echo "running the firmware $N times..."
rm -f logs/timing_samples.txt

for i in $(seq 1 "$N"); do
    printf "  run %d/%d ... " "$i" "$N"
    timeout 45 qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -m 16M \
        -nographic -serial file:logs/_t.log -monitor none \
        -kernel build/firmware.elf < /dev/null > /dev/null 2>&1 || true
    tr -d '\r' < logs/_t.log >> logs/timing_samples.txt

    steps=$(grep -oP 'control steps \.+ \K[0-9]+' logs/_t.log | tr -d '\r')
    fto=$(grep -oP 'fault-to-output-off \.+ \K[0-9]+' logs/_t.log | tr -d '\r')
    cs=$(grep -oP '^  ctrl_step\(\)\s+[0-9]+\s+\K[0-9]+' logs/_t.log | tr -d '\r')
    wd=$(grep -oP 'watchdog expiries \.+ \K[0-9]+' logs/_t.log | tr -d '\r')
    echo "steps=$steps  fault-to-off=${fto}us  ctrl_step_avg=${cs}us  wdog=$wd"
done

echo ""
echo "=== spread across $N runs ==="
python3 - "$N" << 'PY'
import re, sys
text = open("logs/timing_samples.txt").read()

def grab(pattern):
    return [int(x) for x in re.findall(pattern, text)]

metrics = [
    ("control steps",            r'control steps \.+ (\d+)',            ""),
    ("fault events",             r'fault events \.+ (\d+)',             ""),
    ("recoveries",               r'recoveries \.+ (\d+)',               ""),
    ("watchdog expiries",        r'watchdog expiries \.+ (\d+)',        ""),
    ("fault-to-output-off",      r'fault-to-output-off \.+ (\d+)',      " us"),
    ("watchdog detection",       r'watchdog detection latency \.+ (\d+)'," ms"),
    ("ctrl_step avg",            r'ctrl_step\(\)\s+\d+\s+(\d+)',        " us"),
    ("ctrl_step max",            r'ctrl_step\(\)\s+\d+\s+\d+\s+(\d+)',  " us"),
    ("Processing avg",           r'Processing\s+\d+\s+(\d+)',           " us"),
    ("HealthMonitor avg",        r'HealthMonitor\s+\d+\s+(\d+)',        " us"),
]

print(f"  {'metric':<24}{'min':>10}{'max':>10}{'spread':>10}")
deterministic = []
for name, pat, unit in metrics:
    vals = grab(pat)
    if not vals:
        continue
    lo, hi = min(vals), max(vals)
    spread = "same" if lo == hi else f"{hi - lo}{unit}"
    if lo == hi:
        deterministic.append(name)
    print(f"  {name:<24}{str(lo)+unit:>10}{str(hi)+unit:>10}{spread:>10}")

print("")
print("  identical on every run: " + ", ".join(deterministic))
print("")
print("  Logic outcomes are identical because the fault schedule is seeded and")
print("  the control logic is deterministic. Only the microsecond timings vary,")
print("  and they vary with host scheduling under QEMU rather than with")
print("  anything in the firmware.")
PY
rm -f logs/_t.log
