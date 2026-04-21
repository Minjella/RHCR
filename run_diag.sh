#!/usr/bin/env bash
# Section-mode wall-clock 진단 스크립트 (m=900, simulation_time=1000)
#
# 사용법:
#   bash run_diag.sh                   # 기본: seed=7 하나만
#   SEEDS="1 7 13" bash run_diag.sh    # 여러 seed
#   SIM_TIME=500 bash run_diag.sh      # 더 빠르게 (100 planning call)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO_ROOT"

SEEDS="${SEEDS:-7}"
K="${K:-900}"
SIM_TIME="${SIM_TIME:-1000}"
W="${W:-10}"

echo ">>> [1/N] Building (incremental)..."
if [ ! -d build ]; then
  cmake -S . -B build
fi
cmake --build build -j

mkdir -p diag_logs

run_one() {
  local mode="$1"
  local seed="$2"
  local log="diag_logs/diag_${mode}_m${K}_seed${seed}_t${SIM_TIME}.log"
  echo ""
  echo ">>> [$mode, seed=$seed] → $log"
  /usr/bin/time -l env RHCR_SOLVER_MODE="$mode" ./lifelong \
      -m maps/sorting_map.grid --scenario=SORTING \
      --simulation_window=5 --planning_window="$W" \
      --solver=PBS --simulation_time="$SIM_TIME" \
      --seed="$seed" -k "$K" 2>&1 | tee "$log"
  echo "$log"
}

SEC_LOGS=()
BASE_LOGS=()
for seed in $SEEDS; do
  SEC_LOGS+=("$(run_one section "$seed" | tail -1)")
  BASE_LOGS+=("$(run_one baseline "$seed" | tail -1)")
done

echo ""
echo "=========================================================="
echo " SECTION [DIAG] output"
echo "=========================================================="
for f in "${SEC_LOGS[@]}"; do
  echo ""
  echo "--- $f ---"
  grep -E '^\[DIAG\]' "$f" || echo "(no [DIAG] lines — 빌드가 instrument된 코드로 됐는지 확인)"
done

echo ""
echo "=========================================================="
echo " Wall-clock summary (real time from /usr/bin/time -l)"
echo "=========================================================="
for f in "${SEC_LOGS[@]}" "${BASE_LOGS[@]}"; do
  echo "--- $f ---"
  grep -E '^[[:space:]]*[0-9.]+[[:space:]]+(real|user|sys)' "$f" \
    || grep -E 'real\s+[0-9]' "$f" \
    || tail -10 "$f" | grep -iE 'real|elapsed'
done

echo ""
echo "모든 로그는 $REPO_ROOT/diag_logs/ 에 저장됨."
