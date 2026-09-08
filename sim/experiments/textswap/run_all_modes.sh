#!/usr/bin/env bash
# Four instruction modes on the same 100-episode DT split, sequentially.
# Each writes to its own NEW dir under sim_data/eval (the launcher refuses
# existing dirs); nothing already there is touched.
set -u
source /opt/conda/etc/profile.d/conda.sh && conda activate omtrack
cd /workspace/OmTrackVLA
export HF_MODEL_DIR=/workspace/cache/OmTrackVLA-0.6B
STAMP=20260904
for MODE in original distractor neutral nonsense; do
  OUT=sim_data/eval/dt_textswap_${MODE}_${STAMP}
  echo "[$(date +%H:%M:%S)] === mode=$MODE -> $OUT ==="
  SAVE_VIDEO=0 CUDA_VISIBLE_DEVICES=0 python /tmp/textswap/run_textswap.py \
      --mode $MODE --split-num 14 --split-id 0 --save-path $OUT \
      > /tmp/textswap/log_${MODE}.txt 2>&1
  echo "[$(date +%H:%M:%S)] mode=$MODE exit=$?  episodes=$(find $OUT -name "*.json" -not -name "*_info.json" 2>/dev/null | wc -l)"
done
echo "[$(date +%H:%M:%S)] ALL DONE"
