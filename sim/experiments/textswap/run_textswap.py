"""Launch the DT eval with a rewritten instruction, into a fresh results dir.

Equivalent to eval_dt.sh with two guards and one substitution:
  * the shadow trained_agent.py from /tmp/textswap is put ahead of the repo on
    sys.path, so the worktree is untouched;
  * --save-path must not already exist, so no earlier results can be
    overwritten (they are under sim_data/eval/, the user's archive) -- unless
    --resume is given, which allows an existing dir and makes the shadow
    module skip every episode that already has a result file there;
  * OMTRACK_INSTRUCTION_MODE selects what the planner is told.

Run inside omtrackvla-dev under the omtrack env, from /workspace/OmTrackVLA:

    SAVE_VIDEO=0 python /tmp/textswap/run_textswap.py --mode distractor \\
        --split-num 14 --split-id 0 --save-path sim_data/eval/dt_textswap_distractor_20260904

    # finish an interrupted run without repeating what it already wrote
    SAVE_VIDEO=1 python /tmp/textswap/run_textswap.py --mode original --resume \\
        --split-num 1 --split-id 0 --save-path sim_data/eval/dt_full_restore_20260904

--split-num N --split-id 0 evaluates the first ceil(1405/N) episodes of DT val
in dataset order, so every mode run with the same split sees the same
episodes and the comparison is paired.
"""
import argparse
import os
import runpy
import sys

REPO = '/workspace/OmTrackVLA'
SHADOW = os.path.dirname(os.path.abspath(__file__))
MODES = ('original', 'distractor', 'swap', 'neutral', 'nonsense')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=MODES, required=True)
    ap.add_argument('--split-num', type=int, required=True)
    ap.add_argument('--split-id', type=int, default=0)
    ap.add_argument('--save-path', required=True)
    ap.add_argument('--resume', action='store_true',
                    help='allow an existing save path and skip episodes already written there')
    ap.add_argument('--exp-config', default='habitat-lab/habitat/config/benchmark/nav/track/track_infer_dt.yaml')
    a = ap.parse_args()

    os.chdir(REPO)
    save = os.path.abspath(a.save_path)
    archive = os.path.abspath(os.path.join(REPO, 'sim_data', 'eval'))
    if os.path.exists(save) and not a.resume:
        raise SystemExit('refusing to write into an existing results dir: %s (use --resume to finish it)' % save)
    if not save.startswith(archive + os.sep):
        raise SystemExit('save path must be a new dir under %s' % archive)
    if not os.path.exists(os.path.join(SHADOW, 'trained_agent.py')):
        raise SystemExit('run make_textswap_agent.py first; no shadow module in %s' % SHADOW)

    os.environ['OMTRACK_INSTRUCTION_MODE'] = a.mode
    os.environ['OMTRACK_RESUME'] = '1' if a.resume else '0'
    os.environ.setdefault('HF_MODEL_DIR', '/workspace/cache/OmTrackVLA-0.6B')
    os.environ.setdefault('SAVE_VIDEO', '0')
    sys.path[:0] = [SHADOW, REPO, os.path.join(REPO, 'habitat-lab')]

    sys.argv = ['run_eval.py',
                '--split-num', str(a.split_num), '--split-id', str(a.split_id),
                '--exp-config', a.exp_config, '--run-type', 'eval',
                '--save-path', a.save_path]
    print('[textswap] mode=%s resume=%s save=%s shadow=%s' % (a.mode, a.resume, a.save_path, SHADOW), flush=True)
    runpy.run_path(os.path.join(REPO, 'run_eval.py'), run_name='__main__')


if __name__ == '__main__':
    main()
