"""Generate a shadow trained_agent.py: instruction rewrite plus resume.

The OmTrackVLA worktree is dirty and must not be edited, so this writes a copy
of trained_agent.py to a separate directory that run_textswap.py puts ahead of
the repo on sys.path. Two blocks are inserted; nothing is removed.

1. Instruction rewrite, right after the episode's instruction is read, chosen
   by OMTRACK_INSTRUCTION_MODE:

    original    the dataset instruction, unchanged (baseline)
    distractor  the canonical description of the episode's first distractor
                (extra_humanoid_names[0]) from avatar_renders/manifest.json,
                i.e. a real person in the scene who is NOT the target
    swap        the canonical description of an identity not in the scene
    neutral     'follow the person'
    nonsense    'xyzzy plugh frobnicate'

   The rewritten text is what reaches the planner and what the result json
   records as 'instruction'; the original is kept as 'instruction_original'.

2. Resume, at the top of evaluate_agent, when OMTRACK_RESUME=1: episodes whose
   <save_path>/<scene>/<episode_id>.json already exists are dropped from the
   dataset before batching. Order, seeding and batch boundaries of the
   remaining episodes are unchanged, and no existing file is ever rewritten.
   This is how a run interrupted by the 2026-09-04 host shutdown is finished
   without repeating the 923 episodes it had already written.

Usage (inside omtrackvla-dev, from /workspace/OmTrackVLA):
    python /tmp/textswap/make_textswap_agent.py /tmp/textswap
"""
import os
import shutil
import sys

SRC = '/workspace/OmTrackVLA/trained_agent.py'

INSTR_ANCHOR = "                instruction = env.current_episode.info.get('instruction', None)\n"
INSTR_BLOCK = '''                instruction_original = instruction
                instruction = _textswap_rewrite(env.current_episode, instruction)
'''

RESULT_ANCHOR = "            if instruction is not None:\n                result['instruction'] = instruction\n"
RESULT_BLOCK = RESULT_ANCHOR + "            result['instruction_original'] = instruction_original\n            result['instruction_mode'] = _TS_MODE\n"

RESUME_ANCHOR = "    if robot_config is None:\n        robot_config = GTBBoxAgent(save_path)\n"
RESUME_BLOCK = RESUME_ANCHOR + '''
    # textswap: resume support. Drop episodes that already have a result file
    # so an interrupted run can be finished without repeating or overwriting.
    if _ts_os.environ.get('OMTRACK_RESUME') == '1':
        _keep = []
        _skipped = 0
        for _ep in dataset_split.episodes:
            _scene_key = osp.splitext(osp.basename(_ep.scene_id))[0].split('.')[0]
            if _ts_os.path.exists(_ts_os.path.join(save_path, _scene_key, '%s.json' % _ep.episode_id)):
                _skipped += 1
            else:
                _keep.append(_ep)
        if _skipped:
            print('[textswap] resume: skipping %d episodes with existing results, %d remain'
                  % (_skipped, len(_keep)), flush=True)
        dataset_split.episodes = _keep
        if not _keep:
            print('[textswap] resume: nothing left to do', flush=True)
            return
'''

HELPER = '''

# ---- textswap: instruction rewrite for the text-ablation eval -------------
import json as _ts_json
import os as _ts_os

_TS_MODE = _ts_os.environ.get('OMTRACK_INSTRUCTION_MODE', 'original')
_TS_MANIFEST = None


def _textswap_manifest():
    global _TS_MANIFEST
    if _TS_MANIFEST is None:
        path = _ts_os.path.join(_ts_os.path.dirname(_ts_os.path.abspath(__file__)), 'manifest.json')
        with open(path) as f:
            _TS_MANIFEST = _ts_json.load(f)['identities']
    return _TS_MANIFEST


def _textswap_rewrite(episode, instruction):
    """Return the instruction the planner should be given for this episode."""
    mode = _TS_MODE
    if mode == 'original' or instruction is None:
        return instruction
    if mode == 'neutral':
        return 'follow the person'
    if mode == 'nonsense':
        return 'xyzzy plugh frobnicate'
    info = getattr(episode, 'info', {}) or {}
    manifest = _textswap_manifest()
    if mode == 'distractor':
        others = info.get('extra_humanoid_names') or []
        if not others:
            return instruction
        return manifest[others[0]]['canonical_instruction']
    if mode == 'swap':
        names = sorted(manifest)
        present = set([info.get('main_humanoid_name')] + list(info.get('extra_humanoid_names') or []))
        main = info.get('main_humanoid_name')
        start = names.index(main) if main in names else 0
        for k in range(1, len(names)):
            cand = names[(start + 7 * k) % len(names)]
            if cand not in present:
                return manifest[cand]['canonical_instruction']
        return instruction
    raise ValueError('unknown OMTRACK_INSTRUCTION_MODE: %r' % mode)
'''


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/textswap'
    os.makedirs(out_dir, exist_ok=True)
    src = open(SRC).read()
    for name, anchor in (('instruction', INSTR_ANCHOR), ('result', RESULT_ANCHOR), ('resume', RESUME_ANCHOR)):
        if src.count(anchor) != 1:
            raise SystemExit('%s anchor not found exactly once in %s' % (name, SRC))
    patched = (src.replace(INSTR_ANCHOR, INSTR_ANCHOR + INSTR_BLOCK)
                  .replace(RESULT_ANCHOR, RESULT_BLOCK)
                  .replace(RESUME_ANCHOR, RESUME_BLOCK)
               + HELPER)
    # The resume block runs before the helper section is defined at import
    # time? No: functions are called later, and _ts_os is a module-level name
    # bound when the module finishes importing. evaluate_agent is only invoked
    # afterwards, so the reference resolves.
    dst = os.path.join(out_dir, 'trained_agent.py')
    open(dst, 'w').write(patched)
    shutil.copy('/workspace/OmTrackVLA/avatar_renders/manifest.json', os.path.join(out_dir, 'manifest.json'))
    print('wrote', dst, '(%d lines; original %d)' % (patched.count('\n'), src.count('\n')))


if __name__ == '__main__':
    main()
