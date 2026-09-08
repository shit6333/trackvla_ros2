"""First-frame target selection: does the instruction decide which way tau points?

Each case is one rendered opening frame with two Habitat humanoids, left and
right, plus a JSON sidecar giving their identities and bearings. The frame is
fed as the planner's first observation exactly the way the upstream evaluator
does at step 0: the coarse history is the same frame repeated, so there is no
temporal lock-in and appearance plus text is all the model has.

For each case and each instruction the first predicted step tau[1] is turned
into a bearing atan2(y, x) and compared with the two people's bearings; the
closer one is the pick. A horizontally mirrored copy of every case is run too:
if the pick follows the mirror, the model is choosing a side, not a person.

Run inside trackvla-ros2-dev through the entrypoint:
    python3 /tmp/firstframe_test.py /tmp/pairs --out /tmp/firstframe
"""
import argparse
import glob
import json
import os

import numpy as np
from PIL import Image

from vla_tracking.backend_interface import Observation
from vla_tracking_omtrackvla.omtrackvla_backend import OmTrackVLABackend

# Canonical dataset instructions per identity (avatar_renders/manifest.json),
# plus a verb-normalised form so the two prompts differ only in the person.
PROMPTS = {
    'female_31': {
        'canonical': 'Stay behind the woman wearing a blue top and sandals.',
        'follow': 'Follow the woman wearing a blue top and sandals.',
    },
    'male_36': {
        'canonical': 'Pursue the individual dressed in brown leather armor and a blue shirt.',
        'follow': 'Follow the person wearing brown leather armor and a blue shirt.',
    },
}
CONTROLS = {
    'neutral': 'follow the person',
    'nonsense': 'xyzzy plugh frobnicate',
}
HISTORY = 31
DT = 0.1


def run_case(backend, rgb, instruction):
    backend.reset(instruction)
    pred = None
    for i in range(HISTORY):
        pred = backend.infer(Observation(rgb=rgb, stamp_ns=i * 100_000_000, instruction=instruction))
    return np.asarray(pred.waypoints, dtype=np.float64)


def pick(tau, left_bearing, right_bearing):
    """Which person the first step points at, by bearing; +y is left."""
    x, y, th = tau[1]
    b = np.degrees(np.arctan2(y, x))
    return ('left' if abs(b - left_bearing) < abs(b - right_bearing) else 'right'), b, th


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cases_dir')
    ap.add_argument('--out', default='/tmp/firstframe')
    ap.add_argument('--warm-up', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    backend = OmTrackVLABackend()
    backend.configure({'warm_up': a.warm_up})

    cases = sorted(glob.glob(os.path.join(a.cases_dir, '*.json')))
    rows, agg = [], {'target': [], 'mirror_follows_side': []}
    print('%-22s %-8s %-10s %-9s %9s %9s %8s  %s' % (
        'case', 'mirror', 'prompt', 'names', 'tau_y', 'tau_th', 'bearing', 'pick'))
    for jpath in cases:
        meta = json.load(open(jpath))
        name = os.path.basename(jpath)[:-5]
        rgb0 = np.asarray(Image.open(jpath[:-5] + '.png').convert('RGB'))
        for mirrored in (False, True):
            rgb = rgb0[:, ::-1, :].copy() if mirrored else rgb0
            left, right = (meta['right'], meta['left']) if mirrored else (meta['left'], meta['right'])
            lb, rb = (-left['bearing_deg'], -right['bearing_deg']) if mirrored else (left['bearing_deg'], right['bearing_deg'])
            plist = []
            for side, who in (('left', left), ('right', right)):
                for kind, text in PROMPTS[who['identity']].items():
                    plist.append((side, kind, text))
            for kind, text in CONTROLS.items():
                plist.append(('none', kind, text))
            dump = {'rgb': rgb, 'left_bearing': lb, 'right_bearing': rb}
            for names, kind, text in plist:
                tau = run_case(backend, rgb, text)
                chosen, b, th = pick(tau, lb, rb)
                dump[f'tau::{names}::{kind}'] = tau
                hit = (chosen == names) if names != 'none' else None
                rows.append(dict(case=name, mirrored=mirrored, prompt=kind, names=names,
                                 tau_y=float(tau[1][1]), tau_th=float(tau[1][2]),
                                 bearing=float(b), pick=chosen, hit=hit))
                print('%-22s %-8s %-10s %-9s %+9.4f %+9.4f %+8.1f  %s%s' % (
                    name, 'mirror' if mirrored else '', kind, names,
                    tau[1][1], tau[1][2], b, chosen,
                    '  <- OK' if hit else ('  <- WRONG' if hit is False else '')))
                if hit is not None:
                    agg['target'].append(hit)
            np.savez_compressed(os.path.join(a.out, '%s%s.npz' % (name, '_mirror' if mirrored else '')), **dump)
        print()

    json.dump(rows, open(os.path.join(a.out, 'rows.json'), 'w'), indent=1)
    n = len(agg['target'])
    print('=== summary ===')
    print('instruction names a person; first step points at that person: %d / %d = %.0f%%  (chance 50%%)'
          % (sum(agg['target']), n, 100.0 * sum(agg['target']) / max(n, 1)))
    # Side bias: for each (case, prompt) compare original vs mirrored pick.
    by = {}
    for r in rows:
        by.setdefault((r['case'], r['prompt'], r['names']), {})[r['mirrored']] = r['pick']
    same_side = sum(1 for v in by.values() if len(v) == 2 and v[False] == v[True])
    print('same image side picked before and after mirroring: %d / %d  (high = side bias, not person)'
          % (same_side, len(by)))
    left_picks = sum(1 for r in rows if r['pick'] == 'left')
    print('overall picks: left %d / right %d' % (left_picks, len(rows) - left_picks))


if __name__ == '__main__':
    main()
