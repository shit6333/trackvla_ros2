"""First frames of real DT evaluation episodes: does the first step aim at the target?

Uses the opening frame of four episodes the user ran in Habitat (scene
tQ5s4ShP627), fed as the evaluator's step 0 (the frame padded 31 times), under
the episode's own instruction, the other episodes' instructions as wrong-person
controls, a neutral prompt and nonsense. Each frame is also mirrored.

The target's image column was read off the frames by eye (384 px wide, centre
192) and converted to a bearing in the robot's convention (+ left):
    bearing = -atan((x - 192) / 192 * tan 45 deg)

Run inside trackvla-ros2-dev through the entrypoint:
    python3 /tmp/dtff_test.py
"""
import json
import os

import cv2
import numpy as np

from vla_tracking.backend_interface import Observation
from vla_tracking_omtrackvla.omtrackvla_backend import OmTrackVLABackend

DIR = '/tmp/dtff'
EPS = [2, 3, 4, 6]
HISTORY = 31

# Target column per episode (and a note on ambiguity), from looking at the frames.
TARGET_X = {2: 245, 3: 185, 4: 165, 6: 205}
DISTRACTOR_X = {2: 205, 3: 120, 4: 35, 6: 130}
NOTE = {3: 'ambiguous: near olive uniform at x~120 also has a red belt'}

CONTROLS = {'neutral': 'follow the person', 'nonsense': 'xyzzy plugh frobnicate'}


def bearing_of_x(x):
    return float(np.degrees(-np.arctan((x - 192.0) / 192.0 * np.tan(np.radians(45.0)))))


def run(backend, rgb, text):
    backend.reset(text)
    pred = None
    for i in range(HISTORY):
        pred = backend.infer(Observation(rgb=rgb, stamp_ns=i * 100_000_000, instruction=text))
    return np.asarray(pred.waypoints, dtype=np.float64)


def main():
    backend = OmTrackVLABackend()
    backend.configure({'warm_up': False})

    instr = {n: json.load(open(os.path.join(DIR, '%d.json' % n)))['instruction'] for n in EPS}
    distinct = []
    for n in EPS:
        if instr[n] not in distinct:
            distinct.append(instr[n])

    out = {}
    print('%-4s %-7s %-14s %+8s %+8s %+8s %+8s  %-8s %-8s  %s' % (
        'ep', 'mirror', 'prompt', 'x1', 'y1', 'th1', 'brg', 'target', 'distr', 'note'))
    for n in EPS:
        bgr = cv2.imread(os.path.join(DIR, '%d_f0.png' % n))
        rgb0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        for mirrored in (False, True):
            rgb = rgb0[:, ::-1, :].copy() if mirrored else rgb0
            tx = 383 - TARGET_X[n] if mirrored else TARGET_X[n]
            dx = 383 - DISTRACTOR_X[n] if mirrored else DISTRACTOR_X[n]
            tb, db = bearing_of_x(tx), bearing_of_x(dx)
            prompts = [('own', instr[n])]
            prompts += [('other%d' % i, t) for i, t in enumerate(distinct) if t != instr[n]]
            prompts += list(CONTROLS.items())
            for kind, text in prompts:
                tau = run(backend, rgb, text)
                x1, y1, th1 = tau[1]
                b = float(np.degrees(np.arctan2(y1, x1)))
                nearer = 'target' if abs(b - tb) < abs(b - db) else 'distr'
                key = '%d%s::%s' % (n, '_m' if mirrored else '', kind)
                out[key] = dict(tau=tau.tolist(), bearing=b, target_bearing=tb, distractor_bearing=db,
                                nearer=nearer, text=text)
                print('%-4d %-7s %-14s %+8.4f %+8.4f %+8.4f %+8.1f  %+7.1f  %+7.1f  %s%s' % (
                    n, 'mirror' if mirrored else '', kind, x1, y1, th1, b, tb, db,
                    ('-> ' + nearer) if kind == 'own' else '', ('  ' + NOTE[n]) if (n in NOTE and kind == 'own' and not mirrored) else ''))
        print()
    json.dump(out, open(os.path.join(DIR, 'results.json'), 'w'), indent=1)

    ys = np.array([v['tau'][1][1] for v in out.values()])
    ths = np.array([v['tau'][1][2] for v in out.values()])
    xs = np.array([v['tau'][1][0] for v in out.values()])
    print('=== %d runs: tau[1].x mean %+.4f | |y| max %.4f | |theta| max %.4f ===' % (
        len(out), xs.mean(), np.abs(ys).max(), np.abs(ths).max()))
    print('bearings needed to reach the target: ' + ', '.join('ep%d %+.0f deg' % (n, bearing_of_x(TARGET_X[n])) for n in EPS))


if __name__ == '__main__':
    main()
