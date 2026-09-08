"""Frames 0..30 as real history, then the prompt at frame 31: does text steer?

The first-frame tests showed the planner drives straight ahead at step 0 under
any instruction. This asks the question the way DT actually poses it: give the
model the first 31 frames of a real episode as its history, exactly as the
Habitat evaluator did, and read the prediction at frame 31 under the episode's
own instruction, the other episodes' instructions, a neutral prompt and
nonsense. Every frame is also mirrored.

Two things come out:
  * temporal parity: under the episode's own instruction, tau[1]/dt at each
    step is compared with the base_velocity the real run actually took at that
    step (from <ep>_info.json). Match means the replay reproduces the run;
  * text effect with history: how the frame-31 prediction moves between
    prompts, against the target's and distractor's bearings at frame 31.

Target and distractor image columns at frame 31 were read off the frames
(384 px wide, centre 192) and converted to bearings (+ left).

Run inside trackvla-ros2-dev through the entrypoint:
    python3 /tmp/dtseq_test.py
"""
import json
import os

import cv2
import numpy as np

from vla_tracking.backend_interface import Observation
from vla_tracking_omtrackvla.omtrackvla_backend import OmTrackVLABackend

DIR = '/tmp/dtff'
EPS = [2, 3, 4, 6]
N_HIST = 31          # frames 0..30
QUERY = 31           # the frame the prompts are compared on
DT = 0.1

TARGET_X31 = {2: 250, 3: None, 4: 130, 6: 370}
DISTRACTOR_X31 = {2: None, 3: 190, 4: 220, 6: 60}
NOTE = {3: 'target ~2.9 m away and not clearly visible; the near uniform is a distractor'}
CONTROLS = {'neutral': 'follow the person', 'nonsense': 'xyzzy plugh frobnicate'}


def bearing_of_x(x):
    if x is None:
        return None
    return float(np.degrees(-np.arctan((x - 192.0) / 192.0)))


def frames(n, count):
    cap = cv2.VideoCapture(os.path.join(DIR, '%d.mp4' % n))
    out = []
    for _ in range(count):
        ok, f = cap.read()
        if not ok:
            break
        out.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
    return out


def replay(backend, seq, text):
    """Feed seq in order under one instruction; return every step's waypoints."""
    backend.reset(text)
    preds = []
    for i, f in enumerate(seq):
        p = backend.infer(Observation(rgb=f, stamp_ns=i * 100_000_000, instruction=text))
        preds.append(np.asarray(p.waypoints, dtype=np.float64))
    return preds


def fmt_vel(tau):
    x, y, th = tau[1]
    return x / DT, y / DT, th / DT


def main():
    backend = OmTrackVLABackend()
    backend.configure({'warm_up': False})

    instr = {n: json.load(open(os.path.join(DIR, '%d.json' % n)))['instruction'] for n in EPS}
    info = {n: json.load(open(os.path.join(DIR, '%d_info.json' % n))) for n in EPS}
    distinct = []
    for n in EPS:
        if instr[n] not in distinct:
            distinct.append(instr[n])

    results = {}
    for n in EPS:
        seq = frames(n, QUERY + 1)
        assert len(seq) == QUERY + 1, (n, len(seq))
        real = {e['step']: e['base_velocity'] for e in info[n]}

        # --- temporal parity under the episode's own instruction ---------
        preds = replay(backend, seq, instr[n])
        print('=== ep %d  %s ===' % (n, instr[n]))
        print('temporal parity, own instruction: replay tau[1]/dt vs real base_velocity')
        print('  %4s %24s %24s %8s' % ('step', 'replay (vx vy wz)', 'real (vx vy wz)', 'max|d|'))
        worst = 0.0
        for step in (1, 5, 10, 15, 20, 25, 30, 31):
            if step >= len(preds) or step not in real:
                continue
            r = fmt_vel(preds[step])
            g = real[step]
            d = max(abs(a - b) for a, b in zip(r, g))
            worst = max(worst, d)
            print('  %4d %8.3f %7.3f %7.3f  %8.3f %7.3f %7.3f  %8.3f' % (step, r[0], r[1], r[2], g[0], g[1], g[2], d))
        print('  worst max|diff| over listed steps: %.3f' % worst)

        # --- text effect at frame 31, with real history ------------------
        prompts = [('own', instr[n])]
        prompts += [('other%d' % i, t) for i, t in enumerate(distinct) if t != instr[n]]
        prompts += list(CONTROLS.items())
        print('prediction at frame %d under each prompt' % QUERY)
        print('  %-7s %-9s %8s %8s %8s %8s %8s   %s' % ('mirror', 'prompt', 'vx', 'vy', 'wz', 'brg1', 'brg8', 'target/distractor brg'))
        for mirrored in (False, True):
            s = [f[:, ::-1, :].copy() for f in seq] if mirrored else seq
            sign = -1.0 if mirrored else 1.0
            tb = bearing_of_x(TARGET_X31[n])
            db = bearing_of_x(DISTRACTOR_X31[n])
            tb = None if tb is None else sign * tb
            db = None if db is None else sign * db
            taus = {}
            for kind, text in prompts:
                tau = replay(backend, s, text)[QUERY]
                taus[kind] = tau
                vx, vy, wz = fmt_vel(tau)
                b1 = float(np.degrees(np.arctan2(tau[1][1], tau[1][0])))
                b8 = float(np.degrees(np.arctan2(tau[-1][1], tau[-1][0])))
                results['%d%s::%s' % (n, '_m' if mirrored else '', kind)] = dict(
                    tau=tau.tolist(), vx=vx, vy=vy, wz=wz, brg1=b1, brg8=b8,
                    target_brg=tb, distractor_brg=db, text=text)
                print('  %-7s %-9s %8.3f %8.3f %8.3f %8.1f %8.1f   T %s  D %s' % (
                    'mirror' if mirrored else '', kind, vx, vy, wz, b1, b8,
                    '%+.0f' % tb if tb is not None else ' -', '%+.0f' % db if db is not None else ' -'))
            own = taus['own']
            spread = max(float(np.abs(taus[k] - own).max()) for k in taus if k != 'own')
            print('  %-7s max|tau diff| between own and any other prompt: %.4f' % ('mirror' if mirrored else '', spread))
        if n in NOTE:
            print('  note:', NOTE[n])
        print()

    json.dump(results, open(os.path.join(DIR, 'seq_results.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
