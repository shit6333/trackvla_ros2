#!/usr/bin/env python3
"""Score which walker the robot actually followed.

Watching a run and deciding by eye which person the robot went after is the
part of this experiment most likely to be wrong, because the two walkers cross
and because "it went right" and "it went to the man in the white shirt" look
identical from the outside. This turns the question into numbers.

Run it inside the Gazebo container while a tracking run is in progress:

    python3 sim/scripts/score_tracking.py --seconds 90

Ground truth for the walkers comes from actor_paths.position_at, because
Gazebo publishes no pose for an actor at all. The robot's pose is the
simulator's own, not the wheel odometry, which drifts.

What the summary means. `nearest` is the share of samples where that walker
was the closest person to the robot, and it is the headline number: two runs
under two instructions should move it if the instruction is doing anything.
`followed` is stricter and requires the walker to also be within reach and
inside the camera, which excludes the case where the robot has lost everyone
and happens to be marginally nearer one of them.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from actor_paths import ACTORS, position_at  # noqa: E402

#: Horizontal half-field of the robot's camera, in degrees.
HALF_FOV = 45.0

#: A walker further than this is not being followed, whatever the ranking says.
FOLLOW_RANGE = 3.0

ROBOT = 'turtlebot3_burger'


def _gz(topic):
    """Return one message from a Gazebo topic, or an empty string.

    Each call opens its own subscription, and under load that occasionally
    outlasts the timeout. A dropped sample costs a row; letting the exception
    out costs the whole run, which by then is several minutes of simulation.
    """
    try:
        return subprocess.run(
            ['gz', 'topic', '-e', '-t', topic, '-n', '1'],
            capture_output=True, text=True, timeout=20,
        ).stdout
    except subprocess.TimeoutExpired:
        return ''


def sim_time():
    """Return the simulator's clock, which the walker paths are indexed by."""
    m = re.search(r'sim_time \{\s*sec: (\d+)\s*nsec: (\d+)', _gz('/world/tracking/stats'))
    return float(m.group(1)) + float(m.group(2)) * 1e-9 if m else None


def robot_pose():
    """Return the robot's (x, y, yaw) from the simulator's ground truth."""
    out = _gz('/world/tracking/dynamic_pose/info')
    m = re.search(
        rf'name: "{ROBOT}".*?position \{{(.*?)\}}.*?orientation \{{(.*?)\}}',
        out, re.S,
    )
    if not m:
        return None

    def g(block, key):
        found = dict(re.findall(r'(\w+): ([-\d.e+]+)', block))
        return float(found.get(key, 0.0))

    pos, ori = m.group(1), m.group(2)
    qw, qz = g(ori, 'w') or 1.0, g(ori, 'z')
    return g(pos, 'x'), g(pos, 'y'), math.degrees(2.0 * math.atan2(qz, qw))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--period', type=float, default=0.5)
    parser.add_argument('--quiet', action='store_true',
                        help='Summary only, no per-sample rows.')
    args = parser.parse_args()

    names = list(ACTORS)
    tally = {n: {'nearest': 0, 'followed': 0, 'in_view': 0, 'centred': 0,
                 'bearing_sum': 0.0, 'dist_sum': 0.0, 'dist_min': 1e9}
             for n in names}
    samples = 0

    if not args.quiet:
        head = ''.join(f'{n[7:]:>26}' for n in names)
        print(f'{"sim_t":>7}{"robot":>18}{head}')
        print(f'{"":>7}{"":>18}' + ''.join(f'{"dist   bearing":>26}' for _ in names))

    started = time.time()
    while time.time() - started < args.seconds:
        t, pose = sim_time(), robot_pose()
        if t is None or pose is None:
            time.sleep(args.period)
            continue
        rx, ry, ryaw = pose

        row, best, best_d = '', None, 1e9
        aimed, aimed_b, bearings = None, 1e9, {}
        for n in names:
            ax, ay = position_at(n, t)
            d = math.hypot(ax - rx, ay - ry)
            bearing = (math.degrees(math.atan2(ay - ry, ax - rx)) - ryaw + 180) % 360 - 180
            bearings[n] = bearing
            in_view = abs(bearing) <= HALF_FOV
            tally[n]['dist_sum'] += d
            tally[n]['dist_min'] = min(tally[n]['dist_min'], d)
            tally[n]['bearing_sum'] += abs(bearing)
            tally[n]['in_view'] += in_view
            if d < best_d:
                best, best_d = n, d
            # Which walker the robot is actually pointed at. Distance alone
            # cannot tell "following someone" from "parked between two people",
            # and the difference is the whole question here.
            if abs(bearing) < aimed_b:
                aimed, aimed_b = n, abs(bearing)
            row += f'{d:>18.2f} m{bearing:>+7.0f}'
        tally[best]['nearest'] += 1
        tally[aimed]['centred'] += 1
        if best_d <= FOLLOW_RANGE and abs(
            (math.degrees(math.atan2(position_at(best, t)[1] - ry,
                                     position_at(best, t)[0] - rx)) - ryaw + 180) % 360 - 180
        ) <= HALF_FOV:
            tally[best]['followed'] += 1
        samples += 1
        if not args.quiet:
            print(f'{t:7.1f}({rx:6.2f},{ry:6.2f}){ryaw:+6.0f}deg{row}')
        time.sleep(args.period)

    if not samples:
        print('no samples: is the world paused?')
        return

    print(f'\n{samples} samples over {args.seconds:.0f} s\n')
    print(f'{"walker":22}{"centred":>9}{"nearest":>9}{"followed":>10}'
          f'{"in view":>9}{"mean |brg|":>12}{"mean dist":>12}{"min dist":>11}')
    for n in names:
        s = tally[n]
        print(f'{n:22}{s["centred"]/samples:>8.0%}{s["nearest"]/samples:>9.0%}'
              f'{s["followed"]/samples:>10.0%}{s["in_view"]/samples:>9.0%}'
              f'{s["bearing_sum"]/samples:>11.0f}d{s["dist_sum"]/samples:>11.2f} m'
              f'{s["dist_min"]:>10.2f} m')

    winner = max(names, key=lambda n: tally[n]['centred'])
    share = tally[winner]['centred'] / samples
    print(f'\naimed at most: {winner} ({share:.0%} of samples)')
    if share < 0.6:
        print('under 60%: the robot is sitting between them, not choosing one')


if __name__ == '__main__':
    main()
