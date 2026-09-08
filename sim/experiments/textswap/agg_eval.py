"""Aggregate SR / CR / TR over OmTrackVLA eval result dirs, paired by episode.

Each result dir holds <scene>/<episode_id>.json with success, collision,
following_rate, total_step, status and (for textswap runs) instruction,
instruction_original, instruction_mode. Metrics:

    SR  success rate         mean(success)
    CR  collision rate       mean(collision == 1)
    TR  tracking rate        mean(following_rate)
    DC  distractor collisions per episode (when recorded)

Given several dirs, the table is computed on the episodes present in ALL of
them, so runs on the same split are compared like for like.

    python3 agg_eval.py sim_data/eval/dt sim_data/eval/dt_textswap_distractor_20260904
"""
import glob
import json
import os
import sys


def load(d):
    out = {}
    for f in glob.glob(os.path.join(d, '*', '*.json')):
        if f.endswith('_info.json'):
            continue
        try:
            r = json.load(open(f))
        except ValueError:
            continue
        if 'success' not in r:
            continue
        key = (os.path.basename(os.path.dirname(f)), os.path.basename(f)[:-5])
        out[key] = r
    return out


def summarise(rows):
    n = len(rows)
    if not n:
        return None
    sr = sum(1.0 if r['success'] else 0.0 for r in rows) / n
    cr = sum(1.0 if r.get('collision') == 1.0 else 0.0 for r in rows) / n
    tr = sum(float(r.get('following_rate', 0.0)) for r in rows) / n
    dc = [r['distractor_collision_count'] for r in rows if 'distractor_collision_count' in r]
    lost = sum(1 for r in rows if r.get('status') == 'Lost') / n
    return dict(n=n, SR=sr, CR=cr, TR=tr, lost=lost,
                DC=(sum(dc) / len(dc)) if dc else None)


def main():
    dirs = sys.argv[1:]
    if not dirs:
        raise SystemExit(__doc__)
    data = {d: load(d) for d in dirs}
    common = set.intersection(*(set(v) for v in data.values())) if len(dirs) > 1 else set(data[dirs[0]])
    print('episodes in common: %d' % len(common))
    print('%-58s %5s %7s %7s %7s %7s %7s' % ('results dir', 'n', 'SR', 'CR', 'TR', 'lost', 'DC'))
    for d in dirs:
        rows = [data[d][k] for k in sorted(common)]
        s = summarise(rows)
        if s is None:
            print('%-58s (no results)' % d)
            continue
        modes = sorted(set(r.get('instruction_mode', 'original') for r in rows))
        print('%-58s %5d %7.3f %7.3f %7.3f %7.3f %7s   mode=%s' % (
            d[-58:], s['n'], s['SR'], s['CR'], s['TR'], s['lost'],
            ('%.2f' % s['DC']) if s['DC'] is not None else '-', ','.join(modes)))
    if len(dirs) > 1:
        base = dirs[0]
        print()
        print('paired vs %s:' % base)
        for d in dirs[1:]:
            flips = sum(1 for k in common if bool(data[base][k]['success']) != bool(data[d][k]['success']))
            print('  %-50s success differs on %d / %d episodes' % (d[-50:], flips, len(common)))


if __name__ == '__main__':
    main()
