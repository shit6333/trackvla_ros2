"""Replay the first-frame cases through the upstream classes and diff against ours.

Runs in omtrackvla-dev (the OmTrackVLA container), importing the upstream code
exactly as trained_agent.py does: OpenTrackVLAForWaypoint.from_pretrained for
the planner, VisionFeatureCacher + grid_pool_tokens for the vision tokens, and
the evaluator's step-0 history (the first frame's coarse tokens repeated 31
times, time indices 0..30, the fine tokens at index 31). Each npz written by
firstframe_test.py carries the frame and our backend's tau per prompt; this
prints the max absolute difference per case and prompt.

Run from /workspace/OmTrackVLA under the omtrack env:
    python /tmp/parity_upstream.py /tmp/firstframe
"""
import glob
import os
import sys

import numpy as np
import torch
from PIL import Image

sys.path.insert(0, '/workspace/OmTrackVLA')
from open_trackvla_hf import OpenTrackVLAForWaypoint          # noqa: E402
from cache_gridpool import VisionCacheConfig, VisionFeatureCacher, grid_pool_tokens  # noqa: E402

HISTORY = 31
CKPT = os.environ.get('HF_MODEL_DIR', '/workspace/cache/OmTrackVLA-0.6B')
PROMPTS = {
    'left::canonical': None, 'left::follow': None, 'right::canonical': None,
    'right::follow': None, 'none::neutral': None, 'none::nonsense': None,
}
TEXT = {
    'female_31': {'canonical': 'Stay behind the woman wearing a blue top and sandals.',
                  'follow': 'Follow the woman wearing a blue top and sandals.'},
    'male_36': {'canonical': 'Pursue the individual dressed in brown leather armor and a blue shirt.',
                'follow': 'Follow the person wearing brown leather armor and a blue shirt.'},
    'none': {'neutral': 'follow the person', 'nonsense': 'xyzzy plugh frobnicate'},
}


def main():
    cases_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/firstframe'
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = OpenTrackVLAForWaypoint.from_pretrained(CKPT).to(device).eval()
    enc = VisionFeatureCacher(VisionCacheConfig(image_size=384, batch_size=1, device=str(device)))
    enc.eval()
    print('upstream planner loaded from', CKPT, 'on', device)

    def encode(rgb):
        pil = Image.fromarray(rgb.astype(np.uint8))
        tok_d, Hp, Wp = enc._encode_dino([pil])
        tok_s = enc._encode_siglip([pil], out_hw=(Hp, Wp))
        cat = torch.cat([tok_d, tok_s], dim=-1)
        fine = grid_pool_tokens(cat, Hp, Wp, out_tokens=64)[0].float()
        coarse = grid_pool_tokens(cat, Hp, Wp, out_tokens=4)[0].float()
        return coarse, fine

    def predict(coarse, fine, text):
        hist = [coarse] * HISTORY                       # step-0 padding, as trained_agent does
        coarse_tokens = torch.cat([h.to(device) for h in hist], dim=0).unsqueeze(0)
        coarse_tidx = torch.cat([torch.full((coarse.size(0),), t, dtype=torch.long, device=device)
                                 for t in range(HISTORY)], dim=0).unsqueeze(0)
        fine_tokens = fine.to(device).unsqueeze(0)
        fine_tidx = torch.full((1, fine_tokens.size(1)), HISTORY, dtype=torch.long, device=device)
        with torch.inference_mode():
            tau = model(coarse_tokens, coarse_tidx, fine_tokens, fine_tidx, [text])
        return tau.detach().float().cpu().numpy()[0].astype(np.float64)

    worst = 0.0
    n = 0
    print('%-24s %-18s %10s %10s' % ('case', 'prompt', 'max|diff|', 'ours x1'))
    for f in sorted(glob.glob(os.path.join(cases_dir, '*.npz'))):
        d = np.load(f)
        name = os.path.basename(f)[:-4]
        rgb = d['rgb']
        # Recover which identity stood on which side from the case name.
        base = name.replace('_mirror', '')
        pair = base.split('_')[-1]                       # 'fm' or 'mf'
        left_id, right_id = ('female_31', 'male_36') if pair == 'fm' else ('male_36', 'female_31')
        if name.endswith('_mirror'):
            left_id, right_id = right_id, left_id
        coarse, fine = encode(rgb)
        for key in sorted(k for k in d.files if k.startswith('tau::')):
            _, side, kind = key.split('::')
            ident = {'left': left_id, 'right': right_id, 'none': 'none'}[side]
            text = TEXT[ident][kind]
            ours = d[key]
            theirs = predict(coarse, fine, text)
            diff = float(np.abs(ours - theirs).max())
            worst = max(worst, diff)
            n += 1
            print('%-24s %-18s %10.2e %+10.4f' % (name, side + '::' + kind, diff, ours[1][0]))
    print()
    print('%d comparisons, worst max|diff| = %.3e' % (n, worst))
    print('verdict:', 'IDENTICAL (within fp noise)' if worst < 1e-3 else 'DIFFERENT, investigate')


if __name__ == '__main__':
    main()
