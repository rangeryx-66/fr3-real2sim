"""Replay private, archived RGB-D stages; no GT, inference, planning or Isaac.

Input directories are read only. Output contains derived diagnostic arrays.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import native_g1_metric_rgbd_geometry as metric
from native_g1_observed_regions import cloud


def accepted(episode):
    rows = json.loads((episode / 'AGENT_OBSERVATIONS.json').read_text())
    return [episode / 'rgbd' / Path(r['folder']).name for r in rows
            if r['accepted_for_geometry']]


def stat(points, axis):
    return dict(points=len(points), extent_mm=float(np.ptp(points @ axis) * 1000)
                if len(points) else None)


def replay(episode, output):
    output.mkdir(parents=True, exist_ok=True)
    folders = accepted(episode)
    first = metric.preserve(folders[0])
    axis = np.asarray(first['audit']['axis_world'])
    frame = np.asarray(first['audit']['frame_world'])
    origin = np.median(first['points'], axis=0)
    rows = []
    fig, axes = plt.subplots(len(folders), 5, figsize=(15, 4 * len(folders)), squeeze=False)
    hashes = {}
    for i, folder in enumerate(folders):
        meta, depth, _, _, P, valid = cloud(folder)
        raw = metric.load_seed(folder) & valid
        body = np.load(folder / 'handle_mask.npy').astype(bool) & valid
        own = metric.preserve(folder)
        seed = metric.projected_seed(folder, first['points'])
        projected = metric.preserve(folder, seed, first['plane']) if i else first
        metric.save_view(own, output / ('independent_%02d' % i))
        metric.save_view(projected, output / ('frozen_%02d' % i))
        np.save(output / ('projected_seed_%02d.npy' % i), seed)
        sets = [P[raw], P[body], own['points'], P[seed], projected['points']]
        names = ['Own SAM seed', 'Depth-layer body', 'Own preserved', 'Projected first seed', 'Frozen contribution']
        row = dict(view=folder.name, stages={n: stat(p, axis) for n, p in zip(names, sets)},
                   own_panel=own['audit']['panel_normal_world'],
                   projected_vs_own_retained_pixels=int(np.sum(own['mask'] & ~projected['mask'])))
        rows.append(row)
        for j, (name, p) in enumerate(zip(names, sets)):
            q = (p - origin) @ frame * 1000
            axes[i, j].scatter(q[:, 1], q[:, 0], s=.4, c=q[:, 2], cmap='viridis')
            axes[i, j].set_title('%s / %s\n%d pts, %.1f mm' %
                                 (folder.name, name, len(p), np.ptp(q[:, 0]) if len(p) else 0))
            axes[i, j].set_xlabel('Transverse (mm)')
            axes[i, j].set_ylabel('Longitudinal (mm)')
            axes[i, j].set_xlim(-30, 30)
        for name in ['camera.json', 'depth_m.npy', 'sam3_seed_mask.npy', 'handle_mask.npy', 'robot_self_exclusion.npy']:
            p = folder / name
            if p.exists(): hashes[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
    all_q = np.vstack([metric.preserve(f)['points'] for f in folders]) @ axis * 1000 - origin @ axis * 1000
    for ax in axes.ravel(): ax.set_ylim(all_q.min()-5, all_q.max()+5)
    fig.tight_layout(); fig.savefig(output/'stages.png', dpi=150); plt.close(fig)
    result = dict(episode=str(episode), axis_world=axis.tolist(), rows=rows,
                  input_sha256=hashes, GT_inputs=False)
    (output/'DIAGNOSIS.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(replay(a.episode, a.output)))
