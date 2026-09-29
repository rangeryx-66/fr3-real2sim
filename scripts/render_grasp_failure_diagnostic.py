"""Render rejected Dex1 grasp proposals over the original calibrated RGB frame.

This is a diagnostic animation of ghost gripper poses. It does not simulate or
claim physical execution of candidates rejected by the grasp backend.
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from grasp_compare.dex1_geometry import Dex1Geometry, transform_points


def draw_candidate(background, candidate, geom, K, T_C_B, amount):
    frame = background.copy()
    pose = np.asarray(candidate['T_B_TCP'], dtype=float).copy()
    pose[:3, 3] -= amount * pose[:3, 2]
    T_C_TCP = T_C_B @ pose
    pieces = geom.meshes_in_tcp(min(max(candidate.get('width_m') or geom.open_width,
                                       geom.closed_width), geom.open_width))
    layer = frame.copy()
    palette = {'base_link': (90, 90, 90), 'Link1_3': (15, 50, 245),
               'Link2_3': (15, 50, 245)}
    for name, mesh in pieces:
        points_C = transform_points(T_C_TCP, np.asarray(mesh.vertices))
        valid = points_C[:, 2] > .05
        if valid.sum() < 3:
            continue
        p = points_C[valid]
        uv = np.column_stack((K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2],
                              K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]))
        hull = cv2.convexHull(np.asarray(uv, dtype=np.int32))
        color = palette.get(name, (35, 135, 245))
        cv2.fillConvexPoly(layer, hull, color)
        cv2.polylines(frame, [hull], True, color, 2, cv2.LINE_AA)
    frame = cv2.addWeighted(layer, .40, frame, .60, 0)
    origin = T_C_TCP[:3, 3]
    if origin[2] > .05:
        u = round(K[0, 0] * origin[0] / origin[2] + K[0, 2])
        v = round(K[1, 1] * origin[1] / origin[2] + K[1, 2])
        cv2.drawMarker(frame, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--provider', default='graspgenx')
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidates', type=int, default=5)
    args = parser.parse_args()
    trial = json.loads(args.trial.read_text())
    if trial.get('success') or trial.get('category') != 'NO_EXECUTABLE_CANDIDATE':
        raise ValueError('diagnostic expects a trial rejected before execution')
    scene_dir = ROOT / 'results/grasp_model_execution' / f'seed_{args.seed}'
    document = json.loads((scene_dir / f'{args.provider}_grasps.json').read_text())
    diagnostic = json.loads((scene_dir / 'diagnostics/candidates.json').read_text())
    source_by_rank = {c['rank']: c for c in diagnostic['raw_candidates'][args.provider]}
    grasps = []
    for grasp in document['grasps'][:args.candidates]:
        source = source_by_rank[grasp['source_rank']]
        grasps.append({'rank': grasp['rank'], 'T_B_TCP': source['T_B_TCP'],
                       'width_m': grasp['width']})
    if not grasps:
        raise ValueError('no rejected proposals to visualize')
    with np.load(scene_dir / 'diagnostics/shared_input.npz') as data:
        rgb = np.asarray(data['rgb'])[..., :3]
        K = np.asarray(data['K'], dtype=float)
        T_C_B = np.linalg.inv(np.asarray(data['T_B_C'], dtype=float))
    if rgb.shape[:2] != (480, 640):
        raise ValueError(f'unexpected RGB shape {rgb.shape}')
    base = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    base = cv2.resize(base, (720, 540), interpolation=cv2.INTER_LINEAR)
    K = K.copy(); K[0, :] *= 720 / 640; K[1, :] *= 540 / 480
    geom = Dex1Geometry()
    by_rank = {row['rank']: row for row in trial['candidates']}
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        raise FileNotFoundError('ffmpeg')
    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
               '-pixel_format', 'bgr24', '-video_size', '960x540', '-framerate', '30',
               '-i', 'pipe:0', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '18',
               '-pix_fmt', 'yuv420p', str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    for candidate in grasps:
        row = by_rank.get(candidate['rank'], {})
        failures = row.get('failures', {})
        reason = max(failures, key=failures.get) if failures else row.get('status', 'REJECTED')
        for frame_id in range(75):
            phase = frame_id / 74
            retreat = .08 * (1 - min(1, phase * 1.6))
            rendered = draw_candidate(base, candidate, geom, K, T_C_B, retreat)
            panel = np.full((540, 240, 3), (31, 34, 40), dtype=np.uint8)
            name = trial['object']['id'].replace('_', ' ').upper()
            labels = [(name, 36, .72, (255, 255, 255)),
                      ('GraspGenX + Dex1', 72, .48, (215, 215, 215)),
                      (f'Seed {args.seed}', 106, .48, (215, 215, 215)),
                      (f'Candidate #{candidate["rank"]}', 158, .53, (100, 210, 255)),
                      ('GHOST POSE', 215, .57, (45, 215, 255)),
                      ('NOT EXECUTED', 247, .57, (45, 215, 255)),
                      ('Rejected before', 304, .52, (230, 230, 230)),
                      ('planning/lift:', 329, .52, (230, 230, 230)),
                      (reason, 372, .47, (80, 90, 250)),
                      ('No physical success', 445, .48, (235, 235, 235)),
                      ('in 5 matched poses', 470, .48, (235, 235, 235))]
            for label, y, scale, color in labels:
                cv2.putText(panel, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                            scale, color, 1 if scale < .6 else 2, cv2.LINE_AA)
            process.stdin.write(np.concatenate((rendered, panel), axis=1).tobytes())
    process.stdin.close()
    if process.wait(timeout=120):
        raise RuntimeError('ffmpeg failed')
    report = {'type': 'diagnostic ghost-pose animation; no physical execution',
              'seed': args.seed, 'object': trial['object']['id'],
              'source_trial': str(args.trial.resolve()), 'provider': args.provider,
              'trial_category': trial['category'], 'candidate_ranks': [c['rank'] for c in grasps],
              'video': str(output)}
    output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
