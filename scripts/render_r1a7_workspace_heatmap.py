#!/usr/bin/env python3
"""Render the MoveIt X-Z IK grid as an editable, dependency-free SVG."""
import argparse
import json
from html import escape
from pathlib import Path


def color(value):
    t = value / 11
    start = (245, 248, 252)
    end = (20, 75, 165)
    rgb = tuple(round(a * (1-t) + b * t) for a, b in zip(start, end))
    return '#%02x%02x%02x' % rgb


def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    data = json.loads(a.input.read_text())
    xs = data['x_m']
    zs = data['z_m']
    counts = data['per_cell_11_orientation_successes']
    cell = 48
    left, top = 82, 86
    width = left + len(xs)*cell + 70
    height = top + len(zs)*cell + 110
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
           'font-family="Arial, sans-serif">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<text x="24" y="34" font-size="22" font-weight="700">R1-7a exact 6D IK: frozen AnyGrasp orientations</text>',
           '<text x="24" y="57" font-size="13" fill="#475569">Current base (0, 0, 0; +90°). Each cell: solved orientations / 11.</text>']
    for row_index, z in enumerate(reversed(zs)):
        y = top + row_index*cell
        actual_index = len(zs)-1-row_index
        svg.append(f'<text x="70" y="{y+30}" text-anchor="end" font-size="12" fill="#334155">{z:.3f}</text>')
        for col_index, x in enumerate(xs):
            value = counts[actual_index][col_index]
            xx = left + col_index*cell
            svg.append(f'<rect x="{xx}" y="{y}" width="{cell}" height="{cell}" '
                       f'fill="{color(value)}" stroke="#cbd5e1"/>')
            ink = 'white' if value >= 6 else '#172554'
            svg.append(f'<text x="{xx+cell/2}" y="{y+30}" text-anchor="middle" '
                       f'font-size="15" font-weight="700" fill="{ink}">{value}</text>')
    for i, x in enumerate(xs):
        xx = left + i*cell + cell/2
        svg.append(f'<text x="{xx}" y="{top+len(zs)*cell+22}" text-anchor="middle" '
                   f'font-size="11" fill="#334155">{x:.3f}</text>')
    svg += [f'<text x="{left+len(xs)*cell/2}" y="{height-36}" text-anchor="middle" '
            'font-size="15" fill="#0f172a">World X (m)</text>',
            f'<text x="24" y="{top+len(zs)*cell/2}" text-anchor="middle" '
            'font-size="15" fill="#0f172a" transform="rotate(-90 24 '
            f'{top+len(zs)*cell/2})">World Z (m)</text>',
            f'<text x="{left}" y="{height-10}" font-size="11" fill="#64748b">'
            + escape('Grid translations preserve all 11 grasp rotations; one home seed per MoveIt query.')
            + '</text>', '</svg>']
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text('\n'.join(svg) + '\n')


if __name__ == '__main__':
    main()
