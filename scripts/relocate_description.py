"""Relocate generated FR3 URDF mesh paths after checking out this project."""
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[1]
p = ROOT / 'config/fr3.urdf'
text = p.read_text()
pattern = r'(?:/[^\" ]+)?/fr3_moveit_grasp/franka_description/'
updated, count = re.subn(pattern, str(ROOT/'franka_description')+'/', text)
if count == 0:
    # A repo already relocated to this directory is fine.
    if str(ROOT/'franka_description') not in text:
        raise SystemExit('No known FR3 mesh paths found; inspect config/fr3.urdf manually')
else:
    p.write_text(updated)
missing = sorted({x for x in re.findall(r'<mesh filename="([^\"]+)"', updated) if not Path(x).exists()})
if missing:
    raise SystemExit(f'{len(missing)} referenced mesh files missing; clone official franka_description first. Example: {missing[0]}')
print(f'FR3 URDF mesh paths valid ({count} replacements)')
