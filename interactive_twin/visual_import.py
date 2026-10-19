"""Make a visual-only URDF copy for the Isaac OBJ/UV conversion failure.

Source files are immutable. Geometry vertex/face indices, normals, visual
transforms, collisions, joints and inertials are preserved. Texture coordinates
are omitted in the derived visuals; this is an untextured diagnostic fallback,
not a new object or an interaction geometry change.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def without_uv(text):
    output = []
    for line in text.splitlines():
        if line.startswith('vt '):
            continue
        if line.startswith('f '):
            tokens = []
            for token in line.split()[1:]:
                parts = token.split('/')
                tokens.append(parts[0] + ('//' + parts[2] if len(parts) > 2 and parts[2] else ''))
            line = 'f ' + ' '.join(tokens)
        output.append(line)
    return '\n'.join(output) + '\n'


def geometry_signature(text):
    """Exact vertex/normal records and face geometry; no floating point rounding."""
    vertices, normals, faces = [], [], []
    for line in text.splitlines():
        if line.startswith('v '):
            vertices.append(line)
        elif line.startswith('vn '):
            normals.append(line)
        elif line.startswith('f '):
            faces.append([(p.split('/')[0], p.split('/')[2] if len(p.split('/')) > 2 else '')
                          for p in line.split()[1:]])
    return vertices, normals, faces


def repair(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Use a fresh output directory; existing evidence is never overwritten.')
    output.mkdir(parents=True)
    original = ET.parse(source)
    tree = ET.parse(source)
    rows = []
    for link in tree.getroot().findall('link'):
        for index, visual in enumerate(link.findall('visual')):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            src = (source.parent / mesh.get('filename')).resolve()
            if src.suffix.lower() != '.obj':
                mesh.set('filename', str(src))
                continue
            text = src.read_text()
            fixed = without_uv(text)
            assert geometry_signature(text) == geometry_signature(fixed)
            dst = output / 'visuals' / (link.get('name') + '_' + str(index) + '_' + src.name)
            dst.parent.mkdir(exist_ok=True)
            # Texture images cannot be sampled without UVs. Keep material colors
            # while removing only texture-map directives from derived materials.
            lines = fixed.splitlines()
            for i, line in enumerate(lines):
                if line.startswith('mtllib '):
                    names = []
                    for j, name in enumerate(line.split()[1:]):
                        material = (src.parent / name).resolve()
                        if not material.is_file():
                            continue
                        target = dst.with_name(dst.stem + '_' + str(j) + '.mtl')
                        target.write_text('\n'.join(s for s in material.read_text().splitlines()
                                                    if not s.lstrip().lower().startswith(('map_', 'bump ', 'disp ', 'decal '))) + '\n')
                        names.append(target.name)
                    lines[i] = 'mtllib ' + ' '.join(names) if names else '# material file unavailable'
            dst.write_text('\n'.join(lines) + '\n')
            assert geometry_signature(text) == geometry_signature(dst.read_text())
            mesh.set('filename', str(dst))
            visual.set('name', link.get('name') + '_visual_' + str(index))
            rows.append({'link': link.get('name'), 'source': str(src), 'source_sha256': sha(src),
                         'derived': str(dst), 'derived_sha256': sha(dst),
                         'geometry_and_normals_exactly_preserved': True})
    # Relative collision paths must resolve to the exact existing files.
    collision_rows = []
    for mesh in tree.getroot().findall('./link/collision/geometry/mesh'):
        path = (source.parent / mesh.get('filename')).resolve()
        mesh.set('filename', str(path))
        collision_rows.append({'path': str(path), 'sha256': sha(path)})
    # Verify every physics subtree after normalizing only file resolution.
    original_physics = copy.deepcopy(original.getroot())
    derived_physics = copy.deepcopy(tree.getroot())
    for root in (original_physics, derived_physics):
        for link in root.findall('link'):
            for visual in list(link.findall('visual')):
                link.remove(visual)
        for mesh in root.findall('./link/collision/geometry/mesh'):
            mesh.set('filename', str((source.parent / mesh.get('filename')).resolve()))
    assert ET.tostring(original_physics) == ET.tostring(derived_physics)
    destination = output / source.name
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    audit = {'source_urdf': str(source), 'source_urdf_sha256': sha(source),
             'derived_urdf': str(destination), 'derived_urdf_sha256': sha(destination),
             'purpose': 'visual-only untextured Isaac converter compatibility copy',
             'physics_unchanged': True, 'texture_fidelity_preserved': False,
             'visuals': rows, 'collision_files': collision_rows}
    (output / 'visual_import_audit.json').write_text(json.dumps(audit, indent=2))
    return audit


def compatible_urdf(source, cache_root):
    """Cache a source-immutable, visual-only converter compatibility copy."""
    source = Path(source).resolve()
    digest = hashlib.sha256(source.read_bytes() + Path(__file__).read_bytes()).hexdigest()
    output = Path(cache_root) / digest[:16]
    audit_path = output / 'visual_import_audit.json'
    if audit_path.is_file():
        audit = json.loads(audit_path.read_text())
        if audit['source_urdf_sha256'] != sha(source):
            raise RuntimeError('VISUAL_COMPATIBILITY_SOURCE_CHANGED')
        for row in audit['visuals']:
            if sha(Path(row['source'])) != row['source_sha256'] or sha(Path(row['derived'])) != row['derived_sha256']:
                raise RuntimeError('VISUAL_COMPATIBILITY_MESH_CHANGED')
        for row in audit['collision_files']:
            if sha(Path(row['path'])) != row['sha256']:
                raise RuntimeError('VISUAL_COMPATIBILITY_COLLISION_CHANGED')
        if sha(Path(audit['derived_urdf'])) != audit['derived_urdf_sha256']:
            raise RuntimeError('VISUAL_COMPATIBILITY_URDF_CHANGED')
    else:
        audit = repair(source, output)
    return Path(audit['derived_urdf']), audit
