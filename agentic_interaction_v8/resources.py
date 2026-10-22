"""Explicit, hash-checked IO relocation. No control or geometry algorithms."""
import hashlib
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n')


def inventory(root):
    root = Path(root)
    return {str(p.relative_to(root)): digest(p) for p in sorted(root.rglob('*'))
            if p.is_file() and p.name != 'BUNDLE.json'}


def verify_bundle(root):
    root = Path(root).resolve(); manifest = json.loads((root/'BUNDLE.json').read_text())
    for name, expected in manifest['files'].items():
        p = (root/name).resolve()
        if not p.is_relative_to(root) or digest(p) != expected:
            raise ValueError('RESOURCE_DRIFT:' + name)
    return manifest


def semantic_urdf(path):
    """Compare every non-path XML value; mesh paths replaced by content digest."""
    p = Path(path); root = ET.parse(p).getroot()
    for mesh in root.iter('mesh'):
        f = Path(mesh.attrib['filename'])
        if not f.is_absolute(): f = p.parent/f
        mesh.set('filename', 'sha256:' + digest(f))
    return ET.tostring(root)


def export_bundle(repo, descriptor, output):
    """One-time migration from explicitly declared execution inputs, never at run time.

    Original bytes retained. A minimal station has placement/base only; successful
    grasps, observations, USD caches and physical outcomes are never imported.
    """
    repo, output = Path(repo), Path(output).resolve()
    if output.exists(): raise ValueError('NEW_BUNDLE_DIRECTORY_REQUIRED')
    output.mkdir(parents=True)
    mapping = {}; original = {}
    def copy(src, rel):
        src = Path(src).resolve(); target = output/rel
        if not src.is_file(): raise ValueError('MISSING_RESOURCE:' + str(src))
        target.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(src, target)
        mapping[str(src)] = str(rel); original[str(src)] = digest(src)
        return str(rel)
    native = Path(descriptor['native_configuration']).resolve()
    cfg = json.loads(native.read_text()); copy(native, 'original/robot/NATIVE_CONFIG.json')
    robot = {}
    for key in ['sim_urdf', 'dynamics_urdf']:
        p = Path(cfg[key]); robot[key] = copy(p, 'original/robot/'+key+'.urdf')
        for m in ET.parse(p).getroot().iter('mesh'):
            f = Path(m.get('filename')); f = f if f.is_absolute() else p.parent/f
            f = f.resolve()
            if str(f) not in mapping: copy(f, 'meshes/'+digest(f)[:16]+'_'+f.name)
    if cfg.get('gain_reference') and Path(cfg['gain_reference']).is_file():
        copy(cfg['gain_reference'], 'original/robot/gain_reference.yaml')
    # All 22 historical external commitments preserved verbatim, independently
    # from the URDF/configuration IO relocation.
    external = json.loads((repo/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_EXTERNAL_FREEZE.json').read_text())
    frozen = external['external_robot_camera_closure_uncertainty_sha256']
    relocated = {}
    for i, (src, expected) in enumerate(frozen.items()):
        if digest(src) != expected: raise ValueError('FROZEN_EXTERNAL_DRIFT:' + src)
        rel = mapping.get(str(Path(src).resolve())) or copy(src, f'original/external/{i:02d}_'+Path(src).name)
        relocated[src] = rel
    resources = {}
    for key in ['active_closure_source', 'robust_uncertainty_config', 'wrist_calibration', 'camera_calibration']:
        resources[key] = copy(descriptor[key], 'original/calibration/'+key+Path(descriptor[key]).suffix)
    dump(output/'original/calibration/ft_calibration.json', descriptor['ft_calibration'])
    resources['ft_calibration'] = 'original/calibration/ft_calibration.json'
    scenes = {}
    for scene_id, row in descriptor['scenes'].items():
        src = Path(row['asset_root']).resolve(); dest = output/'scenes'/scene_id/'asset'
        # Library asset directories may be symlinks. Resolve their contents,
        # never copy a link back into the original developer checkout.
        files=[];visited=set()
        for base,dirs,names in os.walk(src,followlinks=True):
            actual=Path(base).resolve()
            if actual in visited: dirs[:]=[];continue
            visited.add(actual)
            dirs[:]=[x for x in dirs if x not in ('usd','visual_compatibility','__pycache__')]
            files.extend(Path(base)/name for name in names)
        for p in sorted(files):
            if not p.is_file() or any(x in ('usd','visual_compatibility','__pycache__') for x in p.relative_to(src).parts): continue
            if p.suffix.lower() not in ('.urdf','.obj','.stl','.dae','.mtl','.png','.jpg','.jpeg','.ply') and p.name != 'manifest.json': continue
            copy(p, str(dest.relative_to(output)/p.relative_to(src)))
        source = json.loads(Path(row['station_report']).read_text())
        station = {k: source[k] for k in ('asset_installation', 'robot_base_pose')}
        dump(output/'scenes'/scene_id/'station/report.json', station)
        template = dict(row['job_template'])
        for k in ('source','asset_root','output','private_physics_configuration','wrist_experiment',
                  'camera_calibration','active_closure_source','robust_uncertainty_config','wrist_calibration','ft_calibration'):
            template.pop(k, None)
        for k in ('passive_geometry_cameras','passive_dataset_output','passive_evaluation_output',
                  'benchmark_registration','registered_episode_id','implementation_commit'):
            template.pop(k, None)
        dump(output/'scenes'/scene_id/'template.json', template)
        scenes[scene_id] = {'asset':'scenes/'+scene_id+'/asset', 'station':'scenes/'+scene_id+'/station',
                            'template':'scenes/'+scene_id+'/template.json'}
        manifest=json.loads((dest/'manifest.json').read_text())
        urdf=dest/'urdf'/(str(manifest['asset_id'])+'.urdf')
        if not urdf.is_file():raise ValueError('MISSING_ASSET_URDF:'+scene_id)
        for mesh in ET.parse(urdf).getroot().iter('mesh'):
            filename=Path(mesh.get('filename'))
            if filename.is_absolute():raise ValueError('ABSOLUTE_ASSET_MESH_REQUIRES_EXPLICIT_RELOCATION:'+str(filename))
            if not (urdf.parent/filename).is_file():raise ValueError('MISSING_ASSET_MESH:'+str(filename))
    manifest = dict(version='8.0.0', role='explicit private assets/calibration; no observations or saved grasps',
                    original_sha256=original, original_to_bundle=mapping,
                    frozen_external_to_bundle=relocated, robot=robot, resources=resources, scenes=scenes,
                    legacy_result_roots=sorted({x.split('/results/')[0]+'/results/' for x in original if '/results/'in x}
                        | {str(x).split('/results/')[0]+'/results/'for x in descriptor.values()if isinstance(x,str)and '/results/'in x}
                        | {str(row['asset_root']).split('/results/')[0]+'/results/'for row in descriptor['scenes'].values()if '/results/'in str(row['asset_root'])}),
                    files=inventory(output))
    dump(output/'BUNDLE.json', manifest)
    return manifest


def materialize(bundle, destination):
    """Relocate filenames only; original XML numeric/topology/materials unchanged."""
    bundle, destination = Path(bundle).resolve(), Path(destination).resolve()
    m = verify_bundle(bundle)
    if destination.exists(): raise ValueError('FRESH_MATERIALIZATION_REQUIRED')
    destination.mkdir(parents=True)
    cfg = json.loads((bundle/'original/robot/NATIVE_CONFIG.json').read_text())
    equivalence = {}
    for key, rel in m['robot'].items():
        source = bundle/rel; tree = ET.parse(source)
        for mesh in tree.getroot().iter('mesh'):
            old = mesh.get('filename')
            if old not in m['original_to_bundle']: raise ValueError('UNDECLARED_MESH:' + old)
            mesh.set('filename', str(bundle/m['original_to_bundle'][old]))
        target = destination/(key+'.urdf'); tree.write(target, encoding='utf-8', xml_declaration=True)
        # Original mesh files may be absent on a new machine. Normalize both
        # trees using the frozen source->content mapping instead of reading them.
        original = ET.parse(source).getroot()
        for mesh in original.iter('mesh'):
            mesh.set('filename', 'sha256:' + m['original_sha256'][mesh.get('filename')])
        if ET.tostring(original) != semantic_urdf(target): raise ValueError('ROBOT_SEMANTIC_DRIFT')
        equivalence[key] = {'original_sha256':digest(source), 'relocated_sha256':digest(target),
                            'non_path_xml_and_mesh_bytes_equal':True}
        cfg[key] = str(target)
    cfg['urdf'] = cfg['sim_urdf']
    if 'original/robot/gain_reference.yaml' in m['files']:
        cfg['gain_reference'] = str(bundle/'original/robot/gain_reference.yaml')
    dump(destination/'NATIVE_CONFIG.json', cfg); dump(destination/'IO_EQUIVALENCE.json', equivalence)
    return equivalence
