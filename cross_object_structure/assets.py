"""Outcome-independent selection from the already downloaded dataset archive."""
import concurrent.futures
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def write(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_one(args):
    row, source, out = args
    target = Path(out)/'prepared'/row['asset_id']/row['asset_id']
    audit = Path(out)/'preparation'/f"{row['asset_id']}.json"
    if audit.exists():
        return json.loads(audit.read_text())
    result = dict(row, asset_root=str(target), geometry_only=True)
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, str(ROOT/'scripts/prepare_physx_microwave.py'),
               '--source', str(source), '--asset-id', row['asset_id'],
               '--height-m', str(row['height_m']), '--output', str(target)]
        env = dict(os.environ); env.pop('PYTHONPATH', None)
        with (target.parent/'prepare.log').open('w') as log:
            subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                           check=True, timeout=180)
        meta = json.loads((target/'manifest.json').read_text())
        meta.update(category=row['category'], object_name=row['object_name'],
                    source_urdf=str(Path(source)/'urdf'/f"{row['asset_id']}.urdf"),
                    dimension_source='dataset finaljson dimension, third component in cm; frozen existing preparation rule')
        write(target/'manifest.json', meta)
        spec = importlib.util.spec_from_file_location('frozen_ranker', ROOT/'scripts/rank_prepared_handle_assets.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        ranked = module.scan(target.parent)
        from interactive_twin.manifest import _handle, DEFAULT_POLICY
        usable = [r for r in ranked['ranking'] if not _handle(r, DEFAULT_POLICY)['geometric_exclusion_reasons']]
        result.update(status='GEOMETRY_ELIGIBLE' if usable else 'NO_SUPPORTED_VISUAL_BAR_HANDLE',
                      ranking=ranked['ranking'], valid_handles=usable,
                      prepared_manifest_sha256=sha(target/'manifest.json'))
    except Exception as error:
        result.update(status='PREPARATION_FAILED', error=str(error), valid_handles=[])
    write(audit, result)
    return result


def prepare(config, output):
    """Freeze a category-balanced source pool before geometry processing or execution."""
    out = Path(output); policy = config['assets']; file = out/'source_pool.json'
    archive = Path(policy['archive']); excluded = set(policy['previously_prepared_ids'])
    with zipfile.ZipFile(archive) as z:
        if file.exists():
            pool = json.loads(file.read_text())
            if pool['policy'] != policy or pool['archive_sha256'] != sha(archive):
                raise RuntimeError('FROZEN_SOURCE_POOL_CHANGED')
        else:
            groups = defaultdict(list); inventory = []
            for name in sorted(z.namelist()):
                if not name.endswith('.urdf'): continue
                aid = Path(name).stem
                joints = [j for j in ET.fromstring(z.read(name)).findall('joint') if j.get('type') != 'fixed']
                metadata = f'PhysX_mobility/finaljson/{aid}.json'
                try: meta = json.loads(z.read(metadata))
                except (KeyError, ValueError): continue
                label = meta.get('object_name', '')
                row = {'asset_id':aid, 'category':meta.get('category','unknown'),
                       'object_name':label, 'source_urdf':name, 'source_urdf_sha256':hashlib.sha256(z.read(name)).hexdigest(),
                       'dimension':meta.get('dimension'), 'exclusion_reasons':[]}
                if aid in excluded: row['exclusion_reasons'].append('PREVIOUSLY_PREPARED_OR_EXPOSED')
                if len(joints)!=1 or joints[0].get('type')!='revolute': row['exclusion_reasons'].append('NOT_NATIVE_SINGLE_REVOLUTE_OPENING')
                if not any(x in label.casefold() for x in policy['opening_name_keywords']): row['exclusion_reasons'].append('OUTSIDE_OPENING_CATEGORY')
                dim = re.findall(r'\d+(?:\.\d+)?', str(meta.get('dimension','')))
                if len(dim)!=3 or float(dim[-1])<=0: row['exclusion_reasons'].append('INVALID_DATASET_DIMENSION')
                else: row['height_m']=float(dim[-1])/100.
                inventory.append(row)
                if not row['exclusion_reasons']: groups[(row['category'],label)].append(row)
            selected=[]
            for key in sorted(groups):
                groups[key].sort(key=lambda r:hashlib.sha256(f"{policy['seed']}:{r['asset_id']}".encode()).hexdigest())
            for i in range(policy['source_pool_per_category']):
                for key in sorted(groups):
                    if i<len(groups[key]): selected.append(groups[key][i])
            selected=selected[:policy['maximum_source_preparations']]
            pool={'policy':policy,'archive_sha256':sha(archive),'inventory':inventory,'selected':selected,
                  'selected_before_geometry_or_execution':True,'physical_outcomes_used':False}
            write(file,pool)
        source=out/'extracted'/'PhysX_mobility'
        wanted={r['asset_id'] for r in pool['selected']}
        for name in z.namelist():
            parts=Path(name).parts
            if len(parts)>=3 and ((parts[1] in ('urdf','finaljson') and Path(name).stem in wanted)
                                  or (parts[1]=='partseg' and parts[2] in wanted)):
                # All members must stay inside the dedicated new extraction root.
                dest=(out/'extracted'/name).resolve()
                if not dest.is_relative_to((out/'extracted').resolve()): raise RuntimeError('UNSAFE_ARCHIVE_PATH')
                if not dest.exists(): z.extract(name,out/'extracted')
    with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool_exec:
        rows=list(pool_exec.map(prepare_one,[(r,str(source),str(out)) for r in pool['selected']]))
    write(out/'preparation_inventory.json',{'assets':rows,'physical_outcomes_used':False})
    ranking=[r for a in rows for r in a.get('ranking',[])]
    write(out/'ranking.json',{'ranking':ranking,'geometry_only':True})
    return rows
