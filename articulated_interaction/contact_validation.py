"""Ownership envelope gate and raw-backed non-pad event classification.

No global penetration allowance. Numeric bounds only account for float32 USD
coordinates. Sustained non-pad impulses remain forbidden even with a raw gap.
"""
import json
from pathlib import Path
import numpy as np
import trimesh
import fcl
from scipy.spatial import ConvexHull
from piper_mobile_demo.owned_scene import Shape, meshes


def audit_envelope(root, export, manifest):
    records={}
    for name,spec in manifest['fingers'].items():
        raw=trimesh.load(Path(root)/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh')
        reference_vertices=np.asarray(spec.get('source_collision_envelope_vertices',raw.vertices))
        reference=ConvexHull(reference_vertices)
        solids=[];deltas=[];piece_planes=[]
        for e in export['shapes']:
            if e.get('finger')!=name:continue
            relative=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform'])
            cooked=trimesh.util.concatenate(meshes(e));cooked.apply_transform(relative)
            authored=trimesh.util.concatenate(meshes(e,False));authored.apply_transform(relative)
            planes=ConvexHull(authored.vertices).equations
            deltas.append(float(np.max(cooked.vertices@planes[:,:3].T+planes[:,3])))
            solids.append(cooked);piece_planes.append(ConvexHull(cooked.vertices).equations)
        vertices=np.vstack([m.vertices for m in solids]);directions=reference.equations[:,:3]
        expected=np.max(reference_vertices@directions.T,axis=0)
        observed=np.max(vertices@directions.T,axis=0)
        # Floating-point representation bound, not a physical contact allowance.
        numeric_bound=float(64*np.finfo(np.float32).eps*np.max(np.abs(raw.vertices)))
        outer=float(np.max(vertices@reference.equations[:,:3].T+reference.equations[:,3]))
        support=float(np.max(np.abs(expected-observed)))
        samples=[]
        for face in reference.simplices:
            for i in range(11):
                for j in range(11-i):samples.append((i*reference_vertices[face[0]]+j*reference_vertices[face[1]]+(10-i-j)*reference_vertices[face[2]])/10)
        samples=np.asarray(samples)
        coverage=float(np.max(np.min([np.max(samples@eq[:,:3].T+eq[:,3],axis=1) for eq in piece_planes],axis=0)))
        record={'max_uncovered_exterior_sample_m':coverage,'max_piece_outward_error_m':max(deltas),'max_union_outward_error_m':outer,'max_union_support_error_m':support,'float32_coordinate_bound_m':numeric_bound,'reference':'original unsplit native finger collision envelope' if 'source_collision_envelope_vertices' in spec else 'official raw convex envelope','source_hull_volume_m3':float(reference.volume),'cooked_piece_volume_sum_m3':float(sum(abs(m.volume) for m in solids)),'piece_count':len(solids)}
        record['passed']=max(max(deltas),outer,support,coverage)<=numeric_bound
        records[name]=record
    return {'fingers':records,'passed':all(r['passed'] for r in records.values()),'rule':'preserve authored partition and original hull envelope within float32 representation precision; no physical penetration allowance'}


class NonpadClassifier:
    def __init__(self,root,export,manifest,allowed):
        self.entries={e['path']:e for e in export['shapes']}
        self.raw={};self.consecutive={};self.events=[];self.allowed=set(allowed)
        for name,spec in manifest['fingers'].items():
            m=trimesh.load(Path(root)/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh')
            m.update_faces(np.array([i not in set(spec['official_pad_face_ids']) for i in range(len(m.faces))]));m.remove_unreferenced_vertices()
            self.raw[name]=Shape(m,np.eye(4),False,'official_nonpad_'+name,name)

    def classify(self,contacts,finger_poses,target_pose,dt):
        result=[];seen=set()
        for c in contacts:
            if c['owner']=='pad' or c['force_n']<=0:continue
            key=(c['collider'],c['target']);seen.add(key)
            self.consecutive[key]=self.consecutive.get(key,0.)+dt
            e=self.entries.get(c['target']);source=self.raw.get(c['finger'])
            if source is None or e is None or c['target'] not in self.allowed:
                row={'classification':'CONFIRMED_FORBIDDEN_UNKNOWN_OR_NONHANDLE_CONTACT','contact':c};result.append(row);continue
            relative=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform'])
            source.place(np.asarray(finger_poses[c['finger']]))
            target=Shape(trimesh.util.concatenate(meshes(e)),relative,True,e['path'],'handle');target.place(target_pose)
            hit=fcl.collide(source.object,target.object)>0
            distance=fcl.distance(source.object,target.object) if not hit else 0.
            active=self.entries[c['collider']]
            authored=trimesh.util.concatenate(meshes(active,False));cooked=trimesh.util.concatenate(meshes(active))
            local_to_body=np.linalg.inv(np.asarray(active['rigid_body_world_transform']))@np.asarray(active['world_transform'])
            authored.apply_transform(local_to_body);cooked.apply_transform(local_to_body)
            planes=ConvexHull(authored.vertices).equations
            protrusion=max(0.,float(np.max(cooked.vertices@planes[:,:3].T+planes[:,3])))
            offsets=c.get('separation_m')
            # Shell contacts may carry speculative solver impulse before overlap.
            # A persistent load is never waived by the representation diagnosis.
            sustained=self.consecutive[key]>=.05
            offset_a=active['offsets']['contactOffset']['schema_value']
            offset_b=e.get('runtime_body_shapes',{}).get('contact_offsets',[])
            offset_b=np.asarray(offset_b,dtype=float).ravel()
            shell=float(offset_a)+float(np.max(offset_b)) if offset_a is not None and offset_b.size else 0.
            explained=distance>0 and distance<=shell+protrusion
            kind='CONFIRMED_FORBIDDEN_RAW_CONTACT' if hit or distance<=0 else ('CONFIRMED_FORBIDDEN_SUSTAINED_LOAD' if sustained else ('PROXIMITY_OR_REPRESENTATION_WARNING' if explained else 'CONFIRMED_FORBIDDEN_UNEXPLAINED_IMPULSE'))
            row={'classification':kind,'raw_nonpad_gap_m':float(distance),'raw_intersection':hit,'cooked_piece_outward_error_m':protrusion,'native_separation_m':offsets,'contact_shell_upper_bound_m':shell,'raw_gap_explained_by_cooking_or_shell':explained,'consecutive_loaded_time_s':self.consecutive[key],'contact':c}
            result.append(row)
        for key in list(self.consecutive):
            if key not in seen:self.consecutive[key]=0.
        self.events.extend(result)
        return {'events':result,'confirmed_forbidden':any(r['classification'].startswith('CONFIRMED_FORBIDDEN') for r in result),'warnings':sum(r['classification']=='PROXIMITY_OR_REPRESENTATION_WARNING' for r in result)}
