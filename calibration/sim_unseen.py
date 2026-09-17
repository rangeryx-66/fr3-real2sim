"""Support telemetry and unseen assets on the existing calibrated physics plant."""
import os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
# Apply the existing force-control adapter, intercepting its final exec only.
adapter=(ROOT/'calibration/sim_calibration.py').read_text()
end="exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert adapter.count(end)==1
ns={'__file__':str(ROOT/'calibration/sim_calibration.py')}
exec(compile(adapter.replace(end,''),str(ROOT/'calibration/sim_calibration.py'),'exec'),ns)
source=ns['source'];text=ns['text']
changes={
    'import calibration_scene':'import unseen_scene as calibration_scene',
    'clutter=None':"clutter=calibration_scene.ArenaMonitor(world,stage,mat) if not os.environ.get('UNSEEN_NO_CLUTTER') else None",
    'camera.add_distance_to_image_plane_to_frame()':'camera.add_distance_to_image_plane_to_frame()\ncamera.add_instance_id_segmentation_to_frame()',
    '                    results.clear()':'                    results.clear();calib.phase="RESET";support.reset()',
    'mask=(np.abs(xyz-bp)<=SIZE/2+.002).all(axis=1)':'mask=calibration_scene.target_mask(camera,valid)',
    'np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K)':'np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K,rgb=camera.get_rgba())',
    "                elif op=='capture':":"                elif op=='capture':\n                    for _ in range(16):world.step(render=True);tick+=1",
    "calib.phase=cmd['phase'];results[token]={'ok':True}":"calib.phase=cmd['phase']\n                    if clutter:clutter.phase=cmd['phase']\n                    results[token]={'ok':True}",
    "                elif op=='calibration_stability':":"""                elif op=='support_begin':
                    support.begin(cmd['initial_z']);results[token]={'ok':True}
                elif op=='support_pause':
                    support.armed=False;results[token]={'ok':True}
                elif op=='support_query':
                    results[token]={'ok':True,'support':support.query(cmd.get('since_t'))}
                elif op=='support_hold':
                    results[token]={'ok':True,'outcome':support.hold_outcome(cmd['since_t'])}
                elif op=='calibration_stability':""",
    "print('SIM_READY',names,flush=True)":"""from support_aware import SupportMonitor
asset_directory=Path(os.environ['FR3_ASSET_DIRECTORY'])
support=SupportMonitor(np.load(asset_directory/(calibration_scene.TARGET+'_mesh.npz'))['vertices'])
snapshot=Path(os.environ.get('UNSEEN_SCENE_SNAPSHOT',str(ROOT/'results/runtime_scene_snapshots'/f'{calibration_scene.TARGET}_{os.getpid()}.usdc')))
snapshot.parent.mkdir(parents=True,exist_ok=True)
stage.Flatten().Export(str(snapshot))
print('SIM_READY',names,flush=True)""",
    '        calib.observe(bp,bq,tp,tq,f)':"""        calib.observe(bp,bq,tp,tq,f)
        support.sample(tick*DT,bp,bq,tp,tq,f)
        if active and support.armed and calib.phase in ['SUPPORTED_MICRO_LIFT','LIFT'] and support.contact_gap>.05:
            results[active['id']]={'ok':False,'reason':'SUPPORT_CONTACT_LOSS','gap_s':support.contact_gap}
            active=None;target=robot.get_joint_positions().copy()""",
    "if trace.path is not None:trace.records[-1]['hand_calibration']=calib.state()":"""if trace.path is not None:
            trace.records[-1]['hand_calibration']=calib.state()
            trace.records[-1]['mesh_bottom_z_m']=support.last['mesh_bottom_z_m']
            if clutter:
                obp,obq=clutter.contacts.get_world_poses()
                trace.records[-1]['non_target_poses']=dict(positions=obp.tolist(),quaternions_wxyz=obq.tolist())""",
}
for old,new in changes.items():
    assert text.count(old)==1,(old,text.count(old))
    text=text.replace(old,new)
# The original server has no os import. No per-category execution branches exist.
exec(compile('import os\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))
