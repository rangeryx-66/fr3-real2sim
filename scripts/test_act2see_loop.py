"""Regression checks for the EE-only loop's information and force boundaries."""
from pathlib import Path
import sys,tempfile
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_identification.act2see_loop import InteractionMemory,ConstrainedDrive,TactileRetention


def run():
 with tempfile.TemporaryDirectory() as tmp:
  T=np.eye(4);T[:3,3]=[.3,0,.2];memory=InteractionMemory(T,Path(tmp))
  memory.begin_attempt(0,0.)
  for x in np.linspace(0,.000055,20):
   P=T.copy();P[0,3]+=x;memory.observe(P)
  assert memory.try_fit() is None and memory.final_fit()['joint_type']=='UNOBSERVABLE'
  memory.finish_attempt(1.,'SAFE_LOW_EXCITATION');assert len(memory.attempts)==1
  memory.begin_attempt(1,2.);assert len(memory.attempts)==2
  memory=InteractionMemory(T,Path(tmp))
  for angle in np.linspace(0,.08,100):
   P=T.copy();P[:3,:3]=Rotation.from_rotvec([0,0,angle]).as_matrix();P[:3,3]=P[:3,:3]@T[:3,3];memory.observe(P)
  assert memory.try_fit()['joint_type']=='revolute' and memory.estimate is not None
  assert np.allclose(np.abs(memory.estimate['revolute']['axis']),[0,0,1],atol=1e-7)
  assert abs(memory.estimate['revolute']['radius_m']-.3)<1e-6
  assert abs(memory.angle_deg(P)-np.rad2deg(.08))<1e-6
  assert (Path(tmp)/'estimated_articulation.urdf').exists()
  assert memory.consistency_error(P)<1e-8
  slipped=P.copy();slipped[2,3]+=.002
  assert memory.consistency_error(slipped)>.0019
 T=np.eye(4);drive=ConstrainedDrive(T,[1,0,0]);J=np.eye(6);M=np.eye(6);zero=np.zeros(6)
 changed=T.copy();changed[1,3]=.01;changed[:3,:3]=Rotation.from_rotvec([0,.2,0]).as_matrix()
 for _ in range(300):tau=drive.command(changed,J,M,zero,zero,zero,1/240)
 assert abs(tau[1])<1e-12 and np.linalg.norm(tau[3:])<1e-12 # no hidden pose spring
 assert 0<tau[0]<=drive.force_cap_n
 for speed in [1.,10.,100.]:
  drive.command(T,J,M,np.ones(6)*speed,zero,zero,1/240)
  wrench=np.array(drive.last['command_wrench_world'])
  assert np.linalg.norm(wrench[:3])<=drive.force_cap_n+1e-10
  assert np.linalg.norm(wrench[3:])<=drive.moment_cap_nm+1e-10
 elapsed=drive.elapsed;old_lead=drive.direction@(drive.reference-T[:3,3])
 drive.refresh_tangent([1,.1,0],T)
 assert drive.elapsed==elapsed and abs(drive.direction@(drive.reference-T[:3,3])-np.clip(old_lead,-.002,.002))<1e-12
 sensor=TactileRetention(['a','b'])
 contacts=[{'finger':f,'allowed_pad_target':True,'force_n':.5 if i==0 else 0.,'contact_point_world_m':[x,y,z]}
           for f,y in [('a',.01),('b',-.01)] for i,(x,z) in enumerate([(-.01,-.01),(-.01,.01),(.01,-.01),(.01,.01)])]
 sensor.observe(0,T,contacts);sensor.arm();shift=T.copy();shift[0,3]=.01
 moved=[{**c,'contact_point_world_m':(np.array(c['contact_point_world_m'])+[.01,0,0]).tolist()} for c in contacts]
 sensor.observe(.2,shift,moved);assert sensor.drift_m<1e-12
 redistributed=[{**c,'force_n':.5 if i%4==3 else 0.} for i,c in enumerate(moved)]
 sensor.observe(.4,shift,redistributed);assert sensor.drift_m<1e-12
 # A pressure-center jump must not become a fabricated 20 mm slip.
 shifted_normal=T.copy();shifted_normal[1,3]=.002
 sensor.observe(.6,shifted_normal,contacts);assert sensor.drift_m>.0019
 # Explicitly document the unobservable tangential direction.
 result=sensor.observe(.8,shift,contacts);assert sensor.drift_m<1e-12 and result['unobservable_slip_components']
 print('PASS: short-motion guard, attempt memory, EE-only screw fit, free orthogonal DOFs, wrench bounds, tactile frame invariance')

if __name__=='__main__':run()
