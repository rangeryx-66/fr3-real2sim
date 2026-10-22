"""Scoped mobile-mount assembly around byte-frozen V8 physical owners."""
import json
from pathlib import Path
from unittest.mock import patch

from .mobile_mount import author,drive_class


def runtime_class(original):
    class MobileRuntime(original):
        def __init__(self,job):
            import xarm_ft_v7_scene as plant
            import xarm7_native_gripper as gripper
            original_author=plant.author
            def assemble(stage,g,case,output):
                audit=original_author(stage,g,case,output)
                author(stage,output)
                # This object was wrapped before the mount existed and cached
                # /Geometry/world as its tensor root. Reinitialize the SAME
                # scene-registered wrapper before the first world.reset(),
                # without position/orientation or xform reset. All caller and
                # Scene references retain the one explicit robot object.
                from isaacsim.core.prims import SingleArticulation
                SingleArticulation.__init__(g['robot'],prim_path='/World/Piper',
                    name='piper',reset_xform_properties=False)
                return audit
            # Explicit constructor-only bindings: never hot-swap an active actor.
            # NativeDrive still owns every arm/G1 command and its exact caps.
            with patch.object(plant,'author',assemble),patch.object(gripper,'NativeDrive',drive_class(gripper.NativeDrive)):
                super().__init__(job)
            import numpy as np
            from isaacsim.core.prims import RigidPrim
            self.mount=json.loads((self.output/'V81_MOBILE_MOUNT.json').read_text())
            self.base_indices=np.array([self.names.index(n) for n in self.mount['joint_names']])
            self.base_view=RigidPrim(prim_paths_expr=self.mount['base_body'],name='v81_measured_base',reset_xform_properties=False)
            self.base_view.initialize()
            self.initial_base=self.base.copy()
            self.base_goal=np.asarray(self.robot.get_joint_positions())[self.base_indices].copy()
            self.base_velocity=np.zeros(2)
            self.base_active=False
            self.base_trace=[]
            self.base_generation=0
            self.read_base()
            # Passive object law remains the existing owner. Base actuation is
            # separate and cannot submit object or arm/G1 joint commands.
            passive=self.engineering_before_physics
            def actuate():
                if passive is not None:passive()
                from isaacsim.core.utils.types import ArticulationAction
                self.controller.raw.apply_action(ArticulationAction(joint_positions=self.base_goal,
                    joint_velocities=self.base_velocity,joint_indices=self.base_indices))
            self.engineering_before_physics=actuate
            self.record.metadata['v81_base']=dict(joint_indices=self.base_indices.tolist(),joint_names=self.mount['joint_names'],
                moving_only_when_verified_unloaded=True,kind='simulated driven XY carriage; not wheels')

        def read_base(self):
            import numpy as np
            from scipy.spatial.transform import Rotation
            p,q=self.base_view.get_world_poses();T=self.matrix(p[0],q[0])
            B=T@np.linalg.inv(self.model.chain.root_to_link('link_base',{}))
            self.base[:3]=B[:3,3]
            self.base[3]=Rotation.from_matrix(B[:3,:3]).as_euler('xyz',degrees=True)[2]
            self.g['base']=self.base
            return B

        def enrich_measurement(self,row,pre_contact_poses):
            if hasattr(self,'base_view'):
                B=self.read_base()
                row['measured_base_xyzyaw']=self.base.copy()
                if self.base_active:
                    self.base_trace.append(dict(t=row['t'],base=self.base.tolist(),
                        commanded_joint_positions_m=self.base_goal.tolist(),
                        measured_joint_positions_m=row['q'][self.base_indices].tolist()))
            return super().enrich_measurement(row,pre_contact_poses)

        def close(self):
            (self.output/'V81_BASE_TRACE.json').write_text(json.dumps(self.base_trace))
            return super().close()
    MobileRuntime.__name__='Mobile'+original.__name__
    return MobileRuntime
