"""Current-station collision wrapper with the original cooked support solid."""
import copy
import numpy as np
from articulated_interaction.physical_baseline import PhysicalScene
from piper_mobile_demo.owned_scene import Shape,meshes,intersects


class MobileScene:
    def __init__(self,export,model,allowed,initial_base):
        self.model=model;self.initial_base=np.asarray(initial_base).copy()
        self.initial_root=model.poses(model.home,self.initial_base)['link_base']
        e=copy.deepcopy(export)
        support=[s for s in e['shapes'] if 'r1a7_pedestal' in s['path']]
        if not support:raise ValueError('NATIVE_SUPPORT_COLLIDER_REQUIRED')
        e['shapes']=[s for s in e['shapes'] if s not in support]
        self.inner=PhysicalScene(e,model,allowed)
        self.support=[]
        for s in support:
            for mesh in meshes(s):
                self.support.append(Shape(mesh,np.asarray(s['world_transform']),True,s['path'],'r1a7_pedestal'))
        self.entries={s['path']:s for s in export['shapes']}

    def __getattr__(self,name):return getattr(self.inner,name)

    def check(self,poses,moving_pose,allow_handle):
        valid,why=self.inner.check(poses,moving_pose,allow_handle)
        if not valid:return valid,why
        delta=np.asarray(poses['link_base'])@np.linalg.inv(self.initial_root)
        for a in self.support:
            a.place(delta)
            for robot in self.inner.robot:
                # The original mount's mechanically adjacent root/first link.
                # No finger, arm, panel or cabinet collision is waived.
                if robot.body in ('base_link','link_base','link1'):continue
                if intersects(a,robot):return False,'SUPPORT_ROBOT_COLLISION:'+robot.body
            for s in self.inner.scene:
                if intersects(a,s):return False,'SUPPORT_SCENE_COLLISION:'+s.path
        return True,'SAFE'

    def contact_guard(self,contacts,poses):return self.inner.contact_guard(contacts,poses)
