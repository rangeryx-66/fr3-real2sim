"""Class-independent support transition and physical outcome measurement.

No object identifiers, rigid constraints, grasp poses, or controller commands.
The lowest convex-hull vertex is exactly the minimum over the input GT mesh.
Only free-space samples contribute to slip/stability and vertical following.
"""
from collections import deque
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from settling_gate import compute_metrics, relative_pose, quat_angle_deg

CONFIG=dict(clearance_m=.002,clear_dwell_s=.100,contact_N=.1,contact_gap_s=.050,
            settle_min_s=.5,settle_max_s=1.,stable_dwell_s=.3,
            translation_velocity_m_s=.00025,angular_velocity_deg_s=.5,
            following_residual_m=.001,min_vertical_follow_ratio=.8,
            free_cumulative_translation_m=.010,free_cumulative_rotation_deg=15.,
            lift_goal_m=.085,success_height_m=.080,hold_s=2.2,
            lift_step_m=.005,lift_speed_m_s=.010,max_tcp_travel_m=.250)

class SupportMonitor:
    def __init__(self,vertices):
        v=np.asarray(vertices,dtype=float)
        self.height_reference_local=(v.min(0)+v.max(0))/2
        self.vertices=v[ConvexHull(v).vertices]
        self.rows=deque(maxlen=1440);self.reset()

    def reset(self):
        self.rows.clear();self.armed=False;self.clear_since=None;self.clear_t=None
        self.free_reference=None;self.close_reference=None;self.initial_z=None
        self.contact_gap=0.;self.max_contact_gap=0.;self.ever_clear=False
        self.peak_lift=0.;self.picked=False;self.ever_contact_loss=False
        self.last_t=None;self.last=None;self.post_clear_peak_z=None;self.dropped=False
        self.support_returns=0
        self.cumulative_translation=0.;self.cumulative_rotation=0.

    def begin(self,initial_z):
        self.reset();self.armed=True;self.initial_z=float(initial_z)

    def sample(self,t,bp,bq,tp,tq,forces):
        R=Rotation.from_quat(np.roll(bq,-1)).as_matrix()
        bottom=float(np.min(self.vertices@R[2])+bp[2])
        origin=np.asarray(bp,dtype=float)
        # USD origins differ across assets. Use one geometric reference rule;
        # this is identical to the old origin metric for centered development assets.
        bp=origin+R@self.height_reference_local
        row=dict(t=float(t),box=np.asarray(bp).tolist(),box_quat=np.asarray(bq).tolist(),
                 object_origin_world_m=origin.tolist(),
                 tcp=np.asarray(tp).tolist(),tcp_quat=np.asarray(tq).tolist(),
                 forces=np.asarray(forces).tolist(),mesh_bottom_z_m=bottom)
        self.last=row
        if not self.armed:return
        dt=0. if self.last_t is None else t-self.last_t;self.last_t=t
        bilateral=min(forces)>CONFIG['contact_N']
        self.contact_gap=0. if bilateral else self.contact_gap+dt
        self.max_contact_gap=max(self.max_contact_gap,self.contact_gap)
        self.ever_contact_loss |= self.contact_gap>CONFIG['contact_gap_s']
        lift=float(bp[2])-self.initial_z;self.peak_lift=max(self.peak_lift,lift)
        clear=bottom>CONFIG['clearance_m']
        # Clearance is a current physical state, not a one-way episode flag.
        # A held object can briefly clear and then pivot back onto the table.
        if not clear and self.clear_t is not None:
            self.clear_t=None;self.free_reference=None;self.support_returns+=1
        self.clear_since=(t if self.clear_since is None else self.clear_since) if clear else None
        if self.clear_t is None and self.clear_since is not None and t-self.clear_since>=CONFIG['clear_dwell_s']:
            self.clear_t=float(t);self.free_reference=relative_pose(row);self.ever_clear=True
            self.post_clear_peak_z=max(float(bp[2]),self.post_clear_peak_z or float(bp[2]))
        self.picked |= bool(clear and bilateral)
        if self.close_reference is None:self.close_reference=relative_pose(row)
        rp,rq=relative_pose(row)
        self.cumulative_translation=max(self.cumulative_translation,float(np.linalg.norm(rp-self.close_reference[0])))
        self.cumulative_rotation=max(self.cumulative_rotation,quat_angle_deg(self.close_reference[1],rq))
        if self.ever_clear:
            self.post_clear_peak_z=max(self.post_clear_peak_z,float(bp[2]))
            # A height shortfall or held reorientation cannot produce DROP.
            self.dropped |= bool(self.ever_contact_loss and bottom<=.001 and self.post_clear_peak_z-float(bp[2])>.005)
        self.rows.append(row)

    def query(self,since_t=None):
        row=self.last
        if row is None:return dict(ready=False)
        bilateral=min(row['forces'])>CONFIG['contact_N']
        bottom=row['mesh_bottom_z_m'];net=0. if self.initial_z is None else row['box'][2]-self.initial_z
        flags=dict(PICKED=bool(self.picked),RETAINED=bool(self.ever_clear and bottom>.002 and bilateral),
                   CLEAR_TABLE=bool(self.ever_clear),lift_ge_8cm=bool(net>=.080),DROP=bool(self.dropped))
        result=dict(ready=self.armed,t=row['t'],mesh_bottom_z_m=bottom,currently_clear=bottom>.002,
                    clear_t=self.clear_t,net_lift_m=net,peak_lift_m=self.peak_lift,flags=flags,
                    support_returns=self.support_returns,
                    height_reference_local_m=self.height_reference_local.tolist(),
                    bilateral=bilateral,contact_gap_s=self.contact_gap,max_contact_gap_s=self.max_contact_gap,
                    cumulative_translation_m=self.cumulative_translation,cumulative_rotation_deg=self.cumulative_rotation,
                    passed=False,category='SUPPORTED_SETTLING',free_metrics=None)
        if self.dropped:result['category']='DROP';return result
        if self.max_contact_gap>CONFIG['contact_gap_s']:result['category']='CONTACT_LOSS';return result
        if not self.armed or self.clear_t is None:return result
        # The reference resets at actual clearance, never at closure or fixed TCP travel.
        begin=max(self.clear_t,float(since_t)) if since_t is not None else self.clear_t
        rows=[r for r in self.rows if r['t']>=begin]
        if len(rows)<3 or rows[-1]['t']-rows[0]['t']<.295:
            result['category']='FREE_SPACE_SETTLE';return result
        m=compute_metrics(rows,reference=self.free_reference)
        result['free_metrics']=m
        tail=[r for r in rows if r['t']>=rows[-1]['t']-.3]
        stable=True;last_reason='STABLE'
        # Require the frozen 200 ms velocity bounds over a full 300 ms dwell.
        for age in [0.,.1,.2,.3]:
            stop=rows[-1]['t']-age
            window=[r for r in rows if stop-.2001<=r['t']<=stop+1e-8]
            if len(window)<40:stable=False;last_reason='FREE_SPACE_SETTLE';continue
            w=compute_metrics(window)['windows']['200ms']
            if w['relative_angular_velocity_deg_s']>CONFIG['angular_velocity_deg_s']:
                stable=False;last_reason='ROTATIONAL_INSTABILITY'
            elif w['relative_translation_velocity_m_s']>CONFIG['translation_velocity_m_s']:
                stable=False;last_reason='CONTINUOUS_SLIP'
        if not bilateral or not all(min(r['forces'])>.1 for r in tail):
            stable=False;last_reason='CONTACT_LOSS'
        if m['max_cumulative_rotation_deg']>CONFIG['free_cumulative_rotation_deg']:
            stable=False;last_reason='ROTATIONAL_INSTABILITY'
        if m['max_cumulative_translation_m']>CONFIG['free_cumulative_translation_m']:
            stable=False;last_reason='CONTINUOUS_SLIP'
        if since_t is not None and abs(m['tcp_delta_m'][2])>.001:
            if m['vertical_follow_ratio']<CONFIG['min_vertical_follow_ratio'] or m['following_residual_m']>CONFIG['following_residual_m']:
                stable=False;last_reason='VERTICAL_FOLLOW_FAIL'
        result['passed']=stable
        result['category']=('SETTLING_THEN_STABLE' if self.cumulative_translation>.005 or self.cumulative_rotation>5. else 'STABLE') if stable else last_reason
        return result

    def hold_outcome(self,start_t):
        rows=[r for r in self.rows if r['t']>=start_t]
        if not rows:return dict(success=False,category='INSUFFICIENT_LIFT')
        duration=rows[-1]['t']-rows[0]['t']
        height=np.asarray([r['box'][2]-self.initial_z for r in rows])
        bottom=np.asarray([r['mesh_bottom_z_m'] for r in rows])
        bilateral=all(min(r['forces'])>.1 for r in rows)
        q=self.query();category='SUCCESS'
        if q['flags']['DROP']:category='DROP'
        elif not bilateral:category='CONTACT_LOSS'
        elif min(height)<.08 or duration<2. or min(bottom)<=.002:category='INSUFFICIENT_LIFT'
        elif np.ptp(height)>.010:category='CONTINUOUS_SLIP'
        elif not q['passed']:category=q['category']
        return dict(success=category=='SUCCESS',category=category,duration_s=duration,
                    min_lift_m=float(min(height)),max_lift_m=float(max(height)),bilateral=bilateral,
                    min_mesh_bottom_m=float(min(bottom)),stability=q)
