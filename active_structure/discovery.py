"""Coarse hypotheses are permission to gather information, never final acceptance."""
import copy,json
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_identification.act2see_loop import InteractionMemory,unit
from interaction_identification.fitting import fit_articulation
from interactive_twin_refinement.fitting import fit_se3


def hypotheses(poses,policy):
    T=np.asarray(poses,float)
    if len(T)<24:return {'status':'UNOBSERVABLE','reason':'sample_count','candidates':[]}
    T=T[np.unique(np.linspace(0,len(T)-1,min(240,len(T))).astype(int))]
    center=fit_articulation(T);r=center.get('revolute',{})
    if not r:return {'status':'UNOBSERVABLE','reason':'insufficient_translation','candidates':[]}
    a=np.asarray(r['axis']);candidates=[center];records=[]
    blocks=np.array_split(np.arange(len(T)),4)
    samples=[('first_75',T[:int(.75*len(T))]),('last_75',T[int(.25*len(T)):])]
    rng=np.random.default_rng(policy['bootstrap_seed'])
    for i in range(3):
        ids=np.sort(np.concatenate([blocks[k] for k in rng.integers(0,4,4)]))
        samples.append((f'block_bootstrap_{i}',T[ids]))
    for name,X in samples:
        f=fit_articulation(X)
        if 'revolute' not in f:continue
        aa=np.asarray(f['revolute']['axis']);angle=float(np.rad2deg(np.arccos(np.clip(abs(a@aa),0,1))))
        records.append({'name':name,'axis_delta_deg':angle,'radius_m':f['revolute']['radius_m'],'type':f['joint_type']})
        candidates.append(f)
    radii=np.asarray([f['revolute']['radius_m'] for f in candidates])
    tangents=[]
    for f in candidates:
        rr=f['revolute'];aa=np.asarray(rr['axis']);aa*=1 if aa@a>=0 else -1
        tangents.append(unit(np.cross(aa,T[-1,:3,3]-rr['point_on_axis'])))
    tangent_spread=max(float(np.rad2deg(np.arccos(np.clip(tangents[0]@t,-1,1)))) for t in tangents)
    checks={'useful_motion':center['travel_m']>=.005,
            'revolute_evidence':center['joint_type']=='revolute' and center['confidence']>=policy['minimum_model_score_gap'],
            'measured_rotation':np.rad2deg(r['angle_span_rad'])>=policy['minimum_rotation_deg'],
            'segment_axes':len(records)>=4 and max(x['axis_delta_deg'] for x in records)<=policy['maximum_axis_dispersion_deg'],
            'radius_plausible':bool(np.all((radii>=policy['radius_range_m'][0])&(radii<=policy['radius_range_m'][1]))),
            'radius_dispersion':float(np.std(radii)/max(np.mean(radii),1e-9))<=policy['maximum_radius_cv'],
            'tangent_dispersion':tangent_spread<=policy['maximum_tangent_dispersion_deg'],
            'orientation_residual':r['rotation_rmse_rad']<=policy['maximum_provisional_rotation_residual_rad']}
    checks={key:bool(value) for key,value in checks.items()}
    strict=bool(center['joint_type']=='revolute' and center['confidence']>.9 and r['position_rmse_m']<.0003 and r['rotation_rmse_rad']<.001 and r['angle_span_rad']>np.deg2rad(1.))
    return {'status':'ACCEPTED' if strict else ('PROVISIONAL_REVOLUTE' if all(checks.values()) else 'UNOBSERVABLE'),
            'checks':checks,'strict_acceptance':strict,'fit':center,'candidates':candidates,
            'segments':records,'tangent_dispersion_deg':tangent_spread,'GT_used':False}


class ProvisionalMemory(InteractionMemory):
    def __init__(self,T,output,policy):
        self.policy=policy;self.discovery_status='UNOBSERVABLE';self.hypotheses=[];self.last_evidence={};self.segment_count=0
        super().__init__(T,output)

    def try_fit(self):
        if len(self.poses)<24 or self.travel()<.005:return None
        evidence=hypotheses(self.poses,self.policy['discovery']);self.last_evidence=evidence
        fit=evidence.get('fit')
        self.estimates.append({'fit':fit,'accepted':evidence['status']=='ACCEPTED',
                              'provisional':evidence['status']=='PROVISIONAL_REVOLUTE','observation_count':len(self.poses)})
        if evidence['status']!='UNOBSERVABLE':
            a=np.asarray(fit['revolute']['axis']);rv=Rotation.from_matrix(np.asarray(self.poses)[:,:3,:3]@self.initial[:3,:3].T).as_rotvec()
            self.follow_sign=1. if (rv[-1]-rv[0])@a>=0 else -1.
            # The existing joint fitter estimates a latent reference pose.
            # Use that complete SE(3) model for prediction; substituting the
            # noisy first measurement would discard its reference rotation
            # and create radius-amplified false consistency error.
            joint_fit=fit_se3(self.poses,**self.policy['fitting'])
            self.estimate=joint_fit if joint_fit['optimizer']['success'] else fit
            if np.asarray(self.estimate['revolute']['axis'])@a<0:self.follow_sign*=-1.
            self.hypotheses=evidence['candidates'];self.discovery_status=evidence['status']
        (self.output/'provisional_evidence.json').write_text(json.dumps(evidence,indent=2))
        self.save();return fit

    def consistency_error(self,T):
        if self.estimate is None:return None
        if 'reference_pose' not in self.estimate:return super().consistency_error(T)
        reference=np.asarray(self.estimate['reference_pose']);r=self.estimate['revolute']
        axis=np.asarray(r['axis']);center=np.asarray(r['point_on_axis'])
        angle=float(Rotation.from_matrix(T[:3,:3]@reference[:3,:3].T).as_rotvec()@axis)
        predicted=center+Rotation.from_rotvec(angle*axis).apply(reference[:3,3]-center)
        return float(np.linalg.norm(T[:3,3]-predicted))

    def refresh(self):
        # No parameter/threshold changes. Same robust fitter, now actually
        # reached during bounded refinement. Phase and reference are nuisance
        # variables estimated by that fitter, not a forced first observation.
        self.segment_count+=1
        e=hypotheses(self.poses,self.policy['discovery']);self.last_evidence=e
        if e['status']!='UNOBSERVABLE':self.hypotheses=e['candidates']
        if self.segment_count%self.policy['refinement']['robust_every_segments']==0:
            f=fit_se3(self.poses,**self.policy['fitting'])
            if f['optimizer']['success']:
                old=np.asarray(self.estimate['revolute']['axis']);new=np.asarray(f['revolute']['axis'])
                if old@new<0:self.follow_sign*=-1
                self.estimate=f
        self.save()

    def choose_segment(self,T,q,model,collision,base,moving,grasp_tcp,moving_initial):
        models=[self.estimate]+self.hypotheses[:5];axis=np.asarray(self.estimate['revolute']['axis'])
        tangents=[];signed=[]
        for f in models:
            r=f['revolute'];a=np.asarray(r['axis']);a*=1 if a@axis>=0 else -1
            rr=T[:3,3]-np.asarray(r['point_on_axis']);rad=np.linalg.norm(np.cross(a,rr))
            tangents.append(self.follow_sign*unit(np.cross(a,rr)));signed.append((a,max(rad,.001)))
        directions=[unit(np.mean(tangents,axis=0)),*tangents[:3]];valid=[];audits=[]
        ds=self.policy['refinement']['segment_m'];arm=np.asarray(q[:6],float)
        for i,d in enumerate(directions):
            excitation=[float(d@t/rad) for t,(_,rad) in zip(tangents,signed)]
            if min(excitation)<=0:audits.append({'candidate':i,'reason':'HYPOTHESIS_DIRECTION_CONFLICT'});continue
            target=T.copy();target[:3,3]+=d*ds
            target[:3,:3]=Rotation.from_rotvec(self.follow_sign*axis*min(excitation)*ds).as_matrix()@T[:3,:3]
            solution=model.ik(target,base,arm,starts=1)
            if solution is None:audits.append({'candidate':i,'reason':'NO_INCREMENTAL_IK'});continue
            qq=np.asarray(solution,float)
            margin=model.margin(qq)
            if margin<=.05:audits.append({'candidate':i,'reason':'LOW_JOINT_MARGIN'});continue
            poses=model.poses(qq,base,finger_q=q[6:8])
            body=target@np.linalg.inv(grasp_tcp)@moving_initial
            ok,why=collision.check(poses,body,True)
            if not ok:audits.append({'candidate':i,'reason':why});continue
            score=min(excitation)
            audits.append({'candidate':i,'reason':'SAFE','minimum_predicted_rotation_rad':score*ds,'margin_rad':margin})
            valid.append((score,margin,d))
        if not valid:return None,audits
        return max(valid,key=lambda x:(x[0],x[1]))[-1],audits
