"""Bounded geometric acquisition/retention/task ranking; no execution gates.

Ray support is a geometric feature, not pressure or success probability.
The declared friction cone is an assumed direction model, never measured force.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from native_g1_initial_contact_ranking import initial_contact_quality
from handle_contact_quality import ray_surface

LABELS=('gripper_link1','gripper_link2')

def diverse_shortlist(geometries,limit=18):
    ranked=sorted(geometries,key=lambda g:(g['contact']['rank'],initial_contact_quality(g['contact'])['score']),reverse=True)
    picked=[]
    def add(g):
        if len(picked)<limit and not any(g is x for x in picked):picked.append(g)
    # Same family coverage on every object: section, depth, roll, pitch.
    for key in ['section','insertion_m','roll','pitch']:
        for value in sorted({g[key] for g in ranked}):add(next(g for g in ranked if g[key]==value))
    for g in ranked:add(g)
    return picked

class RobustContactRank:
    def __init__(self,model,surfaces,base,hinge_origin,hinge_axis,config):
        self.model=model;self.surfaces=surfaces;self.base=base;self.origin=np.asarray(hinge_origin);self.axis=np.asarray(hinge_axis);self.cfg=config;self.width_cache={};self.grids={};self.transforms=[]
        for n in LABELS:
            b=model.pads[n];xy=np.array([[x,y] for x in np.linspace(*b[:,0],config['pad_grid'][0]) for y in np.linspace(*b[:,1],config['pad_grid'][1])]);grid=np.c_[xy,np.full(len(xy),b[0,2])];long=np.minimum(xy[:,1]-b[0,1],b[1,1]-xy[:,1]);lat=np.minimum(xy[:,0]-b[0,0],b[1,0]-xy[:,0]);ls=(b[1,1]-b[0,1])*config['retention_longitudinal_scale_fraction_pad_depth'];xs=(b[1,0]-b[0,0])*config['retention_lateral_scale_fraction_pad_width'];self.grids[n]=(grid,long,lat,long/(long+ls),lat/(lat+xs))
        self.transforms.append(('nominal',np.eye(4),0.))
        for k in range(3):
            for sign in [-1,1]:
                E=np.eye(4);E[k,3]=sign*config['translation_m'];self.transforms.append((f'translation_{k}_{sign}',E,0.))
                E=np.eye(4);v=np.zeros(3);v[k]=np.deg2rad(sign*config['rotation_deg']);E[:3,:3]=Rotation.from_rotvec(v).as_matrix();self.transforms.append((f'rotation_{k}_{sign}',E,0.))
        for sign in [-1,1]:self.transforms.append((f'aperture_{sign}',np.eye(4),sign*config['aperture_m']))
    def frames(self,width):
        w=float(np.clip(width,.001641,.084));key=round(w,10)
        if key not in self.width_cache:
            P=self.model.poses(self.model.home,self.base,width=w);I=np.linalg.inv(P['tcp_link']);self.width_cache[key]={n:I@P[n] for n in LABELS}
        return self.width_cache[key]
    def sample(self,T,width,mu):
        P=self.frames(width);side=[];normals=[];capacity=[]
        for n in LABELS:
            grid,long,lat,wl,wx=self.grids[n];F=T@P[n];origins=(np.c_[grid,np.ones(len(grid))]@F.T)[:,:3];direction=-F[:3,2]
            # Start outside the handle to preserve signed face gap after small
            # overlap; casting from inside would incorrectly select the far face.
            backoff=.1;rays=[ray_surface(m,origins-backoff*direction,direction) for m in self.surfaces[1]];dist=np.array([z[0] for z in rays]);ids=dist.argmin(0);gap=dist[ids,np.arange(len(grid))]-backoff;normal=np.array([z[1] for z in rays])[ids,np.arange(len(grid))];hit=np.isfinite(gap);alignment=np.clip(-normal@direction,0,1);weights=np.where(hit,np.exp(-np.minimum(abs(gap)/self.cfg['support_gap_band_m'],30)**2),0)*alignment**2;den=max(weights.sum(),1e-30)
            support=float(weights.mean());longscore=float(weights@wl/den);latscore=float(weights@wx/den);retention=float(longscore**(2/3)*latscore**(1/3));point=origins+np.where(hit,gap,0)[:,None]*direction;r=point-self.origin;tangent=np.cross(self.axis,r);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-30)
            if self.model.manifest.get('joint_type','revolute')=='prismatic':
                tangent=np.broadcast_to(self.axis/np.linalg.norm(self.axis),point.shape)
            normal_projection=np.einsum('ij,ij->i',-normal,tangent);cone=np.maximum(0,normal_projection+mu*np.sqrt(np.maximum(0,1-normal_projection**2)))/np.sqrt(1+mu**2);cap=float(weights@cone/den);cap0=float(weights@np.maximum(0,normal_projection)/den);mean=(weights[:,None]*normal).sum(0)/den;mean/=max(np.linalg.norm(mean),1e-30);normals.append(mean);capacity.append(cap)
            side.append({'finger':n,'support':support,'longitudinal_retention':longscore,'lateral_retention':latscore,'retention':retention,'longitudinal_margin_mm':float(weights@long/den*1000),'lateral_margin_mm':float(weights@lat/den*1000),'opening_capacity_assumed_mu':cap,'opening_capacity_mu0':cap0,'geometric_ray_support_area_proxy_m2':float(np.prod(self.model.pads[n][1,:2]-self.model.pads[n][0,:2])*support)})
        opposition=float(np.clip((1-normals[0]@normals[1])/2,0,1));contact=min(z['support'] for z in side)*opposition;retention=min(z['retention'] for z in side);task=max(capacity);score=contact*retention*task
        return {'contact_score':contact,'retention_score':retention,'task_score':task,'combined':score,'opening_capacity_mu0':max(z['opening_capacity_mu0'] for z in side),'sides':side}
    def evaluate(self,g):
        quality=g['contact'];old=initial_contact_quality(quality);first=quality['first_contacts'];widths=[x['width_m'] for x in first.values()];entry=max(widths,default=.084);bilateral=min(widths,default=entry);low=max(.001641,bilateral-self.cfg['aperture_m']);high=min(.084,entry+self.cfg['aperture_m']);trials=[]
        for width in np.linspace(low,high,self.cfg['aperture_samples']):
            samples=[]
            for label,D,dw in self.transforms:
                sample=self.sample(g['T']@D,float(width+dw),self.cfg['friction_cone_assumption_mu']);sample.update(perturbation=label,common_aperture_m=float(np.clip(width+dw,.001641,.084)));samples.append(sample)
            values=np.array([z['combined'] for z in samples]);robust=float(self.cfg['mean_weight']*values.mean()+self.cfg['lower_quartile_weight']*np.quantile(values,.25));trials.append({'width_m':float(width),'robust_score':robust,'mean':float(values.mean()),'lower_quartile':float(np.quantile(values,.25)),'nominal':samples[0],'samples':samples})
        chosen=max(trials,key=lambda z:z['robust_score']);nominal_choice=max(trials,key=lambda z:z['nominal']['combined']);mismatch=abs(widths[0]-widths[1]) if len(widths)==2 else .084
        # Bounded centering already permits 2 mm; mismatch is a soft feature,
        # never a new bilateral/contact acceptance requirement.
        accessibility=(quality['pad_fingers']/2)*np.exp(-mismatch/.008)/max(1.,1+quality['root_contact_count']);accessibility*=float(quality['non_handle_closing_collision'] is None)
        score=float(accessibility*chosen['robust_score'])
        report={'acquisition_score':float(accessibility),'frozen_initial_contact_feature':old,'retention_score':chosen['nominal']['retention_score'],'task_score':chosen['nominal']['task_score'],'robust_mean':chosen['mean'],'robust_lower_quartile':chosen['lower_quartile'],'robust_combined_score':score,'chosen_expected_aperture_m':chosen['width_m'],'nominal_best_expected_aperture_m':nominal_choice['width_m'],'uncertainty_changes_aperture_choice':abs(chosen['width_m']-nominal_choice['width_m'])>1e-9,'aperture_trials':trials,'classification':'APPROXIMATE_GEOMETRIC_RANK_ONLY','sample_failures_are_not_vetoes':True,'friction_assumption_mu':self.cfg['friction_cone_assumption_mu'],'pressure_or_calibrated_success_probability':False}
        return report

def rescore_geometries(geometries,model,surfaces,base,origin,axis,config,output):
    start=time.perf_counter();short=diverse_shortlist(geometries,config['shortlist_max']);scorer=RobustContactRank(model,surfaces,base,origin,axis,config);ranked=[]
    for i,g in enumerate(short):
        old=dict(g['contact']);r=scorer.evaluate(g);g['contact']['robust_contact']=r;g['contact']['ideal_support_score']=old['support_score'];g['contact']['support_score']=r['robust_combined_score'];g['contact']['physical_aperture_m']=r['chosen_expected_aperture_m'];g['contact']['rank']=old['rank']-old['support_score']+r['robust_combined_score'];ranked.append(g)
        print('ROBUST_GEOMETRY',i+1,len(short),g['section'],g['insertion_m'],g['roll'],g['pitch'],r['robust_combined_score'],flush=True)
    ranked.sort(key=lambda g:g['contact']['support_score'],reverse=True)
    summary={'wall_s':time.perf_counter()-start,'family_count':len(geometries),'shortlist_count':len(short),'configuration':config,'ranking':[{'section':g['section'],'insertion_m':g['insertion_m'],'roll':g['roll'],'pitch':g['pitch'],'T':np.asarray(g['T']).tolist(),'scores':g['contact']['robust_contact']} for g in ranked]}
    Path(output).mkdir(exist_ok=True,parents=True);(Path(output)/'robust_geometry_audit.json').write_text(json.dumps(summary,indent=2));return ranked
